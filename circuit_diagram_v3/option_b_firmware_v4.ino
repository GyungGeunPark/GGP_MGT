/*
 * ============================================================================
 *  JRT U81 무선 레이저 거리측정 — ESP32-C3 통합 펌웨어 v4
 * ============================================================================
 *  하드웨어: JRT U81 (3.3V UART) + ESP32-C3 Super Mini + Pololu S7V8F3 + 2×AA
 *  통신    : BLE(NimBLE NUS) + WiFi UDP 통합. 런타임 모드 전환.
 *  v4 반영 (option_b_v3_리뷰_개선사항.md / option_b_guide_v4.md):
 *    - 센서 EN: GPIO2(스트래핑) → GPIO10
 *    - 배터리: analogReadMilliVolts() eFuse 캘리브레이션, battV = adcV*2.0 (100k/100k)
 *    - 저전압 2단계: 2.90V 경고(WiFi→BLE 폴백) / 2.80V 안전 종료(딥슬립)
 *    - 부팅 직후 ADC 필터 정착 대기 + 첫 측정값 폐기
 *    - 센서 UART는 UART1로 매핑(부팅 로그가 GPIO20/21로 새지 않도록)
 *
 *  보드 : nologo_esp32c3_super_mini  (Arduino-ESP32 core v3.x)
 *  라이브러리: NimBLE-Arduino (v2.x)
 *  ※ "USB CDC On Boot" = Enabled (디버그 Serial는 USB)
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NimBLEDevice.h>
#include <esp_sleep.h>
#include "driver/gpio.h"   // gpio_hold_en / gpio_deep_sleep_hold_en (딥슬립 EN 유지)
#include <math.h>

// ─────────────────────────── 사용자 설정 ───────────────────────────
// 기본 통신 모드: MODE_BLE 또는 MODE_WIFI
#define DEFAULT_COMM_MODE   MODE_BLE

// WiFi 설정 (WiFi 모드 사용 시)
static const char* WIFI_SSID = "YourNetworkSSID";        // ← 수정
static const char* WIFI_PASS = "YourNetworkPassword";    // ← 수정
static const IPAddress PC_IP(192, 168, 1, 100);          // ← 수신 PC IP
static const uint16_t  UDP_PORT = 4210;

// BLE 설정
#define BLE_DEVICE_NAME     "JRT-U81-LDS"
#define NUS_SERVICE_UUID    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID         "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // Notify (ESP→PC)
#define NUS_RX_UUID         "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Write  (PC→ESP)

// 핀 정의 (v4)
// ※ GPIO20/21은 ESP32-C3의 기본 UART0(U0RXD/U0TXD) 핀이자 보드 실크의 RX/TX.
//   리셋 순간 ROM 부트로더가 GPIO21(TX)로 로그를 토할 수 있어 센서 RX로 유입될 수 있다.
//   → 본 설계의 지정 핀이므로 유지하되, 부팅 후 sensorFlush()로 잔여 바이트를 비운다.
//   (센서는 19200/ASCII 명령 파서라 115200 부트노이즈는 프레이밍 에러로 폐기됨)
#define SENSOR_RX_PIN       20      // ESP32 RX  ← 센서 TX (초록)
#define SENSOR_TX_PIN       21      // ESP32 TX  → 센서 RX (파랑)
// ★v4: EN을 GPIO2(스트래핑)에서 GPIO10으로 이설. GPIO10은 C3의 자유 GPIO이다
//      (프로그램 플래시는 GPIO12~17 사용, GPIO10은 무관 — 데이터시트/핀맵 확인).
#define SENSOR_EN_PIN       10
#define BATTERY_ADC_PIN     3       // ADC1_CH3 (보라)
#define LED_PIN             8       // 온보드 LED

// 타이밍
#define SENSOR_BAUD         19200
// U81 단일측정('D')은 타겟 반사율/거리에 따라 ~0.1~1s 소요. 응답 대기를 넉넉히 두고
// 측정 주기 ≥ 응답 시간으로 설정(실효 속도는 센서 한계 ~2~8Hz에 종속).
#define MEASURE_INTERVAL_MS 700     // 측정 주기 (BLE)
#define MEASURE_INTERVAL_WIFI 500   // 측정 주기 (WiFi)
#define BATTERY_CHECK_MS    30000   // 배터리 점검 주기 (30초)
#define SENSOR_RESP_MS      600     // ★v4: 'D' 응답 대기 (구 80ms는 과소 → 측정 실패)

// 배터리 (Option B: 100k/100k 1:1 분배기, S7V8F3 UVLO ≈ 2.7V)
#define BATT_DIVIDER_RATIO  2.0     // Vbat = Vadc * 2.0
#define LOW_BATT_WARN_B     2.90    // 경고: WiFi 끄고 BLE 전환
#define LOW_BATT_SHUTDOWN_B 2.80    // 안전 종료: 레이저 OFF + 딥슬립
#define BATT_ADC_SAMPLES    16

enum CommMode { MODE_BLE, MODE_WIFI };

// ─────────────────────────── 전역 상태 ───────────────────────────
HardwareSerial LaserSerial(1);          // UART1을 GPIO20/21에 매핑
WiFiUDP        udp;

CommMode  commMode        = DEFAULT_COMM_MODE;
bool      bleConnected    = false;
bool      lowBattLatched  = false;      // 경고 후 WiFi 재진입 방지
uint32_t  lastMeasure     = 0;
uint32_t  lastBattCheck   = 0;
float     lastBatteryV    = 0.0f;

NimBLEServer*         pServer = nullptr;
NimBLECharacteristic* pTxChar = nullptr;

// ─────────────────────────── 유틸 ───────────────────────────
static inline uint32_t measureIntervalMs() {
    return (commMode == MODE_WIFI) ? MEASURE_INTERVAL_WIFI : MEASURE_INTERVAL_MS;
}

void ledBlink(uint32_t period_ms) {
    digitalWrite(LED_PIN, (millis() / period_ms) % 2);
}

// ─────────────────────── 배터리 (v4 캘리브레이션) ───────────────────────
float readBatteryVoltage() {
    uint32_t sum_mv = 0;
    for (int i = 0; i < BATT_ADC_SAMPLES; i++) {
        sum_mv += analogReadMilliVolts(BATTERY_ADC_PIN);  // eFuse 보정값(mV)
        delayMicroseconds(200);
    }
    float adcV = (sum_mv / (float)BATT_ADC_SAMPLES) / 1000.0f;  // mV → V
    return adcV * BATT_DIVIDER_RATIO;                          // Option B
}

// ─────────────────────── 센서 명령/측정 ───────────────────────
void sensorFlush() {
    while (LaserSerial.available()) LaserSerial.read();
}

void sensorWriteCmd(char c) {
    sensorFlush();
    LaserSerial.write((uint8_t)c);
}

int g_lastSQ = -1;   // 최근 신호품질(SQ, 낮을수록 좋음). 없음 = -1

// 거리(mm) 반환. 실패 시 -1. 부수효과: g_lastSQ 갱신.
float measureDistanceMM() {
    g_lastSQ = -1;
    sensorWriteCmd('D');                 // 단일 측정
    uint32_t t0 = millis();
    while (!LaserSerial.available() && (millis() - t0) < SENSOR_RESP_MS) {
        delay(1);
    }
    if (!LaserSerial.available()) return -1.0f;

    String resp = LaserSerial.readStringUntil('\n');
    resp.trim();
    int mIdx = resp.indexOf('m');
    if (mIdx <= 0) return -1.0f;         // 'm' 없음 → 에러/범위초과
    float dist_m = resp.substring(0, mIdx).toFloat();
    if (dist_m <= 0.0f) return -1.0f;
    // 응답 "X.XXXm, SQ" 에서 콤마 뒤 신호품질 파싱
    int cIdx = resp.indexOf(',', mIdx);
    if (cIdx > 0) g_lastSQ = resp.substring(cIdx + 1).toInt();
    return dist_m * 1000.0f;
}

// ─────────────────────── BLE 스택 ───────────────────────
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
        bleConnected = true;
        Serial.println("[BLE] 연결됨");
        s->updateConnParams(info.getConnHandle(), 6, 12, 0, 400);
    }
    void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
        bleConnected = false;
        Serial.printf("[BLE] 연결 해제 (reason=%d)\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
        std::string v = c->getValue();
        if (!v.empty()) {                // PC → 센서 명령 패스스루
            LaserSerial.write((uint8_t)v[0]);
            Serial.printf("[BLE] 센서 명령 전달: 0x%02X\n", v[0]);
        }
    }
};

void bleBegin() {
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(128);
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* svc = pServer->createService(NUS_SERVICE_UUID);
    pTxChar = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
    NimBLECharacteristic* rx =
        svc->createCharacteristic(NUS_RX_UUID,
                                  NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rx->setCallbacks(new RxCallbacks());
    svc->start();

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(NUS_SERVICE_UUID);
    adv->setName(BLE_DEVICE_NAME);
    adv->start();
    Serial.printf("[BLE] 광고 시작: '%s'\n", BLE_DEVICE_NAME);
}

void bleStop() {
    NimBLEDevice::deinit(true);
    pServer = nullptr; pTxChar = nullptr; bleConnected = false;
}

// ─────────────────────── WiFi 스택 ───────────────────────
bool wifiBegin() {
    Serial.printf("[WiFi] 연결 중: %s\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 40) {
        delay(500); ledBlink(250); tries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] 연결 완료! IP: %s\n", WiFi.localIP().toString().c_str());
        udp.begin(UDP_PORT);
        return true;
    }
    Serial.println("[WiFi] 연결 실패");
    return false;
}

void wifiStop() {
    udp.stop();
    WiFi.disconnect(true, false);
    WiFi.mode(WIFI_OFF);
}

// ─────────────────────── 통신 추상화 ───────────────────────
void commBegin(CommMode m) {
    if (m == MODE_BLE) {
        bleBegin();
    } else if (!wifiBegin()) {
        Serial.println("[COMM] WiFi 실패 → BLE 폴백");
        wifiStop();              // ★ RF/STA 자원 정리 후 BLE 시작 (누수 방지)
        commMode = MODE_BLE;
        bleBegin();
    }
}

bool commConnected() {
    return (commMode == MODE_BLE) ? bleConnected : (WiFi.status() == WL_CONNECTED);
}

// 15바이트 패킷 전송: [id][dist_mm f32][ts u32][battV f32][sq u16]  (리틀엔디안)
//   sq = 0xFFFF → 신호품질 미상(수신측은 -1로 처리)
void commSendPacket(float dist_mm, uint32_t ts, float battV, uint16_t sq) {
    uint8_t pkt[15];
    pkt[0] = 0x01;
    memcpy(&pkt[1], &dist_mm, 4);
    memcpy(&pkt[5], &ts, 4);
    memcpy(&pkt[9], &battV, 4);
    memcpy(&pkt[13], &sq, 2);

    if (commMode == MODE_BLE) {
        if (bleConnected && pTxChar) { pTxChar->setValue(pkt, 15); pTxChar->notify(); }
    } else {
        if (WiFi.status() == WL_CONNECTED) {
            udp.beginPacket(PC_IP, UDP_PORT);
            udp.write(pkt, 15);
            udp.endPacket();
        }
    }
}

// WiFi → BLE 런타임 폴백 (저전압 경고 시)
void fallbackToBLE() {
    if (commMode != MODE_WIFI) return;
    Serial.println("[COMM] 저전압 경고 → WiFi 종료, BLE 전환");
    wifiStop();
    commMode = MODE_BLE;
    bleBegin();
}

// 안전 종료: 레이저 OFF, 센서 슬립, 딥슬립 (사용자 전원 OFF까지 ~µA)
void safeShutdown() {
    Serial.println("[BATT] 안전 종료: 레이저 OFF + 딥슬립");
    sensorWriteCmd('C');                 // 레이저 OFF
    delay(50);
    digitalWrite(SENSOR_EN_PIN, LOW);    // 센서 슬립
    // SOS 깜빡임으로 사용자에게 표시
    for (int i = 0; i < 6; i++) { digitalWrite(LED_PIN, HIGH); delay(120);
                                  digitalWrite(LED_PIN, LOW);  delay(120); }
    if (commMode == MODE_WIFI) wifiStop(); else bleStop();
    // ★ 딥슬립 동안 EN을 LOW로 유지(센서 OFF 유지) — 미설정 시 핀이 풀려 센서가 다시 켜질 수 있음
    gpio_hold_en((gpio_num_t)SENSOR_EN_PIN);
    gpio_deep_sleep_hold_en();
    esp_deep_sleep_start();              // wake 소스 없음 → 전원 재투입 시 재시작
}

// ─────────────────────────── setup ───────────────────────────
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n=== JRT U81 LDS 펌웨어 v4 (BLE+WiFi 통합) ===");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);                 // 초기화 중 ON

    pinMode(SENSOR_EN_PIN, OUTPUT);
    digitalWrite(SENSOR_EN_PIN, HIGH);           // 센서 활성화 (GPIO10)

    analogSetAttenuation(ADC_11db);              // 0~3.3V 범위 (analogReadMilliVolts 보정)

    LaserSerial.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
    LaserSerial.setTimeout(SENSOR_RESP_MS);      // ★v4: readStringUntil 기본 1s 블로킹 방지
    delay(300);                                  // ★v4: 센서 초기화 + ADC 필터 정착
    sensorFlush();                               // 부팅 UART0 노이즈 등 잔여 바이트 제거

    LaserSerial.write('O');                      // 레이저 ON
    delay(100);
    sensorFlush();

    // ★v4: 첫 1~2회 배터리 측정은 폐기 (필터 정착 전이면 부정확)
    (void)readBatteryVoltage();
    delay(50);
    (void)readBatteryVoltage();
    lastBatteryV = readBatteryVoltage();
    Serial.printf("[BATT] 초기 전압: %.2fV (모드=%s)\n",
                  lastBatteryV, commMode == MODE_BLE ? "BLE" : "WiFi");

    // 부팅 시 배터리 상태에 따라 즉시 처리 (★v4: 경고 단계도 부팅 시 적용)
    if (lastBatteryV > 0.5f) {
        if (lastBatteryV < LOW_BATT_SHUTDOWN_B) {
            safeShutdown();                      // 종료 임계 이하 → 바로 안전 종료
        } else if (lastBatteryV < LOW_BATT_WARN_B) {
            lowBattLatched = true;               // 경고 이하 → WiFi 진입 금지, BLE 강제
            commMode = MODE_BLE;
            Serial.println("[BATT] 부팅 시 저전압 경고 → BLE 강제");
        }
    }

    commBegin(commMode);
    digitalWrite(LED_PIN, LOW);                  // 초기화 완료
}

// ─────────────────────────── loop ───────────────────────────
void loop() {
    uint32_t now = millis();

    // 주기적 거리 측정 + 전송
    if (now - lastMeasure >= measureIntervalMs()) {
        lastMeasure = now;
        float dist_mm = measureDistanceMM();
        if (dist_mm >= 0 && commConnected()) {
            uint16_t sq = (g_lastSQ < 0) ? 0xFFFF : (uint16_t)g_lastSQ;
            commSendPacket(dist_mm, now, lastBatteryV, sq);
        }
        // WiFi 모드 링크 끊김 시 재접속 시도 (저전압 폴백과는 별개)
        if (commMode == MODE_WIFI && WiFi.status() != WL_CONNECTED) {
            WiFi.reconnect();
        }
        // 상태 LED: 연결 1Hz / 미연결 5Hz
        ledBlink(commConnected() ? 1000 : 200);
    }

    // 주기적 배터리 점검 + 2단계 저전압 처리
    if (now - lastBattCheck >= BATTERY_CHECK_MS) {
        lastBattCheck = now;
        lastBatteryV = readBatteryVoltage();
        Serial.printf("[BATT] %.2fV\n", lastBatteryV);

        if (lastBatteryV > 0.5f) {               // 유효 측정일 때만
            if (lastBatteryV < LOW_BATT_SHUTDOWN_B) {
                safeShutdown();                  // 2.80V: 종료(복귀 없음)
            } else if (lastBatteryV < LOW_BATT_WARN_B && !lowBattLatched) {
                lowBattLatched = true;           // 2.90V: 경고 1회
                Serial.println("[BATT] 저전압 경고");
                fallbackToBLE();                 // WiFi면 BLE로 전환
            }
        }
    }

    delay(1);
}
