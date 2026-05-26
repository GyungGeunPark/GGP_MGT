# Option B v4 — 펌웨어 빌드/업로드 검증 절차 (arduino-cli)

> 대상: `option_b_firmware_v4.ino` (ESP32-C3 Super Mini, BLE+WiFi 통합)
> 목적: GUI 없이 **arduino-cli**로 컴파일·업로드·검증하는 재현 가능한 절차.
> 이 문서의 컴파일 결과는 실제 실행으로 검증되었습니다(§6 결과 로그 참조).

---

## 0. 사전 요구

| 항목 | 값 |
|------|-----|
| 보드 | ESP32-C3 Super Mini (`nologo_esp32c3_super_mini` 또는 일반 `esp32c3`) |
| Arduino-ESP32 코어 | v3.x (Espressif) |
| 필수 라이브러리 | `NimBLE-Arduino` (v2.x) — BLE용. WiFi/UDP는 코어 내장 |
| 호스트 | Linux/macOS/Windows + 인터넷(코어·툴체인 다운로드) |

---

## 1. arduino-cli 설치

```bash
# 단일 바이너리 설치 (~/bin)
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/bin sh
export PATH="$HOME/bin:$PATH"
arduino-cli version
```

## 2. ESP32 보드 패키지 등록·설치

```bash
arduino-cli config init --overwrite
arduino-cli config add board_manager.additional_urls \
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32          # 코어 + RISC-V 툴체인 (수백 MB, 수 분)
```

## 3. 라이브러리 설치

```bash
arduino-cli lib install "NimBLE-Arduino"      # BLE NUS 스택
# WiFi.h / WiFiUdp.h 는 esp32 코어에 내장 — 별도 설치 불필요
```

## 4. 스케치 디렉터리 준비

> arduino-cli는 **`.ino` 파일이 같은 이름의 폴더 안**에 있어야 합니다.

```bash
mkdir -p ~/build/option_b_firmware_v4
cp option_b_firmware_v4.ino ~/build/option_b_firmware_v4/
```

## 5. 컴파일 (검증)

```bash
# 일반 ESP32-C3 FQBN + USB CDC On Boot 활성 (Serial = USB)
arduino-cli compile \
  --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc" \
  ~/build/option_b_firmware_v4

# Super Mini 보드 정의가 있는 코어라면:
# --fqbn "esp32:esp32:nologo_esp32c3_super_mini:CDCOnBoot=cdc"
```

**핵심 빌드 옵션**
| 옵션 | 값 | 이유 |
|------|-----|------|
| `CDCOnBoot` | `cdc` | `Serial`을 USB-CDC로 → 디버그 로그가 USB로, 센서 UART(GPIO20/21)와 분리 |
| FQBN | `esp32:esp32:esp32c3` | C3 타깃. Super Mini 변형이 있으면 그쪽 사용 |

> ⚠ 컴파일은 펌웨어가 USB CDC로 빌드되든 아니든 통과합니다. `CDCOnBoot=cdc`는 **런타임에 `Serial`(USB) 로그가 나오게** 하기 위한 것이며, 센서 UART는 UART1(GPIO20/21)로 매핑되어 영향받지 않습니다.

## 6. 컴파일 결과 (실측 — 본 절차로 검증됨)

환경: arduino-cli **1.5.0** · arduino-esp32 코어 **3.3.8** · NimBLE-Arduino **2.5.0** (Linux)

```
--- COMPILE (esp32:esp32:esp32c3:CDCOnBoot=cdc) ---
Sketch uses 1212943 bytes (92%) of program storage space. Maximum is 1310720 bytes.
Global variables use 44668 bytes (13%) of dynamic memory, leaving 283012 bytes for local variables. Maximum is 327680 bytes.
COMPILE_EXIT=0      ← ✅ 빌드 성공
```

> ✅ BLE(NimBLE v2.5.0) + WiFi 스택을 **모두 포함**한 통합 펌웨어가 정상 컴파일됨.
> NimBLE v2 콜백 시그니처, `analogReadMilliVolts()`, `gpio_hold_en()`/`gpio_deep_sleep_hold_en()` 모두 실제 코어에서 검증.

> ⚠ **플래시 92% (1.21MB / 1.31MB)** — BLE+WiFi 동시 포함이라 기본 앱 파티션에서 빠듯합니다.
> 기능 추가로 오버플로 시 더 큰 앱 파티션을 선택하세요:
> ```bash
> # 예: Huge APP(3MB) 파티션
> arduino-cli compile --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app" <sketchdir>
> ```
> BLE 또는 WiFi 한쪽만 쓸 경우, 미사용 스택 #include/코드를 제거하면 크게 절감됩니다.

## 7. 업로드 (실제 보드 연결 시)

```bash
# 포트 확인
arduino-cli board list

# 외부 3.3V 분리 후 USB-C만 연결, 첫 업로드 시 BOOT 버튼 누른 채 시작
arduino-cli upload \
  --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc" \
  -p /dev/ttyACM0 \
  ~/build/option_b_firmware_v4

# 시리얼 모니터 (USB CDC, 115200)
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

> **Linux 권한**: `/dev/ttyACM0` 접근 권한 오류 시 사용자를 `dialout` 그룹에 추가하고 재로그인:
> `sudo usermod -aG dialout $USER`

> **전원 충돌 주의**(가이드 §6.4): USB-C와 외부 3.3V를 **동시에** 연결하지 말 것. 업로드 시 외부 전원 분리.

## 8. WiFi 모드로 빌드하려면

`option_b_firmware_v4.ino` 상단에서:
```cpp
#define DEFAULT_COMM_MODE   MODE_WIFI       // 기본 WiFi
static const char* WIFI_SSID = "...";       // 실제 값
static const char* WIFI_PASS = "...";
static const IPAddress PC_IP(192,168,1,100);
```
수정 후 동일하게 컴파일·업로드. (저전압 2.9V 시 자동으로 BLE로 폴백)

> ✅ **WiFi 기본 모드 빌드도 실측 검증**: `DEFAULT_COMM_MODE=MODE_WIFI`로 컴파일 성공(exit 0, 플래시 1,212,927B / 92%).
> 통합 펌웨어는 BLE·WiFi 양 스택을 항상 포함(런타임 전환)하므로 두 기본 모드의 빌드 산출물 크기는 사실상 동일(16B 차이 = 상수값뿐).

## 9. 빠른 트러블슈팅

| 증상 | 원인/해결 |
|------|-----------|
| `NimBLEDevice.h: No such file` | `arduino-cli lib install "NimBLE-Arduino"` 누락 |
| `esp32:esp32 platform not installed` | §2 core install 재실행 |
| 업로드 후 시리얼 로그 없음 | `CDCOnBoot=cdc` 누락 → USB CDC 비활성. 옵션 추가 후 재빌드 |
| 센서 무응답 | UART 크로스(GPIO20↔21), EN=GPIO10, 3.3V 확인 (가이드 §7) |
| 업로드 실패(포트 못 찾음) | BOOT 버튼 누른 채 연결 → `board list`로 포트 재확인 |

---

*검증 절차는 arduino-cli 단독(헤드리스)로 재현 가능. CI에 그대로 넣을 수 있습니다.*
