# JRT U81 무선 레이저 거리측정 시스템 (v4)

2× AA 건전지로 동작하는 **무선 레이저 거리측정 패키지**. JRT U81 센서 + ESP32-C3 Super Mini로
BLE/WiFi를 통해 PC(또는 웹 UI)로 거리·신호품질·배터리를 스트리밍한다. 만능기판 + 납땜으로 조립.

> v4는 v3 설계 리뷰([`option_b_v3_리뷰_개선사항.md`](circuit_diagram_v3/option_b_v3_리뷰_개선사항.md))의
> 전기적 개선을 반영하고, 펌웨어·PC 수신기·웹 연동·테스트/CI까지 갖춘 버전이다.

---

## 1. 시스템 한눈에 보기

```
 ┌──────────┐  UART 19200   ┌───────────────┐   BLE NUS | WiFi UDP    ┌─────────────────────┐
 │ JRT U81  │ ────────────▶ │  ESP32-C3     │ ──────────────────────▶ │ PC 수신기 / 웹 UI    │
 │ 레이저   │  GPIO20/21    │  Super Mini   │   15B 패킷('<BfIfH')    │ pc_receiver / bridge │
 └──────────┘               └───────────────┘                         └─────────────────────┘
       ▲                          ▲   ▲
   3.3V│ (디커플링)          3V3 │   │ GPIO3(ADC) ← 100k/100k 분배기 ← 배터리
       │                          │   GPIO10 → 센서 EN
 [Pololu S7V8F3 Buck-Boost] ◀── 2×AA (홀더 내장 ON/OFF 스위치)
```

- **무선 패킷(15B, 리틀엔디안 `'<BfIfH'`)**: `id(u8)·distance_mm(f32)·timestamp_ms(u32)·battery_v(f32)·SQ(u16)`
  — 펌웨어·PC 수신기·웹 브리지가 **공유하는 계약**(셋을 함께 바꿀 것).
- **통신**: BLE(저전력, 기본) ↔ WiFi UDP(저지연) 런타임 전환. 저전압 시 WiFi→BLE 자동 폴백.

---

## 2. 빠른 시작 (End-to-End)

### ① 하드웨어 조립
[`circuit_diagram_v3/option_b_guide_v4.md`](circuit_diagram_v3/option_b_guide_v4.md) 따라 조립.
도면: [회로도](circuit_diagram_v3/option_b_schematic_v4.jpg) ·
[배치도](circuit_diagram_v3/option_b_top_component_side_v4.jpg) ·
[배선도](circuit_diagram_v3/option_b_bottom_solder_side_v4.jpg) ·
[배선 텍스트판](circuit_diagram_v3/option_b_solder_wiring_v4.md)

### ② 펌웨어 빌드·업로드 (arduino-cli)
절차: [`option_b_build_verify_v4.md`](circuit_diagram_v3/option_b_build_verify_v4.md) (실측 컴파일 검증됨)
```bash
arduino-cli core install esp32:esp32
arduino-cli lib install "NimBLE-Arduino"
mkdir -p build/fw && cp circuit_diagram_v3/option_b_firmware_v4.ino build/fw/
arduino-cli compile --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc" build/fw
arduino-cli upload  --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc" -p /dev/ttyACM0 build/fw
```

### ③ PC에서 수신
```bash
# CLI 수신기 (터미널 출력 + CSV)
python3 circuit_diagram_v3/pc_receiver_v4.py udp --csv log.csv      # WiFi UDP
python3 circuit_diagram_v3/pc_receiver_v4.py ble --laser-on         # BLE

# 또는 웹 UI (기존 패널 재사용)
pip install fastapi "uvicorn[standard]" websockets   # (+BLE면 bleak)
python3 lsd_interface/wireless_bridge_v4.py --mode udp --port 4210  # http://localhost:8000
```

---

## 3. 디렉터리 구조

```
lds_ws/
├─ README.md                         # 본 문서
├─ run_tests.sh                      # 소프트웨어 테스트 러너
├─ .github/workflows/ci.yml          # CI (소프트웨어 테스트 + 펌웨어 컴파일)
│
├─ circuit_diagram_v3/               # 하드웨어 · 펌웨어 · CLI 수신기
│   ├─ option_b_guide_v4.md          #   조립 가이드(전체)
│   ├─ option_b_schematic_v4.jpg     #   회로도
│   ├─ option_b_top_component_side_v4.jpg / *_bottom_solder_side_v4.jpg
│   ├─ option_b_solder_wiring_v4.md  #   납땜 배선 텍스트판
│   ├─ option_b_firmware_v4.ino      #   ESP32-C3 통합 펌웨어 (BLE+WiFi)
│   ├─ pc_receiver_v4.py             #   PC 수신기 (BLE/UDP CLI)
│   ├─ option_b_build_verify_v4.md   #   arduino-cli 빌드/업로드 절차
│   ├─ option_b_v3_리뷰_개선사항.md   #   v4 근거(설계 리뷰)
│   ├─ generate_v4_images.py         #   도면 생성기 (matplotlib)
│   ├─ make_solder_doc.py            #   배선 텍스트 생성기
│   ├─ test_packet_format.py         #   패킷 계약 테스트
│   └─ (참고) Pololu.png, ESP32_C3_pinmap.png, ESP32_C3_Schematic.png, *.pdf
│
└─ lsd_interface/                    # PC/웹 측
    ├─ wireless_bridge_v4.py         #   무선 → 웹 UI 브리지 (FastAPI)
    ├─ test_bridge_e2e.py            #   브리지 E2E 테스트
    ├─ test_bridge_concurrency.py    #   다중 클라이언트 견고성 테스트
    ├─ static/                       #   웹 UI (index.html, app.js, style.css)
    ├─ main.py / lds_controller.py   #   (별개) 유선 USB-serial ROS2 컨트롤러
    └─ requirements.txt, 실행가이드.md
```

> **참고**: `lsd_interface/main.py`+`lds_controller.py`는 **유선 USB-시리얼(ROS2, 바이너리 0x55 프로토콜)**
> 컨트롤러로, 본 무선 v4 시스템과는 독립적이다. 무선 v4는 `wireless_bridge_v4.py`가 같은 `static/` UI를 재사용한다.

---

## 4. 핵심 v4 설계 결정 (리뷰 반영)

| 항목 | 값 | 이유 |
|------|-----|------|
| ADC 필터 | 100nF~1µF **세라믹** | 전해는 누설·정착지연으로 잔량 오표시 |
| 전압분배 | **100kΩ/100kΩ** | 소스 임피던스↓(ADC 안정) |
| 저전압 | **2.9V 경고(→WiFi off) / 2.8V 종료(딥슬립)** | S7V8F3 UVLO ≈2.7V, 부하강하 여유 |
| 센서 EN | **GPIO10** | GPIO2는 부팅 스트래핑 핀 |
| 전원 안정화 | 3.3V 470µF + VIN 100~220µF | WiFi 돌입전류 브라운아웃 방지 |
| ADC 보정 | `analogReadMilliVolts()` | eFuse 캘리브레이션 |
| Pololu SHDN | NC(상시 ON) | 실제 보드 5핀 — 조립 혼선 방지 |

상세 근거: [`option_b_v3_리뷰_개선사항.md`](circuit_diagram_v3/option_b_v3_리뷰_개선사항.md)

---

## 5. 테스트 / CI

```bash
pip install fastapi "uvicorn[standard]" websockets
bash run_tests.sh
```
| 테스트 | 내용 | 상태 |
|--------|------|------|
| py-compile | 전 파이썬 구문 | ✅ |
| packet-format | 15B `'<BfIfH'` 라운드트립 + reject-bad | ✅ |
| bridge-e2e | UDP 주입 → distance(SQ)·battery 왕복 | ✅ |
| bridge-concurrency | 5클라 + 4000패킷 플러드 + 느린소비자 | ✅ |
| (CI) firmware-compile | ESP32-C3 BLE/WiFi 양쪽 컴파일 | ✅(실측) |

- 펌웨어 실측 컴파일: arduino-esp32 **3.3.8** + NimBLE **2.5.0** → 플래시 **92%**(BLE+WiFi 동시 포함).
- **정직 메모**: 동시성 테스트는 `send_lock` 유무와 무관히 PASS — 즉 락 *필요성*을 차등 증명하진 못함.
  락은 ASGI 규약(소켓당 단일 송신자)에 부합하는 **방어적·무해**한 조치로 유지.
- 이 저장소는 아직 git repo가 아니다 → `git init` + GitHub push 후 Actions가 동작한다.

---

## 6. 라이선스 / 출처
- 센서: JRT U8XX 시리즈(매뉴얼 PDF 동봉). MCU: ESP32-C3 Super Mini. 컨버터: Pololu S7V8F3.
- 펌웨어 의존: Arduino-ESP32 v3.x, NimBLE-Arduino v2.x.
