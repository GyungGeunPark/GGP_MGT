# JRT U81 무선 레이저 거리측정 시스템 구축 가이드

> JRT U81 + ESP32-C3 Super Mini + BLE/WiFi UDP 무선 거리측정 시스템
> 배터리 옵션: **3S LiPo 1000mAh** 또는 **2× AA 건전지**

**로봇 엔드이펙터에 장착할 수 있는 mm급 무선 레이저 거리측정 패키지를 기성 모듈만으로 구축한다.** JRT U81 센서와 ESP32-C3 Super Mini는 모두 3.3V 네이티브이므로 레벨 시프터나 외부 ADC 없이 UART 4선 직결만으로 연결이 완료된다. BLE 모드에서 20~40ms, WiFi UDP 모드에서 3~8ms 지연시간을 달성하며, 배터리 포함 총 무게 80~200g, 부품비 $35~65로 구현 가능하다.

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [부품 목록 (BOM)](#2-부품-목록-bom)
3. [배터리 시스템 A: 3S LiPo 1000mAh + BMS + 충전](#3-배터리-시스템-a-3s-lipo-1000mah--bms--충전)
4. [배터리 시스템 B: 2× AA 건전지 + Buck-Boost](#4-배터리-시스템-b-2-aa-건전지--buck-boost)
5. [소프트 파워 스위치 회로](#5-소프트-파워-스위치-회로)
6. [JRT U81 센서 배선](#6-jrt-u81-센서-배선)
7. [ESP32-C3 Super Mini 배선 요약](#7-esp32-c3-super-mini-배선-요약)
8. [전체 시스템 배선도](#8-전체-시스템-배선도)
9. [만능기판 조립 가이드](#9-만능기판-조립-가이드)
10. [펌웨어: BLE 모드 (Arduino)](#10-펌웨어-ble-모드-arduino)
11. [펌웨어: WiFi UDP 모드 (Arduino)](#11-펌웨어-wifi-udp-모드-arduino)
12. [PC 수신 소프트웨어: BLE (Python)](#12-pc-수신-소프트웨어-ble-python)
13. [PC 수신 소프트웨어: WiFi UDP (Python)](#13-pc-수신-소프트웨어-wifi-udp-python)
14. [배터리 모니터링 코드](#14-배터리-모니터링-코드)
15. [물리적 조립 및 마운팅](#15-물리적-조립-및-마운팅)
16. [테스트 및 검증 절차](#16-테스트-및-검증-절차)
17. [트러블슈팅](#17-트러블슈팅)
- [부록 A: UART 통신 테스트 코드](#부록-a-uart-통신-테스트-코드)
- [부록 B: 배터리 전압 캘리브레이션 코드](#부록-b-배터리-전압-캘리브레이션-코드)
- [부록 C: 완성 시스템 체크리스트](#부록-c-완성-시스템-체크리스트)

---

## 1. 시스템 개요

### 1.1 목표

로봇 엔드이펙터 또는 휴대형 측정 도구에 장착할 수 있는 **완전 무선 레이저 거리측정 패키지**를 기성 모듈만으로 구축한다. PCB 설계 없이 만능기판과 납땜으로 조립하며, 2시간 내 하드웨어 완성이 가능하다.

**기존 HG-C1030/C1100 가이드와의 차이점:**
- **훨씬 간단한 배선**: JRT U81은 3.3V UART 직접 연결 — ADC(ADS1115), 레벨 시프터(BSS138), 24V 부스트 컨버터가 모두 **불필요**
- **듀얼 통신**: BLE와 WiFi UDP 모두 지원 (상황에 맞게 선택)
- **듀얼 배터리**: 3S LiPo (장시간) 또는 2× AA (초간편) 선택 가능
- **경량 소형**: 센서+MCU 합계 ~8g, 배터리 포함 전체 80~200g

### 1.2 핵심 성능 스펙

| 항목 | 값 | 비고 |
|------|-----|------|
| 센서 정확도 | ±1mm | JRT U81 고유 스펙 |
| 분해능 | 1mm | |
| 측정 범위 | 0.03~20m | 타겟 반사율에 따라 |
| 업데이트 속도 | 3~8 Hz | 단일 측정 모드 |
| UART 통신 | 19200 bps, 8N1 | ASCII 명령 프로토콜 |
| 무선 지연 (WiFi UDP) | 3~8 ms | 실시간 제어 루프에 적합 |
| 무선 지연 (BLE) | 20~40 ms | 배터리 절약에 적합 |
| 배터리 동작시간 (3S LiPo, BLE) | ~13시간 | 1000mAh |
| 배터리 동작시간 (2× AA, BLE) | ~30시간 | 알카라인 2500mAh |
| 센서+MCU 무게 | ~8g | 배터리/기판 제외 |
| 총 패키지 무게 (3S LiPo) | ~180g | 센서+기판+배터리+인클로저 |
| 총 패키지 무게 (2× AA) | ~120g | 센서+기판+배터리+인클로저 |

### 1.3 시스템 블록 다이어그램

**Option A: 3S LiPo 전원**
```
┌─────────────────────────────────────────────────────────────────────┐
│              JRT U81 무선 레이저 거리측정 시스템 (Option A)             │
│                                                                     │
│  ┌─────────────┐    ┌─────┐    ┌────────┐    ┌──────────────────┐  │
│  │ 3S LiPo     │───>│ BMS │───>│ 소프트  │───>│ MP1584EN         │  │
│  │ 11.1V       │    │ 3S  │    │ 파워   │    │ 11.1V → 3.3V    │  │
│  │ 1000mAh     │    │     │    │ 스위치  │    │ (고정 3.3V 출력) │  │
│  └─────────────┘    └─────┘    └────────┘    └────────┬─────────┘  │
│                                                        │            │
│                                        ┌───────────────┤            │
│                                        │               │            │
│                                        ▼               ▼            │
│                                  ┌──────────┐    ┌──────────┐      │
│                                  │ JRT U81  │    │ ESP32-C3 │      │
│                                  │ 센서     │<──>│ Super    │      │
│                                  │ (UART)   │    │ Mini     │      │
│                                  └──────────┘    │          │      │
│                                                  │ BLE/WiFi │      │
│                      [ADC: 배터리 전압 모니터링]──>│  )))     │      │
│                                                  └──────────┘      │
│                                                       ↕ 무선       │
│                                                  ┌──────────┐      │
│                                                  │ PC       │      │
│                                                  │ (Python) │      │
│                                                  └──────────┘      │
└─────────────────────────────────────────────────────────────────────┘
```

**Option B: 2× AA 전원**
```
┌─────────────────────────────────────────────────────────────────────┐
│              JRT U81 무선 레이저 거리측정 시스템 (Option B)             │
│                                                                     │
│  ┌─────────────┐    ┌────────┐    ┌───────────────────┐            │
│  │ 2× AA       │───>│ 소프트  │───>│ Pololu S7V8F3     │            │
│  │ 3.0V        │    │ 파워   │    │ Buck-Boost        │            │
│  │ (알카라인)   │    │ 스위치  │    │ 1.8~3.0V → 3.3V  │            │
│  └─────────────┘    └────────┘    └─────────┬─────────┘            │
│                                              │                      │
│                                  ┌───────────┤                      │
│                                  │           │                      │
│                                  ▼           ▼                      │
│                            ┌──────────┐ ┌──────────┐               │
│                            │ JRT U81  │ │ ESP32-C3 │               │
│                            │ 센서     │<>│ Super    │               │
│                            │ (UART)   │ │ Mini     │               │
│                            └──────────┘ │ BLE/WiFi │               │
│              [ADC: 배터리 전압 모니터링]──>│  )))     │               │
│                                         └──────────┘               │
│                                              ↕ 무선                 │
│                                         ┌──────────┐               │
│                                         │ PC       │               │
│                                         │ (Python) │               │
│                                         └──────────┘               │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.4 통신 모드 비교

| 파라미터 | WiFi UDP | BLE (NUS) |
|---------|----------|-----------|
| 일반적 지연시간 | **3~8 ms** | 20~40 ms |
| 처리량 | ~1~5 Mbps | 100~250 Kbps |
| 소비 전력 | 120~240 mA | **20~100 mA** |
| 무선 범위 | 30~100 m | 10~30 m |
| 인프라 | WiFi AP 필요 | **피어-투-피어** |
| 권장 용도 | 실시간 제어 루프 | 배터리 절약, 야외 |

- **실시간 로봇 제어, 실내 실험실** → WiFi UDP 선택
- **배터리 수명 우선, WiFi 없는 환경** → BLE 선택

### 1.5 필요 장비

- 납땜 인두 및 납 (필수)
- 멀티미터 (필수 — 전압/연속성 확인)
- 와이어 스트리퍼 / 니퍼
- PC (Arduino IDE 2.x, Python 3.8+)
- USB-C 케이블 (ESP32 프로그래밍)
- 스마트폰 BLE 스캐너 앱 (nRF Connect — BLE 테스트용)

---

## 2. 부품 목록 (BOM)

### 2.1 공통 부품

| # | 부품 | 모델/스펙 | 수량 | 예상 가격 | 구매처 |
|---|------|----------|------|----------|--------|
| 1 | 레이저 거리 센서 | JRT U81 (41×17×7mm, 3.3V UART) | 1 | $20~30 | AliExpress, Alibaba |
| 2 | MCU | ESP32-C3 Super Mini (22.5×18mm) | 1 | $2~3 | AliExpress |
| 3 | 디커플링 캐패시터 | 100µF 전해 + 100nF 세라믹 | 1세트 | $0.20 | DigiKey, Amazon |
| 4 | 소프트 스위치 MOSFET | AO3401 P-ch SOT-23 | 1 | $0.30 | AliExpress |
| 5 | 스위치 저항 | 10kΩ + 100kΩ ¼W | 각 1 | $0.10 | |
| 6 | 택트 스위치 | 6×6mm 모멘터리 | 1 | $0.10 | |
| 7 | ADC 필터 캐패시터 | 10µF 세라믹 (배터리 모니터링) | 1 | $0.10 | |
| 8 | 만능기판 | 50×30mm 또는 70×50mm, 2.54mm 피치 | 1 | $1.50 | Amazon |
| 9 | 핀헤더 소켓 | 2.54mm 암 (ESP32 탈착용) | 1세트 | $0.50 | |
| 10 | 배선 와이어 | 26AWG 실리콘, 4색 | 1세트 | $1.00 | |
| 11 | 인클로저 | 3D 프린팅 PETG | 1 | $1~3 | |
| 12 | 마운팅 하드웨어 | M2 나사 + 스탠드오프 × 4 | 1세트 | $1.00 | |

### 2.2 Option A 전용 부품: 3S LiPo

| # | 부품 | 모델/스펙 | 수량 | 예상 가격 |
|---|------|----------|------|----------|
| A1 | 3S LiPo 배터리 | 11.1V 1000mAh, JST-XH 밸런스 | 1 | $12~18 |
| A2 | 3S BMS 보호보드 | 12.6V 10A, 밸런스 지원 | 1 | $3.00 |
| A3 | 3.3V 벅 컨버터 | MP1584EN 고정 3.3V 모듈 | 1 | $2.00 |
| A4 | 밸런스 충전기 | B3 Pro 3S 컴팩트 충전기 | 1 | $10.00 |
| A5 | 전압분배 저항 | 100kΩ + 27kΩ ¼W | 각 1 | $0.10 |
| | **Option A 합계 (센서 제외)** | | | **$35~45** |

### 2.3 Option B 전용 부품: 2× AA

| # | 부품 | 모델/스펙 | 수량 | 예상 가격 |
|---|------|----------|------|----------|
| B1 | AA 건전지 | 알카라인 1.5V (또는 NiMH 1.2V) | 2 | $1~2 |
| B2 | 건전지 홀더 | 2셀 AA 홀더 (리드선 포함) | 1 | $1.00 |
| B3 | Buck-Boost 컨버터 | Pololu S7V8F3 (3.3V 고정 출력) | 1 | $5~8 |
| B4 | 전압분배 저항 | 220kΩ × 2 ¼W | 2 | $0.10 |
| | **Option B 합계 (센서 제외)** | | | **$15~22** |
| | **반복 비용 (건전지 교체)** | | | **~$1/세트** |

---

## 3. 배터리 시스템 A: 3S LiPo 1000mAh + BMS + 충전

### 3.1 3S LiPo 기본 사양

| 항목 | 값 |
|------|-----|
| 공칭 전압 | 11.1V (3.7V × 3셀) |
| 만충 전압 | 12.6V (4.2V × 3셀) |
| 방전 컷오프 | 9.0V (3.0V × 3셀) |
| 용량 | 1000mAh |
| 에너지 | 11.1Wh |
| 무게 | ~85~95g |
| 크기 | ~70×34×20mm |
| 밸런스 커넥터 | JST-XH 4핀 |

### 3.2 BMS 배선

#### 배터리 셀 구조

```
     B0 (0V)     B1 (3.7V)    B2 (7.4V)    B3 (11.1V)
      │            │            │            │
      ├── Cell 1 ──┤── Cell 2 ──┤── Cell 3 ──┤
      │   3.7V     │   3.7V     │   3.7V     │
      ▼            ▼            ▼            ▼
    B-(파란)      밸런스1      밸런스2      B+(빨강)
```

#### BMS 모듈 배선도

```
                    3S BMS 모듈
              ┌──────────────────┐
    배터리    │                  │    부하 (회로)
    ─────────>│  B-  ────── P-  │──────────> GND
    B- (파란) │                  │
              │  B1  (밸런스1)  │
    ─────────>│                  │
              │  B2  (밸런스2)  │
    ─────────>│                  │
              │  B+  ────── P+  │──────────> +11.1V
    ─────────>│                  │
    B+ (빨강) └──────────────────┘
```

**배선 절차:**

1. **밸런스 커넥터 확인**: 3S LiPo의 JST-XH 4핀 밸런스 커넥터에서 각 와이어의 전압을 멀티미터로 측정
   - 1번 핀 (검정): 0V (B-)
   - 2번 핀: ~3.7V (B1)
   - 3번 핀: ~7.4V (B2)
   - 4번 핀 (빨강): ~11.1V (B+)

2. **BMS 연결 순서** (반드시 이 순서를 따를 것):
   - ① B- (배터리 음극) → BMS B- 단자
   - ② B1 (밸런스1) → BMS B1 단자
   - ③ B2 (밸런스2) → BMS B2 단자
   - ④ B+ (배터리 양극) → BMS B+ 단자 (**마지막에 연결**)

3. **출력 확인**: P+와 P- 사이에서 ~11.1V 측정 확인

> **주의**: B+ 연결 시 스파크가 발생할 수 있으므로 빠르게 접촉한다. 역극성 연결은 BMS를 즉시 파괴한다.

### 3.3 벅 컨버터: 11.1V → 3.3V

**MP1584EN 고정 3.3V 모듈**을 사용한다. JRT U81과 ESP32-C3 모두 3.3V에서 동작하므로 5V 단계가 불필요하다.

```
BMS P+ (11.1V) ────────> +VIN  (MP1584EN)
BMS P- (GND)   ────────> GND   (MP1584EN)
                          +VOUT (MP1584EN) ────> 3.3V 공통 버스
                          GND   (MP1584EN) ────> GND 공통 버스
```

| 핀 | 연결 |
|----|------|
| +VIN | 소프트 스위치 출력 (11.1V) |
| GND (입력) | 공통 GND |
| +VOUT | ESP32-C3 3V3 핀 + JRT U81 VCC |
| GND (출력) | ESP32-C3 GND + JRT U81 GND |

> **고정 3.3V 모듈 사용**: 반드시 "고정 3.3V" 출력 모듈을 구매한다. 가변형은 포텐 드리프트로 ESP32를 손상시킬 위험이 있다.

**출력 확인**: +VOUT-GND 간 3.3V ±0.1V를 멀티미터로 확인한 **후에** ESP32에 연결한다.

### 3.4 배터리 전압 모니터링 회로

11.1~12.6V를 ESP32-C3의 ADC 범위(0~3.3V)로 변환하기 위해 저항 분배기를 사용한다.

```
배터리 (BMS P+) ──────┬──[R1: 100kΩ]──┬──> ESP32-C3 GPIO3 (ADC)
                      │                │
                      │           [R2: 27kΩ]
                      │                │
                      │           [C: 10µF]  ← 노이즈 필터
                      │                │
GND ──────────────────┴────────────────┴──> GND
```

**계산:**
```
V_adc = V_bat × R2 / (R1 + R2)
      = V_bat × 27k / (100k + 27k)
      = V_bat × 0.2126

만충 12.6V: V_adc = 12.6 × 0.2126 = 2.68V (ADC 범위 내)
컷오프 9.0V: V_adc = 9.0 × 0.2126 = 1.91V
```

**전압-잔량 매핑:**

| 배터리 전압 | ADC 전압 | 잔량 |
|------------|---------|------|
| 12.6V | 2.68V | 100% |
| 11.1V | 2.36V | 50% |
| 9.9V | 2.10V | 10% (저전압 경고) |
| 9.0V | 1.91V | 0% (자동 차단) |

### 3.5 충전 시스템

**B3 Pro 3S 컴팩트 밸런스 충전기** ($10) 사용:

1. 충전기의 밸런스 커넥터에 3S LiPo의 JST-XH 플러그를 연결
2. 충전기 전원 연결 (AC 어댑터)
3. 셀 수 확인 (3S 표시) 후 충전 시작
4. 완충 시 자동 정지 (12.6V)
5. 충전 시간: 1000mAh 기준 약 1~1.5시간

> **안전 수칙**: 충전 중 시스템에서 배터리를 분리한다. LiPo 화재 방지를 위해 충전 중 자리를 비우지 않는다. LiPo 보관 가방 사용을 권장한다.

### 3.6 전력 예산

| 소비원 | 전압 | 전류 | 전력 |
|--------|------|------|------|
| JRT U81 (측정 버스트) | 3.3V | ~100mA | 0.33W |
| JRT U81 (대기) | 3.3V | ~30mA | 0.10W |
| ESP32-C3 (BLE 활성) | 3.3V | ~80mA | 0.26W |
| ESP32-C3 (WiFi 활성) | 3.3V | ~150mA | 0.50W |
| MP1584EN 손실 (효율 92%) | — | — | ~0.06W |
| **총 (BLE 모드, 평균)** | | **~110mA @3.3V** | **~0.42W** |
| **총 (WiFi 모드, 평균)** | | **~180mA @3.3V** | **~0.66W** |

**런타임 계산 (BLE 모드):**
```
배터리 에너지: 1000mAh × 11.1V = 11.1Wh
벅 효율: 92%
유효 에너지: 11.1 × 0.92 = 10.2Wh
시스템 소비: ~0.42W (BLE)
이론 런타임: 10.2 / 0.42 = ~24시간
실측 런타임 (60%): ~14시간
```

**런타임 계산 (WiFi 모드):**
```
유효 에너지: 10.2Wh
시스템 소비: ~0.66W (WiFi)
이론 런타임: 10.2 / 0.66 = ~15시간
실측 런타임 (60%): ~9시간
```

---

## 4. 배터리 시스템 B: 2× AA 건전지 + Buck-Boost

### 4.1 AA 건전지 기본 사양

| 항목 | 알카라인 AA | NiMH AA |
|------|------------|---------|
| 공칭 전압 (1셀) | 1.5V | 1.2V |
| 2셀 직렬 전압 | 3.0V (신품) ~ 1.8V (방전) | 2.4V ~ 2.0V |
| 용량 | ~2500mAh | ~2000mAh |
| 무게 (2셀) | ~46g | ~54g |
| 홀더 포함 무게 | ~56g | ~64g |

### 4.2 Buck-Boost 컨버터: 1.8~3.0V → 3.3V

2× AA의 전압(1.8~3.0V)은 3.3V 위아래로 변동하므로, Buck-Boost 토폴로지가 필수이다.

**Pololu S7V8F3** (3.3V 고정 출력):

| 항목 | 값 |
|------|-----|
| 입력 범위 | 2.7V ~ 11.8V |
| 출력 | 3.3V 고정 |
| 출력 전류 | 최대 1A (입력에 따라) |
| 효율 | >90% |
| 스위칭 주파수 | ~500kHz |
| 크기 | 10.4×8.9×3mm |

> **중요**: S7V8F3 최소 입력 2.7V → 알카라인 1.35V/셀(2.7V 합계)까지 사용 가능. NiMH의 경우 말기 전압(2.0V)이 최소 입력보다 낮으므로, 완전 방전 전에 컨버터가 차단된다.

**대안**: TPS631000 기반 모듈 (~95% 효율, 최소 입력 1.8V, 더 넓은 범위)

```
2× AA (+) ────────> VIN  (Pololu S7V8F3)
2× AA (-) ────────> GND  (Pololu S7V8F3)
                     VOUT (S7V8F3) ────> 3.3V 공통 버스
                     GND  (S7V8F3) ────> GND 공통 버스
```

**출력 확인**: 무부하 상태에서 VOUT-GND 간 **3.3V ±0.1V**를 멀티미터로 확인한다.

### 4.3 배터리 전압 모니터링 회로

2× AA의 전압(1.8~3.0V)을 ADC 범위로 변환한다. 1:1 분배기 사용.

```
배터리 (+) ──────┬──[R1: 220kΩ]──┬──> ESP32-C3 GPIO3 (ADC)
                 │                │
                 │           [R2: 220kΩ]
                 │                │
                 │           [C: 10µF]  ← 노이즈 필터
                 │                │
GND ─────────────┴────────────────┴──> GND
```

**계산:**
```
V_adc = V_bat × R2 / (R1 + R2)
      = V_bat × 220k / (220k + 220k)
      = V_bat × 0.5

신품 3.0V: V_adc = 3.0 × 0.5 = 1.50V
방전 1.8V: V_adc = 1.8 × 0.5 = 0.90V
```

**전압-잔량 매핑 (알카라인):**

| 배터리 전압 | ADC 전압 | 잔량 |
|------------|---------|------|
| 3.0V | 1.50V | 100% |
| 2.6V | 1.30V | 50% |
| 2.4V | 1.20V | 20% (저전압 경고) |
| 2.0V | 1.00V | 0% (교체 필요) |

### 4.4 전력 예산 및 런타임

| 소비원 | 전류 | 전력 |
|--------|------|------|
| JRT U81 (평균) | ~50mA | 0.17W |
| ESP32-C3 (BLE) | ~80mA | 0.26W |
| S7V8F3 손실 (효율 90%) | — | ~0.05W |
| **총 (BLE 모드)** | **~130mA @3.3V** | **~0.48W** |

**런타임 (알카라인, BLE 모드):**
```
배터리 에너지: 2500mAh × 2.4V(평균) = 6.0Wh
컨버터 효율: 90%
유효 에너지: 6.0 × 0.90 = 5.4Wh
시스템 소비: ~0.48W
이론 런타임: 5.4 / 0.48 = ~11시간
실측 런타임 (70%): ~8시간
```

### 4.5 종합 비교

| 항목 | Option A (3S LiPo) | Option B (2× AA) |
|------|--------------------|--------------------|
| **런타임 (BLE)** | ~14시간 | ~8시간 |
| **배터리 무게** | ~90g | ~56g |
| **초기 비용** | ~$35~45 | ~$15~22 |
| **반복 비용** | $0 (재충전) | ~$1/세트 |
| **현장 교체** | 낮음 (충전 필요) | **높음 (건전지 교체)** |
| **회로 복잡도** | 중간 (BMS 필요) | **낮음** |
| **안전성** | LiPo 관리 필요 | **매우 안전** |

**권장:**
- **실내/실험실, 매일 사용** → Option A (재충전, 장시간)
- **현장/야외, 비정기 사용** → Option B (초간편, 즉시 교체)

---

## 5. 소프트 파워 스위치 회로

### 5.1 회로 설계

P-channel MOSFET 기반 래칭 소프트 파워 스위치로, 대기 전류 <1µA를 달성한다.

```
                         AO3401 (P-ch MOSFET, SOT-23)
                         ┌──────────┐
                         │          │
배터리 (+) ──────────────┤ Source   │
                         │          │
                         │  Gate ◄──┼──┬──[R2: 100kΩ]──> 배터리 (+)  (풀업)
                         │          │  │
                         │          │  ├──[R1: 10kΩ]───> 택트 스위치 ──> GND
                         │          │  │
                         │ Drain ───┼──┘
                         └──────────┘
                              │
                              ▼
                         부하 (+) ──> 벅/벅부스트 컨버터 VIN
```

**부품:**
- AO3401 P-channel MOSFET (SOT-23 패키지, V_GS(th) ≈ -1.2V)
- R1: 10kΩ (게이트-스위치)
- R2: 100kΩ (게이트-소스 풀업)
- 택트 스위치: 6×6mm 모멘터리 푸시버튼

### 5.2 동작 원리

**OFF 상태**: R2(100kΩ)가 Gate를 Source(배터리+)로 풀업 → V_GS ≈ 0V → MOSFET OFF → 부하에 전류 없음. 대기 전류는 R2를 통한 누설만 존재 (<1µA).

**ON 전환**: 택트 스위치를 누르면 Gate가 R1(10kΩ)을 통해 GND로 하강 → V_GS < V_GS(th) → MOSFET ON → 전원 공급 시작.

**OFF 전환**: 스위치를 놓으면 R2가 Gate를 다시 Source로 풀업 → MOSFET OFF.

> **참고**: 이 기본 회로는 버튼을 누르고 있는 동안만 ON이다. 래칭(토글) 동작이 필요하면 ESP32 GPIO로 Gate를 제어하는 확장 회로가 필요하지만, 거리측정 세션 중에는 보통 연속 ON 상태를 유지하므로, **물리적 슬라이드 스위치**를 직렬로 사용하는 것이 더 간단하다.

**간단한 대안 — 슬라이드 스위치:**

```
배터리 (+) ──[슬라이드 스위치]──> 벅/벅부스트 컨버터 VIN
```

슬라이드 스위치(SPST, 2A 이상)를 배터리 양극과 컨버터 사이에 직렬 연결하면 가장 간단한 ON/OFF 제어가 된다.

### 5.3 Option A용 배선 위치

```
3S LiPo ──> BMS P+ ──[스위치]──> MP1584EN VIN
                  BMS P- ──────────> MP1584EN GND
```

### 5.4 Option B용 배선 위치

```
2× AA (+) ──[스위치]──> Pololu S7V8F3 VIN
2× AA (-) ─────────────> Pololu S7V8F3 GND
```

### 5.5 조립 주의사항

- AO3401 SOT-23 패키지는 매우 작다 (2.9×1.3mm). **SOT-23→DIP 변환 기판**($0.50)을 사용하면 만능기판 납땜이 편하다
- 슬라이드 스위치 방식이 더 간단하며, 대부분의 용도에 충분하다
- 스위치 접점 전류 정격이 시스템 최대 전류(~300mA) 이상인지 확인

---

## 6. JRT U81 센서 배선

### 6.1 센서 핀 배치

JRT U81은 41×17×7mm의 초소형 OEM 레이저 거리측정 모듈이다.

| 핀 | 기능 | 설명 |
|----|------|------|
| **VCC** | 전원 | DC 2.0~3.3V (3.3V 권장) |
| **GND** | 접지 | 공통 GND |
| **TX** | UART 송신 | 센서 → MCU (측정 데이터 출력) |
| **RX** | UART 수신 | MCU → 센서 (명령 입력) |
| **EN** | 활성화/슬립 | HIGH = 활성, LOW = 슬립 (선택적) |

### 6.2 UART 프로토콜

| 항목 | 값 |
|------|-----|
| 보레이트 | 19200 bps |
| 데이터 비트 | 8 |
| 패리티 | None |
| 정지 비트 | 1 |
| 초기화 시간 | 전원 인가 후 ≥100ms, 이후 `,OK!` 전송 |

**ASCII 명령 세트:**

| 명령 | 코드 | 응답 | 설명 |
|------|------|------|------|
| 레이저 ON | `'O'` (0x4F) | `,OK` | 레이저 점등 |
| 거리 측정 | `'D'` (0x44) | `12.345m, 0079` | 거리(m) + 신호 품질 |
| 레이저 OFF | `'C'` (0x43) | `,OK` | 레이저 소등 |
| 상태 조회 | `'S'` (0x53) | `18.0'C, 2.7V` | 온도 + 공급 전압 |

**응답 파싱 규칙:**
- 거리 응답에서 `'m'` 문자 위치를 찾아 그 앞의 숫자를 float로 변환
- 신호 품질(SQ) 값은 쉼표 뒤의 4자리 숫자, **값이 낮을수록 좋음**
- 타겟이 없거나 범위 초과 시 `,ERR` 또는 에러 코드 반환

### 6.3 ESP32-C3 연결

JRT U81(3.3V)과 ESP32-C3(3.3V)는 전압이 동일하므로 **레벨 시프터가 불필요**하다. UART 크로스 연결만 하면 된다.

```
JRT U81 센서             ESP32-C3 Super Mini
──────────────          ───────────────────
VCC  ──────────────────── 3V3 (3.3V 공통 버스)
GND  ──────────────────── GND (공통 GND)
TX   ──────────────────── GPIO20 (UART0 RX)  ← 크로스!
RX   ──────────────────── GPIO21 (UART0 TX)  ← 크로스!
EN   ──────────────────── GPIO2  (선택적 슬립 제어)
```

> **핵심**: TX↔RX 크로스 연결이다. 센서의 TX(출력)가 ESP32의 RX(입력)에, 센서의 RX(입력)가 ESP32의 TX(출력)에 연결된다. 이것은 UART 통신의 표준 규칙이다.

### 6.4 디커플링 캐패시터

WiFi/BLE 전송 버스트가 센서 전원에 노이즈를 유발할 수 있다. 센서 VCC/GND 핀에 **최대한 가까이** 디커플링 캐패시터를 배치한다.

```
3.3V 버스 ────┬──[100µF 전해]──┬──> JRT U81 VCC
              │                │
              └──[100nF 세라믹]─┘
              │
GND ──────────┴────────────────────> JRT U81 GND
```

- **100µF 전해 캐패시터**: 저주파 전원 변동 흡수 (극성 주의! + 마킹이 VCC 쪽)
- **100nF 세라믹 캐패시터**: 고주파 스위칭 노이즈 필터링
- 두 캐패시터를 **병렬**로, 센서 핀에서 **5mm 이내**에 배치

---

## 7. ESP32-C3 Super Mini 배선 요약

### 7.1 보드 개요

| 항목 | 값 |
|------|-----|
| 프로세서 | 단일 코어 RISC-V 32비트 160MHz |
| 무선 | WiFi 802.11 b/g/n + **BLE 5.0** |
| 플래시 | 4MB |
| SRAM | 400KB |
| UART 컨트롤러 | 2개 |
| 사용 가능 GPIO | 11개 |
| USB | 네이티브 USB-C (외부 칩 불필요) |
| 크기 | 22.5×18mm |
| 무게 | ~2g |
| 가격 | $1.50~3.00 |
| Arduino 보드 식별자 | `nologo_esp32c3_super_mini` |

### 7.2 핀 배치도

```
         ESP32-C3 Super Mini (상면도)

              USB-C 포트
            ┌────┬────┐
            │    │    │
    ┌───────┤    └    ├───────┐
    │ 5V    │         │ GND   │
    │ GND   │  ESP32  │ 3V3   │
    │ 3V3   │   C3    │ GPIO10│
    │ GPIO2 │  Super  │ GPIO9 │  ← BOOT 버튼
    │ GPIO3 │  Mini   │ GPIO8 │  ← 온보드 LED
    │ GPIO4 │         │ GPIO7 │
    │ GPIO5 │         │ GPIO6 │
    │ GPIO0 │         │ GPIO21│  ← UART0 TX ★
    │ GPIO1 │         │ GPIO20│  ← UART0 RX ★
    └───────┘         └───────┘

    ★ = 이 프로젝트에서 사용하는 핀
```

### 7.3 GPIO 연결 표

| GPIO | 기능 | 연결 대상 | 와이어 색상 (권장) |
|------|------|----------|-----------------|
| **GPIO20** | UART0 RX | JRT U81 TX | 초록 |
| **GPIO21** | UART0 TX | JRT U81 RX | 파랑 |
| **GPIO2** | 디지털 OUT | JRT U81 EN (선택적) | 흰색 |
| **GPIO3** | ADC1 | 배터리 전압 분배기 출력 | 보라 |
| **3V3** | 전원 입력 | MP1584EN VOUT 또는 S7V8F3 VOUT | 노랑 |
| **GND** | 접지 | 공통 GND | 검정 |

> **사용하지 않는 핀**: GPIO0, GPIO1, GPIO4~GPIO10은 미사용. GPIO8은 온보드 LED에 연결되어 있어 상태 표시용으로 활용 가능. GPIO9는 BOOT 버튼에 연결.

### 7.4 전원 입력 방식

**Option A (3S LiPo)**: MP1584EN 3.3V 출력 → ESP32-C3 3V3 핀

**Option B (2× AA)**: Pololu S7V8F3 3.3V 출력 → ESP32-C3 3V3 핀

두 경우 모두 ESP32-C3의 **3V3 핀에 직접** 3.3V를 공급한다. 이렇게 하면 온보드 레귤레이터를 우회하여 효율이 최적화된다.

> **주의**: USB-C 포트와 외부 3.3V를 **동시에** 연결하지 않는다. 프로그래밍 시에는 외부 전원을 분리하고 USB-C만 사용한다.

---

## 8. 전체 시스템 배선도

### 8.1 Option A: 3S LiPo 전체 배선도

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Option A: 3S LiPo 전체 배선도                             │
│                                                                             │
│  ┌──────────┐    ┌──────────────────┐    ┌────────┐    ┌──────────────┐    │
│  │ 3S LiPo  │    │    3S BMS        │    │ 슬라이드│    │  MP1584EN    │    │
│  │ 11.1V    │    │                  │    │ 스위치  │    │  3.3V 고정   │    │
│  │ 1000mAh  │    │ B- ─── P- ──────>│───>│────────>│───>│ VIN    VOUT──┤──┐ │
│  │          │───>│ B1              │    │        │    │              │  │ │
│  │  JST-XH  │───>│ B2              │    │        │    │ GND    GND──┤──┤ │
│  │  밸런스   │───>│ B+ ─── P+ ──────>│───>│────────>│───>│              │  │ │
│  └──────────┘    └──────────────────┘    └────────┘    └──────────────┘  │ │
│                                                                           │ │
│     배터리 전압 모니터링                      3.3V 공통 버스 ◄─────────────┘ │
│     ┌──[100kΩ]──┬──> GPIO3                                                 │
│     │           │    (ADC)                GND 공통 버스 ◄───────────────────┘│
│     │      [27kΩ + 10µF]                                                    │
│     │           │                                                           │
│  P+ ┘       GND ┘                                                           │
│                                                                             │
│  ┌──────────────────────────┐           ┌──────────────────────────┐        │
│  │      JRT U81 센서        │           │   ESP32-C3 Super Mini    │        │
│  │                          │           │                          │        │
│  │  VCC ◄── 3.3V 버스 ─────┼───────────┼──> 3V3                   │        │
│  │  GND ◄── GND 버스 ──────┼───────────┼──> GND                   │        │
│  │  TX  ────────────────────┼───────────┼──> GPIO20 (RX)           │        │
│  │  RX  ◄──────────────────┼───────────┼─── GPIO21 (TX)           │        │
│  │  EN  ◄──────────────────┼───────────┼─── GPIO2 (선택)          │        │
│  │                          │           │                          │        │
│  │  [100µF + 100nF] (VCC-GND)          │  GPIO3 ◄── ADC (전압분배) │        │
│  └──────────────────────────┘           │  GPIO8 = 온보드 LED      │        │
│                                         └──────────────────────────┘        │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 8.2 Option B: 2× AA 전체 배선도

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    Option B: 2× AA 전체 배선도                               │
│                                                                             │
│  ┌──────────────┐    ┌────────┐    ┌─────────────────┐                     │
│  │ 2× AA 홀더   │    │ 슬라이드│    │ Pololu S7V8F3   │                     │
│  │ 3.0V (신품)  │    │ 스위치  │    │ Buck-Boost      │                     │
│  │              │───>│────────>│───>│ VIN    VOUT ────┤──┐                  │
│  │  (+) ────────┤    │        │    │                 │  │                  │
│  │  (-) ────────┤───>│────────>│───>│ GND    GND ────┤──┤                  │
│  └──────────────┘    └────────┘    └─────────────────┘  │                  │
│                                                          │                  │
│     배터리 전압 모니터링                 3.3V 공통 버스 ◄──┘                  │
│     ┌──[220kΩ]──┬──> GPIO3                                                 │
│     │           │    (ADC)           GND 공통 버스 ◄────────────────────────┘│
│     │     [220kΩ + 10µF]                                                    │
│     │           │                                                           │
│  (+)┘       GND ┘                                                           │
│                                                                             │
│  ┌──────────────────────────┐           ┌──────────────────────────┐        │
│  │      JRT U81 센서        │           │   ESP32-C3 Super Mini    │        │
│  │                          │           │                          │        │
│  │  VCC ◄── 3.3V 버스 ─────┼───────────┼──> 3V3                   │        │
│  │  GND ◄── GND 버스 ──────┼───────────┼──> GND                   │        │
│  │  TX  ────────────────────┼───────────┼──> GPIO20 (RX)           │        │
│  │  RX  ◄──────────────────┼───────────┼─── GPIO21 (TX)           │        │
│  │  EN  ◄──────────────────┼───────────┼─── GPIO2 (선택)          │        │
│  │                          │           │                          │        │
│  │  [100µF + 100nF] (VCC-GND)          │  GPIO3 ◄── ADC (전압분배) │        │
│  └──────────────────────────┘           │  GPIO8 = 온보드 LED      │        │
│                                         └──────────────────────────┘        │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 8.3 공통 주의사항

1. **공통 GND**: 모든 모듈(배터리, 컨버터, ESP32, 센서)의 GND를 **하나의 공통 GND 버스**로 연결
2. **UART 길이**: 센서↔ESP32 간 UART 선은 **15cm 이내** 유지 (길면 신호 무결성 저하)
3. **디커플링 캡**: 센서 VCC-GND에 100µF+100nF를 **센서 핀에서 5mm 이내**에 배치
4. **전압 확인 후 연결**: 벅/벅부스트 출력이 3.3V ±0.1V인지 멀티미터로 확인한 **후에** ESP32와 센서를 연결
5. **전원선과 신호선 분리**: 전원선(3.3V, GND)과 UART 신호선은 가능한 이격

---

## 9. 만능기판 조립 가이드

### 9.1 레이아웃 (50×30mm 만능기판)

```
    ← 50mm →
    ┌────────────────────────────────────────────┐  ↑
    │                                            │  │
    │  [전원 입력]      [전원 모듈]               │  │
    │  스크류터미널      MP1584EN 또는             │  │
    │  B+ B-           Pololu S7V8F3             │  │
    │  (또는 홀더 리드)                            │  │
    │                                            │  │
    │  [스위치]  [100µF] [100nF]  [전압분배]      │  30mm
    │                              100k/27k      │  │
    │                              또는 220k/220k │  │
    │                                            │  │
    │  ┌──────────────────────────────────┐      │  │
    │  │     ESP32-C3 Super Mini         │      │  │
    │  │   (핀헤더 소켓으로 탈착 가능)     │      │  │
    │  └──────────────────────────────────┘      │  │
    │                                            │  │
    │  [센서 커넥터: VCC GND TX RX EN]           │  ↓
    └────────────────────────────────────────────┘
```

### 9.2 조립 절차

#### 단계 1: 핀헤더 소켓 납땜

1. **ESP32-C3용 암 핀헤더 소켓** (2×8핀 또는 2×7핀)을 만능기판 중앙~하단에 납땜
   - ESP32를 소켓에 꽂아 정렬 확인 후 납땜
   - 소켓 사용으로 ESP32 탈착이 가능해져 프로그래밍과 디버깅이 편리

#### 단계 2: 전원 모듈 장착

2. **MP1584EN** (Option A) 또는 **Pololu S7V8F3** (Option B)를 상단부에 배치
   - 양면테이프로 고정 후 와이어 납땜
   - 또는 핀 헤더를 통해 만능기판에 직접 장착

#### 단계 3: 스위치 장착

3. **슬라이드 스위치**를 기판 가장자리에 배치 (외부 조작 용이하도록)

#### 단계 4: 센서 커넥터

4. **5핀 핀헤더** (VCC, GND, TX, RX, EN)를 기판 하단에 납땜
   - 센서 연결 와이어를 이 핀헤더에 접속

#### 단계 5: 디커플링 캐패시터

5. **100µF 전해 + 100nF 세라믹**을 센서 커넥터의 VCC-GND 핀에 최대한 가까이 납땜
   - 전해 캐패시터 **극성 주의**: (-) 마킹이 GND 쪽

#### 단계 6: 전압 분배기

6. **저항 2개 + 필터 캐패시터**를 배터리 입력 근처에 납땜
   - Option A: 100kΩ + 27kΩ + 10µF
   - Option B: 220kΩ + 220kΩ + 10µF

#### 단계 7: 전원 버스 배선 (기판 하면)

7. **GND 버스** (검정 와이어): 모든 GND를 연결하는 공통 버스
8. **3.3V 버스** (노랑 와이어): 컨버터 VOUT → ESP32 3V3 + 센서 VCC

#### 단계 8: 신호 배선 (기판 상면)

9. **UART** (초록/파랑 와이어):
   - 센서 TX 핀 → ESP32 GPIO20 (RX)
   - 센서 RX 핀 → ESP32 GPIO21 (TX)
10. **ADC** (보라 와이어): 전압 분배기 출력 → ESP32 GPIO3

#### 단계 9: 전원 입력

11. **스크류 터미널** (2핀) 또는 배터리 리드 직접 납땜
    - Option A: BMS P+, P- 연결
    - Option B: AA 홀더 (+), (-) 연결

### 9.3 배선 규칙

| 규칙 | 상세 |
|------|------|
| **컬러 코드** | 빨강=배터리+, 노랑=3.3V, 검정=GND, 초록=UART RX, 파랑=UART TX |
| **신호/전원 분리** | 신호선은 기판 상면, 전원선은 기판 하면 |
| **UART 길이** | 센서↔ESP32 간 **15cm 이내** |
| **바이패스 캐패시터** | 센서 VCC-GND에 100µF+100nF 병렬 |
| **와이어 게이지** | 전원: 24AWG, 신호: 26AWG |
| **납땜 품질** | 진동 환경에서 사용하므로 모든 조인트를 확실히 납땜. **DuPont 점퍼 와이어 사용 금지** |

---

## 10. 펌웨어: BLE 모드 (Arduino)

### 10.1 개발 환경 설정

**Arduino IDE 설정:**
1. 보드 매니저에서 "esp32" by Espressif Systems v3.x+ 설치
2. 보드 선택: `nologo_esp32c3_super_mini`
3. 라이브러리 매니저에서 설치:
   - `NimBLE-Arduino` (BLE 스택 — Bluedroid 대비 RAM 44%, Flash 46% 절감)

### 10.2 전체 펌웨어 코드

```cpp
/*
 * JRT U81 무선 레이저 거리측정 - ESP32-C3 BLE 브리지
 *
 * 하드웨어: JRT U81 (UART 3.3V) → ESP32-C3 Super Mini
 * 무선: BLE NUS (Nordic UART Service) via NimBLE
 * 배터리 모니터링: GPIO3 ADC
 */

#include <NimBLEDevice.h>

// ─── BLE UUIDs (Nordic UART Service) ───
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // Notify
#define NUS_RX_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Write

// ─── 설정 ───
#define BLE_DEVICE_NAME   "JRT-U81-LDS"
#define SENSOR_BAUD       19200
#define SENSOR_RX_PIN     20    // ESP32 GPIO20 ← 센서 TX
#define SENSOR_TX_PIN     21    // ESP32 GPIO21 → 센서 RX
#define SENSOR_EN_PIN     2     // GPIO2 → 센서 EN (선택)
#define BATTERY_ADC_PIN   3     // GPIO3 ← 전압 분배기
#define LED_PIN           8     // 온보드 LED

#define MEASURE_INTERVAL_MS  200   // 측정 간격 (ms)
#define BATTERY_CHECK_MS     60000 // 배터리 체크 간격 (60초)

// ─── 전역 변수 ───
HardwareSerial LaserSerial(0);
NimBLECharacteristic *pTxChar = nullptr;
bool deviceConnected = false;
uint32_t lastMeasure = 0;
uint32_t lastBattCheck = 0;
float lastBatteryV = 0.0;

// ─── BLE 서버 콜백 ───
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) {
        deviceConnected = true;
        Serial.println("BLE 클라이언트 연결됨");
        pServer->updateConnParams(connInfo.getConnHandle(), 6, 12, 0, 400);
    }
    void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) {
        deviceConnected = false;
        Serial.println("BLE 클라이언트 연결 해제");
        NimBLEDevice::startAdvertising();
    }
};

// ─── RX 콜백 (PC → ESP32 명령 수신) ───
class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) {
        std::string cmd = pChar->getValue();
        if (cmd.length() > 0) {
            LaserSerial.write(cmd[0]);  // 센서에 명령 전달
            Serial.printf("센서 명령 전달: 0x%02X\n", cmd[0]);
        }
    }
};

// ─── 배터리 전압 읽기 ───
float readBatteryVoltage() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += analogRead(BATTERY_ADC_PIN);
    }
    float adcAvg = sum / 16.0;
    float adcV = adcAvg * (3.3 / 4095.0);

    // Option A (3S LiPo, 100k/27k 분배기): V_bat = adcV / 0.2126
    // Option B (2× AA, 220k/220k 분배기): V_bat = adcV * 2.0
    // 아래는 Option A 기준. Option B 사용 시 주석 변경.
    float battV = adcV / 0.2126;  // Option A
    // float battV = adcV * 2.0;  // Option B

    return battV;
}

// ─── 센서 거리 측정 ───
float measureDistance() {
    // UART 버퍼 비우기
    while (LaserSerial.available()) LaserSerial.read();

    // 측정 명령 전송
    LaserSerial.write('D');
    delay(80);  // 센서 응답 대기

    if (LaserSerial.available()) {
        String resp = LaserSerial.readStringUntil('\n');
        int mIdx = resp.indexOf('m');
        if (mIdx > 0) {
            float dist_m = resp.substring(0, mIdx).toFloat();
            return dist_m * 1000.0;  // mm로 변환
        }
    }
    return -1.0;  // 측정 실패
}

void setup() {
    Serial.begin(115200);
    Serial.println("JRT U81 BLE 거리측정 시스템 시작");

    // ─── LED ───
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // LED ON (초기화 중)

    // ─── 센서 EN 핀 ───
    pinMode(SENSOR_EN_PIN, OUTPUT);
    digitalWrite(SENSOR_EN_PIN, HIGH);  // 센서 활성화

    // ─── ADC 설정 ───
    analogSetAttenuation(ADC_11db);  // 0~3.3V 범위

    // ─── UART 초기화 ───
    LaserSerial.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
    delay(200);  // 센서 초기화 대기

    // 센서 초기화 응답 읽기
    if (LaserSerial.available()) {
        String initResp = LaserSerial.readStringUntil('\n');
        Serial.printf("센서 응답: %s\n", initResp.c_str());
    }

    // 레이저 ON
    LaserSerial.write('O');
    delay(100);
    if (LaserSerial.available()) {
        String resp = LaserSerial.readStringUntil('\n');
        Serial.printf("레이저 ON: %s\n", resp.c_str());
    }

    // ─── BLE 초기화 ───
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(128);

    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService *pService = pServer->createService(NUS_SERVICE_UUID);

    pTxChar = pService->createCharacteristic(
        NUS_TX_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    NimBLECharacteristic *pRxChar = pService->createCharacteristic(
        NUS_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pRxChar->setCallbacks(new RxCallbacks());

    pService->start();

    NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(NUS_SERVICE_UUID);
    pAdv->setName(BLE_DEVICE_NAME);
    pAdv->start();

    Serial.printf("BLE 광고 시작: '%s'\n", BLE_DEVICE_NAME);
    digitalWrite(LED_PIN, LOW);  // LED OFF (초기화 완료)

    // 초기 배터리 전압 읽기
    lastBatteryV = readBatteryVoltage();
    Serial.printf("배터리 전압: %.2fV\n", lastBatteryV);
}

void loop() {
    uint32_t now = millis();

    // ─── 주기적 거리 측정 ───
    if (now - lastMeasure >= MEASURE_INTERVAL_MS) {
        lastMeasure = now;

        float dist_mm = measureDistance();

        if (dist_mm >= 0 && deviceConnected) {
            // 13바이트 패킷: [ID(1)] [distance_mm(4)] [timestamp(4)] [battery_v(4)]
            uint8_t pkt[13];
            pkt[0] = 0x01;  // 센서 ID
            memcpy(&pkt[1], &dist_mm, 4);
            uint32_t ts = now;
            memcpy(&pkt[5], &ts, 4);
            memcpy(&pkt[9], &lastBatteryV, 4);

            pTxChar->setValue(pkt, 13);
            pTxChar->notify();
        }

        // LED 깜빡임 (연결 상태 표시)
        if (deviceConnected) {
            digitalWrite(LED_PIN, (now / 1000) % 2);  // 1초 간격 깜빡임
        } else {
            digitalWrite(LED_PIN, (now / 200) % 2);   // 빠른 깜빡임 (미연결)
        }
    }

    // ─── 주기적 배터리 체크 ───
    if (now - lastBattCheck >= BATTERY_CHECK_MS) {
        lastBattCheck = now;
        lastBatteryV = readBatteryVoltage();
        Serial.printf("배터리: %.2fV\n", lastBatteryV);
    }

    delay(1);
}
```

### 10.3 코드 설명

**UART 통신 흐름:**
1. `LaserSerial.write('D')` — 센서에 측정 명령 전송
2. 80ms 대기 (센서 응답 시간)
3. 응답 문자열에서 `'m'` 위치를 찾아 거리 파싱
4. mm 단위로 변환 후 BLE로 전송

**BLE 패킷 구조 (13바이트):**

| 바이트 | 내용 | 타입 | 설명 |
|--------|------|------|------|
| 0 | 센서 ID | uint8 | 0x01 (고정) |
| 1~4 | 거리 (mm) | float32 | 리틀엔디안 |
| 5~8 | 타임스탬프 (ms) | uint32 | millis() |
| 9~12 | 배터리 전압 (V) | float32 | 리틀엔디안 |

### 10.4 펌웨어 업로드 절차

1. ESP32-C3를 USB-C로 PC에 연결
2. **만능기판에서 외부 전원 와이어를 분리** (USB 전원과 충돌 방지)
3. Arduino IDE에서:
   - 보드: `nologo_esp32c3_super_mini`
   - 포트: 해당 시리얼 포트 선택
   - Upload Speed: 460800
4. 첫 업로드 시 **BOOT 버튼**을 누른 채 업로드 시작, "Connecting..." 표시 후 버튼 해제
5. 업로드 완료 후 시리얼 모니터(115200)에서 초기화 메시지 확인
6. USB 분리 후 외부 전원 와이어 재연결

### 10.5 주요 설정 조정

| 변경 사항 | 코드 수정 위치 | 기본값 | 범위 |
|----------|--------------|--------|------|
| BLE 이름 | `BLE_DEVICE_NAME` | "JRT-U81-LDS" | 자유 |
| 측정 간격 | `MEASURE_INTERVAL_MS` | 200ms (5Hz) | 100~2000ms |
| 배터리 체크 간격 | `BATTERY_CHECK_MS` | 60000ms (1분) | 10000~300000 |
| 배터리 옵션 | `readBatteryVoltage()` 내부 | Option A | A 또는 B |
| 센서 보레이트 | `SENSOR_BAUD` | 19200 | 센서 설정에 따라 |

---

## 11. 펌웨어: WiFi UDP 모드 (Arduino)

### 11.1 개발 환경 설정

BLE와 동일한 보드/환경이지만 NimBLE 라이브러리는 불필요하다. WiFi.h와 WiFiUdp.h는 ESP32 코어에 내장.

### 11.2 전체 펌웨어 코드

```cpp
/*
 * JRT U81 무선 레이저 거리측정 - ESP32-C3 WiFi UDP 브리지
 *
 * 하드웨어: JRT U81 (UART 3.3V) → ESP32-C3 Super Mini
 * 무선: WiFi UDP (3~8ms 지연시간)
 */

#include <WiFi.h>
#include <WiFiUdp.h>

// ─── WiFi 설정 ───
const char* WIFI_SSID = "YourNetworkSSID";    // ← 수정 필요
const char* WIFI_PASS = "YourNetworkPassword"; // ← 수정 필요

// ─── UDP 설정 ───
const IPAddress PC_IP(192, 168, 1, 100);  // ← PC IP 주소 수정 필요
const uint16_t UDP_PORT = 4210;

// ─── 핀 설정 ───
#define SENSOR_BAUD       19200
#define SENSOR_RX_PIN     20
#define SENSOR_TX_PIN     21
#define SENSOR_EN_PIN     2
#define BATTERY_ADC_PIN   3
#define LED_PIN           8

#define MEASURE_INTERVAL_MS  100   // 100ms (10Hz)
#define BATTERY_CHECK_MS     60000

// ─── 전역 변수 ───
HardwareSerial LaserSerial(0);
WiFiUDP udp;
uint32_t lastMeasure = 0;
uint32_t lastBattCheck = 0;
float lastBatteryV = 0.0;

// ─── 배터리 전압 읽기 ───
float readBatteryVoltage() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += analogRead(BATTERY_ADC_PIN);
    }
    float adcAvg = sum / 16.0;
    float adcV = adcAvg * (3.3 / 4095.0);
    float battV = adcV / 0.2126;  // Option A (3S LiPo)
    // float battV = adcV * 2.0;  // Option B (2× AA)
    return battV;
}

// ─── WiFi 연결 ───
void connectWiFi() {
    Serial.printf("WiFi 연결 중: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi 연결 완료! IP: %s\n", WiFi.localIP().toString().c_str());
        digitalWrite(LED_PIN, LOW);
    } else {
        Serial.println("\nWiFi 연결 실패!");
        digitalWrite(LED_PIN, HIGH);
    }
}

// ─── 센서 거리 측정 ───
float measureDistance() {
    while (LaserSerial.available()) LaserSerial.read();
    LaserSerial.write('D');
    delay(50);

    if (LaserSerial.available()) {
        String resp = LaserSerial.readStringUntil('\n');
        int mIdx = resp.indexOf('m');
        if (mIdx > 0) {
            float dist_m = resp.substring(0, mIdx).toFloat();
            return dist_m * 1000.0;
        }
    }
    return -1.0;
}

void setup() {
    Serial.begin(115200);
    Serial.println("JRT U81 WiFi UDP 거리측정 시스템 시작");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    pinMode(SENSOR_EN_PIN, OUTPUT);
    digitalWrite(SENSOR_EN_PIN, HIGH);

    analogSetAttenuation(ADC_11db);

    // UART 초기화
    LaserSerial.begin(SENSOR_BAUD, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
    delay(200);

    // 센서 초기화 응답
    if (LaserSerial.available()) {
        String initResp = LaserSerial.readStringUntil('\n');
        Serial.printf("센서 응답: %s\n", initResp.c_str());
    }

    // 레이저 ON
    LaserSerial.write('O');
    delay(100);
    if (LaserSerial.available()) {
        String resp = LaserSerial.readStringUntil('\n');
        Serial.printf("레이저 ON: %s\n", resp.c_str());
    }

    // WiFi 연결
    connectWiFi();
    udp.begin(UDP_PORT);

    // 초기 배터리 전압
    lastBatteryV = readBatteryVoltage();
    Serial.printf("배터리: %.2fV\n", lastBatteryV);

    digitalWrite(LED_PIN, LOW);
}

void loop() {
    uint32_t now = millis();

    // WiFi 재연결
    if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
    }

    // 주기적 측정
    if (now - lastMeasure >= MEASURE_INTERVAL_MS) {
        lastMeasure = now;

        float dist_mm = measureDistance();

        if (dist_mm >= 0 && WiFi.status() == WL_CONNECTED) {
            // CSV 형식: "거리mm,배터리V,타임스탬프ms\n"
            char msg[64];
            snprintf(msg, sizeof(msg), "%.1f,%.2f,%lu\n", dist_mm, lastBatteryV, now);

            udp.beginPacket(PC_IP, UDP_PORT);
            udp.write((uint8_t*)msg, strlen(msg));
            udp.endPacket();
        }

        // LED 상태
        if (WiFi.status() == WL_CONNECTED) {
            digitalWrite(LED_PIN, (now / 1000) % 2);
        } else {
            digitalWrite(LED_PIN, (now / 200) % 2);
        }
    }

    // 배터리 체크
    if (now - lastBattCheck >= BATTERY_CHECK_MS) {
        lastBattCheck = now;
        lastBatteryV = readBatteryVoltage();
    }

    delay(1);
}
```

### 11.3 코드 설명

**WiFi UDP vs BLE의 차이점:**
- WiFi는 `WiFi.begin()` + `udp.beginPacket()`으로 데이터 전송 — 코드가 더 간결
- UDP는 텍스트 CSV 형식 (`"거리,배터리,타임스탬프\n"`) — 디버깅이 쉬움
- BLE는 바이너리 패킷 (13바이트) — 효율적이지만 파싱 필요
- WiFi 측정 간격을 100ms(10Hz)로 설정 — BLE보다 빠른 업데이트 가능

### 11.4 WiFi AP 모드 대안

WiFi AP가 없는 환경에서는 ESP32 자체를 AP로 사용할 수 있다. `setup()`에서 다음으로 교체:

```cpp
// Station 모드 대신 AP 모드
WiFi.softAP("JRT-U81-AP", "password123");
Serial.printf("AP 시작! IP: %s\n", WiFi.softAPIP().toString().c_str());
// 기본 IP: 192.168.4.1, PC에서 이 AP에 연결 후 UDP 수신
```

이 경우 PC를 ESP32의 AP에 연결하고, `PC_IP`를 PC가 할당받는 IP(보통 192.168.4.2)로 설정한다.

### 11.5 주요 설정 조정

| 변경 사항 | 코드 수정 위치 | 기본값 | 범위 |
|----------|--------------|--------|------|
| WiFi SSID | `WIFI_SSID` | 수정 필요 | 문자열 |
| WiFi 비밀번호 | `WIFI_PASS` | 수정 필요 | 문자열 |
| PC IP 주소 | `PC_IP` | 192.168.1.100 | PC 실제 IP |
| UDP 포트 | `UDP_PORT` | 4210 | 1024~65535 |
| 측정 간격 | `MEASURE_INTERVAL_MS` | 100ms (10Hz) | 50~2000ms |

---

## 12. PC 수신 소프트웨어: BLE (Python)

### 12.1 환경 설정

```bash
pip install bleak
```

Linux에서는 BlueZ 5.43 이상이 필요하다:
```bash
bluetoothd --version   # 5.43 이상 확인
```

### 12.2 실시간 수신 + CSV 로깅

```python
"""
JRT U81 BLE 수신기 - 실시간 데이터 표시 + CSV 로깅
"""
import asyncio
import struct
import csv
import time
from datetime import datetime
from bleak import BleakClient, BleakScanner

# BLE UUID
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

# 센서 디바이스 이름
DEVICE_NAME = "JRT-U81-LDS"

# CSV 파일
CSV_FILE = f"jrt_u81_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"


class SensorReceiver:
    def __init__(self):
        self.count = 0
        self.csv_writer = None
        self.csv_file = None
        self.start_time = time.time()

    def start_csv(self):
        self.csv_file = open(CSV_FILE, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'pc_time_s', 'sensor_id', 'distance_mm',
            'sensor_timestamp_ms', 'battery_v'
        ])

    def on_data(self, sender, data: bytearray):
        """BLE 데이터 수신 콜백 — 13바이트 패킷 파싱"""
        if len(data) < 13:
            return

        sensor_id = data[0]
        distance_mm = struct.unpack('<f', data[1:5])[0]
        timestamp_ms = struct.unpack('<I', data[5:9])[0]
        battery_v = struct.unpack('<f', data[9:13])[0]

        self.count += 1
        pc_time = time.time() - self.start_time

        # 콘솔 출력 (5회마다)
        if self.count % 5 == 0:
            print(f"[{pc_time:8.2f}s] 거리: {distance_mm:8.1f} mm  "
                  f"배터리: {battery_v:.2f}V  "
                  f"({self.count/pc_time:.1f} Hz)")

        # CSV 기록
        if self.csv_writer:
            self.csv_writer.writerow([
                f"{pc_time:.4f}", sensor_id,
                f"{distance_mm:.1f}", timestamp_ms,
                f"{battery_v:.2f}"
            ])

    def close(self):
        if self.csv_file:
            self.csv_file.close()
            print(f"\nCSV 저장 완료: {CSV_FILE} ({self.count}개 샘플)")


async def main():
    receiver = SensorReceiver()

    print(f"BLE 디바이스 검색 중: '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)

    if not device:
        print(f"ERROR: '{DEVICE_NAME}' 디바이스를 찾을 수 없습니다.")
        print("확인사항:")
        print("  1. ESP32가 전원에 연결되어 있는지")
        print("  2. 시리얼 모니터에서 'BLE 광고 시작' 메시지가 표시되는지")
        print("  3. 다른 디바이스가 이미 연결하고 있지 않은지")
        return

    print(f"디바이스 발견: {device.name} ({device.address})")
    print("연결 중...")

    async with BleakClient(device) as client:
        print(f"연결 완료! MTU: {client.mtu_size}")
        print(f"데이터 수신 시작... (Ctrl+C로 종료)\n")
        print(f"{'시간':>10s}  {'거리(mm)':>10s}  {'배터리(V)':>10s}  {'수신율':>8s}")
        print("-" * 45)

        receiver.start_csv()
        await client.start_notify(NUS_TX, receiver.on_data)

        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            print("\n\n수신 중단.")

        await client.stop_notify(NUS_TX)

    receiver.close()


if __name__ == "__main__":
    asyncio.run(main())
```

### 12.3 사용법

```bash
python ble_receiver.py
```

**예상 출력:**
```
BLE 디바이스 검색 중: 'JRT-U81-LDS'...
디바이스 발견: JRT-U81-LDS (AA:BB:CC:DD:EE:FF)
연결 중...
연결 완료! MTU: 128
데이터 수신 시작... (Ctrl+C로 종료)

      시간    거리(mm)   배터리(V)    수신율
---------------------------------------------
[    1.05s] 거리:   1234.5 mm  배터리: 11.85V  (4.8 Hz)
[    2.10s] 거리:   1234.3 mm  배터리: 11.85V  (4.8 Hz)
```

`Ctrl+C`로 종료하면 CSV 파일이 자동 저장된다.

---

## 13. PC 수신 소프트웨어: WiFi UDP (Python)

### 13.1 환경 설정

추가 패키지 설치 불필요 — 표준 라이브러리 `socket`만 사용.

**방화벽 설정**: PC에서 UDP 포트 4210 인바운드를 허용해야 한다.
```bash
# Linux (ufw)
sudo ufw allow 4210/udp
```

### 13.2 실시간 수신 + CSV 로깅

```python
"""
JRT U81 WiFi UDP 수신기 - 실시간 데이터 표시 + CSV 로깅
"""
import socket
import csv
import time
from datetime import datetime

UDP_PORT = 4210
CSV_FILE = f"jrt_u81_udp_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(2.0)

    csv_file = open(CSV_FILE, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['pc_time_s', 'distance_mm', 'battery_v', 'sensor_timestamp_ms'])

    count = 0
    start_time = time.time()

    print(f"UDP 포트 {UDP_PORT}에서 수신 대기 중...")
    print(f"{'시간':>10s}  {'거리(mm)':>10s}  {'배터리(V)':>10s}  {'수신율':>8s}")
    print("-" * 45)

    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                msg = data.decode('utf-8').strip()

                # CSV 형식: "거리mm,배터리V,타임스탬프ms"
                parts = msg.split(',')
                if len(parts) >= 3:
                    dist_mm = float(parts[0])
                    batt_v = float(parts[1])
                    sensor_ts = int(parts[2])

                    count += 1
                    pc_time = time.time() - start_time

                    if count % 5 == 0:
                        hz = count / pc_time if pc_time > 0 else 0
                        print(f"[{pc_time:8.2f}s] 거리: {dist_mm:8.1f} mm  "
                              f"배터리: {batt_v:.2f}V  "
                              f"({hz:.1f} Hz)")

                    csv_writer.writerow([
                        f"{pc_time:.4f}", f"{dist_mm:.1f}",
                        f"{batt_v:.2f}", sensor_ts
                    ])

            except socket.timeout:
                pass

    except KeyboardInterrupt:
        print(f"\n\n수신 중단. CSV 저장: {CSV_FILE} ({count}개 샘플)")

    csv_file.close()
    sock.close()


if __name__ == "__main__":
    main()
```

### 13.3 사용법

```bash
python udp_receiver.py
```

**참고**: PC와 ESP32가 **같은 네트워크**에 있어야 한다. ESP32 펌웨어의 `PC_IP`가 실제 PC IP와 일치하는지 확인.

### 13.4 ROS2 노드 (선택사항)

`sensor_msgs/msg/Range` 메시지로 퍼블리시하는 ROS2 노드:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import socket

class LaserBridgeNode(Node):
    def __init__(self):
        super().__init__('jrt_u81_bridge')
        self.declare_parameter('udp_port', 4210)
        self.declare_parameter('frame_id', 'laser_sensor')

        self.pub = self.create_publisher(Range, 'laser_distance', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.get_parameter('udp_port').value))
        self.sock.settimeout(0.05)
        self.timer = self.create_timer(0.01, self.poll)

    def poll(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            parts = data.decode().strip().split(',')
            if len(parts) >= 1:
                dist_m = float(parts[0]) / 1000.0
                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.get_parameter('frame_id').value
                msg.radiation_type = Range.INFRARED
                msg.field_of_view = 0.003
                msg.min_range = 0.03
                msg.max_range = 20.0
                msg.range = dist_m
                self.pub.publish(msg)
        except socket.timeout:
            pass

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LaserBridgeNode())
    rclpy.shutdown()
```

실행: `ros2 run your_package jrt_u81_bridge` → `ros2 topic echo /laser_distance`

---

## 14. 배터리 모니터링 코드

### 14.1 ESP32-C3 ADC 특성

| 항목 | 값 |
|------|-----|
| ADC1 채널 | GPIO0, GPIO1, GPIO2, GPIO3, GPIO4 |
| 분해능 | 12비트 (0~4095) |
| 기준 전압 | ~3.3V (ADC_11db 감쇠) |
| 권장 범위 | 0.1V ~ 2.6V (비선형 구간 회피) |
| 샘플링 | 소프트웨어 트리거, 16회 평균 권장 |

> **GPIO2는 스트래핑 핀**이므로 ADC에는 **GPIO3 또는 GPIO4**를 권장한다.

### 14.2 Option A: 3S LiPo 전압 모니터링

```cpp
// Option A: 100kΩ / 27kΩ 분배기
float readBattery_OptionA() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += analogRead(3);  // GPIO3
        delayMicroseconds(100);
    }
    float adcAvg = sum / 16.0;
    float adcV = adcAvg * (3.3 / 4095.0);
    float battV = adcV * (100.0 + 27.0) / 27.0;  // = adcV / 0.2126

    // 잔량 계산 (선형 근사)
    float pct = (battV - 9.0) / (12.6 - 9.0) * 100.0;
    pct = constrain(pct, 0.0, 100.0);

    return battV;
}
```

### 14.3 Option B: 2× AA 전압 모니터링

```cpp
// Option B: 220kΩ / 220kΩ 분배기 (1:1)
float readBattery_OptionB() {
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += analogRead(3);  // GPIO3
        delayMicroseconds(100);
    }
    float adcAvg = sum / 16.0;
    float adcV = adcAvg * (3.3 / 4095.0);
    float battV = adcV * 2.0;  // 1:1 분배기

    // 잔량 계산 (알카라인 근사)
    float pct = (battV - 1.8) / (3.0 - 1.8) * 100.0;
    pct = constrain(pct, 0.0, 100.0);

    return battV;
}
```

### 14.4 무선 데이터에 배터리 전압 포함

**BLE 모드**: 이미 13바이트 패킷의 바이트 9~12에 `float battery_v`가 포함되어 있다.

**WiFi UDP 모드**: CSV 문자열의 두 번째 필드에 포함: `"거리mm,배터리V,타임스탬프ms"`

### 14.5 저전압 자동 차단

배터리가 임계치 이하로 떨어지면 ESP32를 딥 슬립으로 전환하여 과방전을 방지한다.

```cpp
#define LOW_BATT_THRESHOLD_A  9.5   // Option A: 3S LiPo 컷오프
#define LOW_BATT_THRESHOLD_B  2.2   // Option B: 2× AA 컷오프

void checkLowBattery(float battV) {
    float threshold = LOW_BATT_THRESHOLD_A;  // 또는 _B

    if (battV < threshold && battV > 1.0) {  // 1.0V 미만은 미연결로 판단
        Serial.printf("⚠ 저전압 경고: %.2fV < %.1fV\n", battV, threshold);
        Serial.println("10초 후 딥 슬립 진입...");

        // BLE/WiFi로 저전압 알림 전송
        // ... (현재 통신 모드에 따라)

        delay(10000);

        // 딥 슬립 진입 (외부 버튼으로 웨이크업)
        esp_deep_sleep_start();
    }
}
```

딥 슬립 모드에서는 소비 전류가 ~8µA로 감소하여 배터리 과방전을 방지한다.

---

## 15. 물리적 조립 및 마운팅

### 15.1 인클로저 설계

| 항목 | Option A (3S LiPo) | Option B (2× AA) |
|------|--------------------|--------------------|
| 내부 치수 | ~80×40×25mm | ~65×35×20mm |
| 소재 | PETG (내열 70~80°C) | PETG |
| 벽 두께 | 2.5mm | 2.0mm |

**설계 요소:**
- **레이저 개구부**: JRT U81 레이저 렌즈에 맞춰 정면에 구멍 (Ø8mm)
- **USB-C 접근**: ESP32 프로그래밍/디버깅을 위한 개구부 (측면)
- **스위치 접근**: 슬라이드 스위치 조작 홀 (상면 또는 측면)
- **배터리 교체** (Option B): 스냅핏 뚜껑 또는 나사 고정 뚜껑
- **통기구**: 장시간 동작 시 열 방출용 슬릿 (선택)

### 15.2 마운팅 방식

**M2/M3 나사 + 스탠드오프**: 로봇 플랜지 또는 툴 어댑터 플레이트에 나사 고정
- 인클로저 4모서리에 Ø2.5mm 구멍
- M2 스크류 + 8mm 스탠드오프로 고정

**자석 마운팅 (대안)**: N52 네오디뮴 자석 (Ø6×3mm) × 4개를 인클로저 저면에 부착
- 철재 표면에 즉시 장착/분리
- 위치 미세 조정이 쉬움

**레이저 정렬**: 레이저 개구부가 **측정 대상 표면**에 대해 시야가 확보되도록 배치. 레이저 빔이 표면에 **수직**으로 입사하는 것이 가장 정확하다.

### 15.3 진동 환경 대응

- **DuPont 점퍼 와이어 사용 금지** — 로봇 가속도에서 빠진다
- **모든 연결은 납땜** — 스크류 터미널은 센서 케이블 진입부에만 사용
- **스트레인 릴리프**: 외부 케이블은 케이블 타이나 클램프로 고정
- **나사 잠금**: 진동 환경에서는 나사 잠금제 (Loctite 222 등) 사용 권장
- **컨포멀 코팅** (선택): 만능기판 전체에 실리콘 컨포멀 코팅 스프레이 → 습기/진동 추가 보호

---

## 16. 테스트 및 검증 절차

### 16.1 단계별 통전 테스트

#### Step 1: 전원 모듈 독립 테스트

```
테스트 1-1: 벅/벅부스트 출력 전압 확인
  연결: 배터리 → (BMS) → 스위치 → 컨버터
  측정: 멀티미터로 VOUT-GND
  ✅ Pass: 3.3V ±0.1V
  ❌ Fail: 모듈 방향, 입력 전압, 납땜 확인
```

#### Step 2: 센서 전원 테스트

```
테스트 2-1: JRT U81 전원 인가
  연결: 3.3V → 센서 VCC, GND → 센서 GND
  확인: 100ms 이후 센서 초기화
  ✅ Pass: 센서 TX에서 ",OK!" 전송 (USB-UART 어댑터로 확인 가능)
  ❌ Fail: VCC/GND 극성, 센서 핀에서 3.3V 확인
```

#### Step 3: UART 통신 테스트

```
테스트 3-1: ESP32 ↔ 센서 UART 통신
  코드: 부록 A의 UART 테스트 코드 업로드
  전송: 'O' 명령
  ✅ Pass: ",OK" 응답 수신
  ❌ Fail: TX/RX 크로스 확인, 보레이트 19200 확인

테스트 3-2: 거리 측정
  전송: 'D' 명령
  ✅ Pass: "X.XXXm, XXXX" 형식 응답 (예: "1.234m, 0079")
  ❌ Fail: 레이저 ON ('O') 명령 선행 확인, 타겟 존재 확인
```

#### Step 4: 무선 전송 테스트

```
테스트 4-1: BLE 광고 확인 (BLE 모드)
  도구: 스마트폰 nRF Connect 앱
  확인: "JRT-U81-LDS" 디바이스 스캔 결과에 표시
  ✅ Pass: 디바이스 발견
  ❌ Fail: BLE 초기화 코드, ESP32 전원 확인

테스트 4-2: WiFi 연결 확인 (WiFi 모드)
  확인: 시리얼 모니터에서 "WiFi 연결 완료! IP: x.x.x.x" 표시
  ✅ Pass: IP 주소 할당됨
  ❌ Fail: SSID/비밀번호 확인, WiFi 신호 강도

테스트 4-3: PC 데이터 수신
  실행: python ble_receiver.py 또는 python udp_receiver.py
  확인: 콘솔에 거리값 표시
  ✅ Pass: 연속 데이터 수신 (기대 주파수의 90% 이상)
  ❌ Fail: BLE 연결/WiFi 네트워크 확인, 방화벽 확인 (UDP)
```

### 16.2 최종 성능 검증

#### 정밀도 테스트

1. 기지 거리(예: 1.000m)에 고정 타겟 배치
2. 100회 연속 측정
3. 표준편차(σ) 계산

```
✅ Pass: σ ≤ 1mm (센서 스펙 내)
⚠ Warning: 1mm < σ ≤ 2mm (전원 노이즈 또는 디커플링 확인)
❌ Fail: σ > 2mm (배선/전원/타겟 반사율 점검)
```

#### 연속 동작 테스트

1. 1시간 연속 데이터 수신
2. 패킷 손실률 계산

```
✅ Pass: 손실률 < 0.1%
⚠ Warning: 0.1% ≤ 손실률 < 1%
❌ Fail: 손실률 ≥ 1% (무선 간섭, 안테나 방향 확인)
```

#### 배터리 테스트

1. 완충 상태에서 시작
2. 자동 차단까지 연속 동작 시간 측정
3. 전압 변화 로그 확인

---

## 17. 트러블슈팅

### 17.1 센서 통신 실패

| 증상 | 원인 | 해결 |
|------|------|------|
| 센서 응답 없음 | TX/RX 뒤바뀜 | GPIO20↔GPIO21 연결 스왑 |
| 센서 응답 없음 | 보레이트 불일치 | `LaserSerial.begin(19200...)` 확인 |
| 깨진 문자 수신 | 보레이트 불일치 | 19200 bps 8N1 설정 확인 |
| 센서 전원 안 들어옴 | 전압 부족 | 센서 VCC 핀에서 3.3V 확인 |
| `,ERR` 응답 | 레이저 차단 또는 범위 초과 | 타겟 거리/반사율 확인 |
| 센서 초기화 안 됨 | 전원 인가 후 대기 부족 | `delay(200)` 이상으로 증가 |

### 17.2 무선 연결 문제

| 증상 | 원인 | 해결 |
|------|------|------|
| BLE 디바이스 미검색 | BLE 초기화 실패 | NimBLE 코드, ESP32 전원 확인 |
| BLE 간헐적 끊김 | 거리/간섭 | 10m 이내 접근, WiFi 간섭 줄이기 |
| WiFi 연결 안 됨 | SSID/비밀번호 오류 | 코드 내 자격증명 확인 |
| WiFi 연결됨, UDP 수신 없음 | 방화벽 차단 | PC에서 UDP 4210 포트 허용 |
| WiFi 연결됨, UDP 수신 없음 | PC IP 불일치 | 펌웨어의 `PC_IP`가 실제 PC IP와 일치하는지 확인 |
| 높은 패킷 손실 | WiFi 혼잡 | 5GHz 대역 사용, 측정 주기 늘리기 |

### 17.3 전원 문제

| 증상 | 원인 | 해결 |
|------|------|------|
| ESP32 랜덤 리셋 | 전류 부족 | 디커플링 캡 추가, 컨버터 정격 확인 |
| WiFi TX 시 전압 강하 | 전류 스파이크 >300mA | 470µF 캡을 3.3V 레일에 추가 |
| 배터리 빠른 소모 | WiFi 상시 활성 | BLE 모드 전환, 또는 슬립 주기 추가 |
| ADC 전압 읽기 부정확 | 비선형성/노이즈 | 부록 B 캘리브레이션 수행 |
| 스위치 동작 불량 | MOSFET 배선 오류 | AO3401 핀 배치(Gate/Source/Drain) 확인 |
| 3.3V 출력 불안정 | 가변형 벅 모듈 사용 | 반드시 **고정 3.3V** 모듈로 교체 |

### 17.4 측정값 이상

| 증상 | 원인 | 해결 |
|------|------|------|
| 항상 0 또는 ERROR | 타겟 없음 | 측정 범위 내 반사 타겟 배치 |
| 노이즈 심함 (σ >2mm) | 전원 노이즈 | 디커플링 캡 확인/추가 |
| 일정 오프셋 | 센서 보정 필요 | 'S' 명령으로 상태 확인 |
| 신호 품질 값 매우 높음 | 타겟 반사율 부족 | 밝은색/반사율 높은 타겟 사용 |
| 간헐적 측정 실패 | UART 타이밍 | 응답 대기 시간 `delay(80)` 증가 (100~150ms) |

---

## 부록 A: UART 통신 테스트 코드

ESP32에 업로드하여 센서와의 UART 통신을 독립적으로 검증하는 최소 스케치.

```cpp
/*
 * JRT U81 UART 통신 테스트
 * 시리얼 모니터에서 결과 확인 (115200 bps)
 */

HardwareSerial LaserSerial(0);

void sendCmd(char cmd, const char* desc) {
    Serial.printf("\n--- %s (0x%02X) ---\n", desc, cmd);

    while (LaserSerial.available()) LaserSerial.read();  // 버퍼 비우기
    LaserSerial.write(cmd);
    delay(200);

    String resp = "";
    while (LaserSerial.available()) {
        char c = LaserSerial.read();
        resp += c;
    }
    Serial.printf("응답: [%s] (%d 바이트)\n", resp.c_str(), resp.length());
}

void setup() {
    Serial.begin(115200);
    LaserSerial.begin(19200, SERIAL_8N1, 20, 21);  // RX=GPIO20, TX=GPIO21

    Serial.println("\n=== JRT U81 UART 통신 테스트 시작 ===");
    delay(300);  // 센서 초기화 대기

    // 초기화 응답 읽기
    if (LaserSerial.available()) {
        String init = LaserSerial.readStringUntil('\n');
        Serial.printf("초기화 응답: %s\n", init.c_str());
    }
}

void loop() {
    sendCmd('O', "레이저 ON");
    delay(500);

    sendCmd('D', "거리 측정");
    delay(500);

    sendCmd('D', "거리 측정 (2회차)");
    delay(500);

    sendCmd('S', "상태 조회");
    delay(500);

    sendCmd('C', "레이저 OFF");
    delay(3000);

    Serial.println("\n========== 다음 사이클 ==========");
}
```

**사용법**: 시리얼 모니터(115200 bps)에서 각 명령의 응답을 확인한다.

---

## 부록 B: 배터리 전압 캘리브레이션 코드

ESP32-C3의 ADC는 비선형성이 있으므로 실측 보정이 권장된다.

```cpp
/*
 * 배터리 전압 ADC 캘리브레이션
 * 멀티미터로 측정한 실제 전압과 비교하여 보정 계수를 계산
 */

#define ADC_PIN  3
#define DIVIDER_RATIO  0.2126  // Option A: 27/(100+27)
// #define DIVIDER_RATIO  0.5  // Option B: 220/(220+220)

void setup() {
    Serial.begin(115200);
    analogSetAttenuation(ADC_11db);
    Serial.println("\n=== 배터리 전압 캘리브레이션 ===");
    Serial.println("멀티미터로 배터리 전압을 측정하여 비교하세요.\n");
}

void loop() {
    // 100회 샘플링 평균
    uint32_t sum = 0;
    for (int i = 0; i < 100; i++) {
        sum += analogRead(ADC_PIN);
        delayMicroseconds(500);
    }
    float adcAvg = sum / 100.0;
    float adcV = adcAvg * (3.3 / 4095.0);
    float calcBattV = adcV / DIVIDER_RATIO;

    Serial.printf("ADC 원시값: %.1f  |  ADC 전압: %.4fV  |  계산된 배터리: %.3fV\n",
                  adcAvg, adcV, calcBattV);
    Serial.println("→ 멀티미터 실측값과 비교하여 보정 계수 계산:");
    Serial.println("   보정계수 = 멀티미터값 / 계산값");
    Serial.println("   코드에서: battV = calcBattV * 보정계수;\n");

    delay(2000);
}
```

**사용법:**
1. 코드 업로드 후 시리얼 모니터 확인
2. 멀티미터로 배터리 실측 전압 확인
3. `보정계수 = 멀티미터값 / 계산값`
4. 메인 펌웨어의 `readBatteryVoltage()`에 보정계수 적용: `battV *= 보정계수;`

---

## 부록 C: 완성 시스템 체크리스트

### 하드웨어 조립

- [ ] 전원 모듈 조립 및 3.3V 출력 검증 (멀티미터)
- [ ] 슬라이드 스위치 ON/OFF 동작 확인
- [ ] JRT U81 센서 배선 완료 (VCC, GND, TX→GPIO20, RX→GPIO21)
- [ ] 디커플링 캐패시터 장착 (100µF + 100nF, 센서 VCC-GND)
- [ ] 전압 분배기 조립 및 ADC 핀(GPIO3) 연결
- [ ] 모든 GND 공통 버스 연결 확인
- [ ] 납땜 품질 점검 (콜드 솔더, 브릿지 없는지)

### 소프트웨어

- [ ] Arduino IDE 설정: 보드 `nologo_esp32c3_super_mini`, ESP32 코어 v3.x+
- [ ] NimBLE-Arduino 라이브러리 설치 (BLE 모드 사용 시)
- [ ] 펌웨어 업로드 성공 (BLE 또는 WiFi UDP)
- [ ] 시리얼 모니터에서 초기화 메시지 확인
- [ ] 배터리 옵션에 맞게 `readBatteryVoltage()` 설정

### 통신 테스트

- [ ] UART 통신: 'O' → `,OK`, 'D' → 거리값 응답
- [ ] BLE: nRF Connect에서 디바이스 발견 (또는 WiFi: IP 할당 확인)
- [ ] PC 수신기: Python 스크립트로 실시간 데이터 수신

### 성능 검증

- [ ] 정밀도: 고정 타겟 100회 측정, σ ≤ 1mm
- [ ] 연속 동작: 1시간 패킷 손실률 < 0.1%
- [ ] 배터리 모니터링: 전압 읽기 정상 (멀티미터 대조)

### 최종 조립

- [ ] 인클로저 장착
- [ ] 레이저 개구부 정렬 확인
- [ ] 마운팅 하드웨어 고정
- [ ] 외부 배선 스트레인 릴리프 처리

---

**문서 끝**
