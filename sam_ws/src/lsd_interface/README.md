# LSD Interface 코드 분석

---

## 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [폴더 구조](#2-폴더-구조)
3. [전체 시스템 아키텍처](#3-전체-시스템-아키텍처)
4. [파일별 상세 분석](#4-파일별-상세-분석)
   - 4.1 [main.py — ROS2 브릿지 노드](#41-mainpy--ros2-브릿지-노드)
   - 4.2 [lds_controller.py — 시리얼 프로토콜 드라이버](#42-lds_controllerpy--시리얼-프로토콜-드라이버)
   - 4.3 [requirements.txt — Python 의존성](#43-requirementstxt--python-의존성)
   - 4.4 [static/index.html — 웹 UI 마크업](#44-staticindexhtml--웹-ui-마크업)
   - 4.5 [static/app.js — WebSocket 클라이언트](#45-staticappjs--websocket-클라이언트)
   - 4.6 [static/style.css — 웹 UI 스타일](#46-staticstylecss--웹-ui-스타일)
5. [통신 프로토콜 정리](#5-통신-프로토콜-정리)
6. [핵심 동작 시나리오](#6-핵심-동작-시나리오)
7. [에러 처리 및 자동 복구 메커니즘](#7-에러-처리-및-자동-복구-메커니즘)
8. [개발 시 주의사항](#8-개발-시-주의사항)

---

## 1. 프로젝트 개요

### 1.1 프로젝트 목적

**JRT U81 레이저 거리 센서**를 ROS2 환경에서 제어하기 위한 Python 인터페이스 패키지입니다.

- **하드웨어**: JRT U81 LDS 센서 + USB-Serial 어댑터(CH340 칩)
- **통신 프로토콜**: 바이너리 시리얼 프로토콜 (19200 bps)
- **네트워크 인터페이스**: 단일 ROS2 토픽 `perc/lds` (std_msgs/String, JSON 인코딩)
- **측정 범위**: 0.03 ~ 20m
- **언어/프레임워크**: Python 3.8+, ROS2 (Foxy/Humble), pyserial

### 1.2 핵심 특징

| 특징 | 내용 |
|------|------|
| **단일 토픽 통신** | `perc/lds` 하나의 토픽으로 명령/응답 양방향 처리 |
| **자동 재초기화** | 통신 타임아웃 시 PWREN 토글 + 자동 보레이트 재감지 후 명령 재시도 |
| **3가지 측정 모드** | Slow(원거리·저반사), Auto(균형), Fast(근거리·고반사) |
| **자동 폴백** | 신호 약함(0x0008) 시 Slow 모드로 자동 전환 |
| **공급 전압 모니터링** | 전압 < 2.8V 시 경고 (레이저 출력 저하 위험) |
| **연속 측정** | 별도 스레드에서 콜백 기반 실시간 측정 |

### 1.3 시스템 분리 설계

기존 통합형 구조에서 **로봇 PC ↔ 서버 PC** 2분리 구조로 변경됨:
- **로봇 PC**: 센서 제어만 담당 (`main.py` + `lds_controller.py`)
- **서버 PC**: 웹 UI 및 원격 제어 (`mainwindow.py` — 별도 프로젝트)
- **`static/` 폴더**: 기존 통합형 시절의 웹 UI (현재 미사용, 참고용)

---

## 2. 폴더 구조

```
lsd_interface/
├── main.py                  # ROS2 노드 (perc/lds 토픽 브릿지)
├── lds_controller.py        # JRT U81 시리얼 프로토콜 드라이버
├── requirements.txt         # Python 의존성
├── 실행가이드.md            # 사용자용 실행 가이드
├── 코드분석문서.md          # 본 문서
└── static/                  # (미사용 — 기존 웹 UI)
    ├── index.html           # 웹 UI HTML 구조
    ├── app.js               # WebSocket 클라이언트 로직
    └── style.css            # 다크 테마 스타일
```

### 2.1 파일별 역할 요약

| 파일 | 라인 수 | 역할 |
|------|--------|------|
| [main.py](main.py) | 226 | ROS2 노드 — 토픽 ↔ 시리얼 브릿지 |
| [lds_controller.py](lds_controller.py) | 745 | 시리얼 프로토콜 드라이버 (핵심 로직) |
| [requirements.txt](requirements.txt) | 4 | 의존 패키지 명시 |
| [static/index.html](static/index.html) | 111 | 웹 UI 레이아웃 (참고용) |
| [static/app.js](static/app.js) | 327 | WebSocket 통신 + UI 로직 (참고용) |
| [static/style.css](static/style.css) | 500 | GitHub Dark 테마 (참고용) |

---

## 3. 전체 시스템 아키텍처

### 3.1 컴포넌트 다이어그램

```
┌──────────────────────────────────────────────────────────────────────┐
│                            로봇 PC                                    │
│                                                                      │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                   main.py (LDSNode)                          │   │
│  │  ┌─────────────────────┐    ┌─────────────────────────┐    │   │
│  │  │  topic_callback()   │    │   publish_status()       │   │
│  │  │  (명령 수신)         │    │   (2초 주기 상태 발행)   │   │
│  │  └──────────┬──────────┘    └─────────────────────────┘    │   │
│  │             │                                                 │   │
│  │             ▼                                                 │   │
│  │  ┌─────────────────────┐                                     │   │
│  │  │     _dispatch()      │ ─── 별도 스레드에서 처리            │   │
│  │  │     (명령 라우팅)    │                                     │   │
│  │  └──────────┬──────────┘                                     │   │
│  └─────────────┼────────────────────────────────────────────────┘   │
│                │ Method calls                                        │
│                ▼                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │              lds_controller.py (LDSController)               │   │
│  │  ┌────────────┐ ┌────────────┐ ┌────────────────────────┐  │   │
│  │  │ connect()  │ │ power_on() │ │ start_continuous()     │  │   │
│  │  │ disconnect │ │ power_off()│ │ stop_continuous()      │  │   │
│  │  │            │ │ measure_   │ │ set_measure_mode()     │  │   │
│  │  │            │ │  once()    │ │ read_voltage()         │  │   │
│  │  └────────────┘ └────────────┘ └────────────────────────┘  │   │
│  │                                                              │   │
│  │  ┌────────────────── 내부 헬퍼 ───────────────────────┐    │   │
│  │  │  _do_startup()  / _reinitialize()                    │   │
│  │  │  _do_measure_once()  / _continuous_loop()            │   │
│  │  │  _read_measure_frame()  / _read_until_header()       │   │
│  │  │  _write_cmd_with_retry()  / _try_write_cmd()         │   │
│  │  │  _parse_measure_frame()  / _parse_error_code()       │   │
│  │  └──────────────────────────────────────────────────────┘   │
│  └─────────────┬────────────────────────────────────────────────┘   │
│                │ pyserial (binary)                                   │
│                ▼                                                     │
│         ┌──────────────┐                                             │
│         │ /dev/ttyLDS  │                                             │
│         │ (CH340 USB)  │                                             │
│         └──────┬───────┘                                             │
└────────────────┼─────────────────────────────────────────────────────┘
                 │ RS232 (RTS/DTR + TX/RX)
                 ▼
            ┌──────────┐
            │ JRT U81  │
            │   LDS    │
            └──────────┘
```

### 3.2 데이터 흐름

```
[서버 PC]                  [로봇 PC]
   │                          │
   │  perc/lds                │
   ├─────────{cmd}───────────►│  topic_callback()
   │                          │       │
   │                          │       ▼
   │                          │  _dispatch() (스레드)
   │                          │       │
   │                          │       ▼
   │                          │  LDSController.method()
   │                          │       │
   │                          │       ▼
   │                          │  시리얼 송수신
   │                          │       │
   │                          │       ▼
   │  perc/lds                │  결과 처리
   │◄────────{type}───────────┤  _publish()
   │                          │
   │                          │  매 2초마다 publish_status() (자동)
   │◄────────{status}─────────┤
   │                          │
```

---

## 4. 파일별 상세 분석

### 4.1 main.py — ROS2 브릿지 노드

#### 4.1.1 개요
- **역할**: ROS2 토픽 `perc/lds`와 `LDSController` 사이의 브릿지
- **핵심 클래스**: `LDSNode(Node)`
- **단일 토픽**: 명령(`cmd`)과 응답(`type`) 모두 같은 토픽에서 처리

#### 4.1.2 클래스 구조

##### `LDSNode(Node)` — ROS2 노드

**초기화** ([main.py:37-58](main.py:37))
- `LDSController` 인스턴스 생성 (`/dev/ttyLDS`, 19200 bps)
- `perc/lds` 토픽: Publisher + Subscriber 동시 등록
- 2초 주기 상태 발행 타이머 등록

**주요 메서드**

| 메서드 | 역할 | 라인 |
|--------|------|------|
| `topic_callback(msg)` | 토픽 메시지 수신 → `cmd` 필드 있으면 처리 | [62-82](main.py:62) |
| `_dispatch(data)` | 명령 → LDSController 메서드로 라우팅 | [86-182](main.py:86) |
| `_continuous_callback(result)` | 연속 측정 결과를 토픽으로 발행 | [186-188](main.py:186) |
| `publish_status()` | 현재 센서 상태 발행 (주기적/즉시) | [192-195](main.py:192) |
| `_publish(payload)` | dict → JSON → String 메시지 발행 | [199-203](main.py:199) |
| `destroy_node()` | 종료 시 시리얼 포트 해제 | [207-210](main.py:207) |

#### 4.1.3 명령 라우팅 테이블 (`_dispatch`)

| 명령 (cmd) | LDSController 메서드 | 응답 타입 (type) |
|-----------|---------------------|-----------------|
| `power_on` | `power_on()` | `ack` |
| `power_off` | `power_off()` | `ack` |
| `measure_once` | `measure_once()` | `distance` 또는 `error` |
| `continuous_on` | `start_continuous(_continuous_callback)` | `ack` |
| `continuous_off` | `stop_continuous()` | `ack` |
| `set_mode` | `set_measure_mode(mode)` | `ack` |
| `read_voltage` | `read_voltage()` | `voltage` |
| (그 외) | — | `error` |

> **중요**: 시리얼 통신은 블로킹이므로 `_dispatch`는 항상 별도 데몬 스레드에서 실행됩니다 ([main.py:80-82](main.py:80))

#### 4.1.4 메시지 자기 필터링

같은 토픽에서 명령/응답이 모두 흐르므로, **자기가 발행한 메시지는 무시**합니다.

```python
# 응답 메시지(type 필드 있음)는 무시 — 자기가 보낸 것
if 'type' in data:
    return
```
([main.py:69-71](main.py:69))

#### 4.1.5 즉시 상태 브로드캐스트

상태 변경 명령(`power_on`, `power_off`, `continuous_on/off`, `set_mode`) 후 즉시 status 발행 ([main.py:179-182](main.py:179)):

```python
if cmd in ('power_on', 'power_off', 'continuous_on', 'continuous_off', 'set_mode'):
    time.sleep(0.1)  # 상태 반영 대기
    self.publish_status()
```

---

### 4.2 lds_controller.py — 시리얼 프로토콜 드라이버

#### 4.2.1 개요
- **역할**: JRT U81 센서의 바이너리 시리얼 프로토콜 직접 구현
- **핵심 클래스**: `LDSController`
- **외부 라이브러리**: `pyserial`만 의존 (ROS와 무관 — 단독 사용 가능)

#### 4.2.2 하드웨어 연결 매핑

| Module 핀 | USB 어댑터 | 동작 |
|-----------|-----------|------|
| PWREN | RTS | HIGH = 모듈 ON (RTS=False 시 HIGH) |
| nRST | DTR | HIGH = 리셋 해제 (DTR=False 시 HIGH) |
| TXD | USB RXD | (교차 연결) |
| RXD | USB TXD | (교차 연결) |

> **주의**: pyserial의 RTS/DTR은 inverted logic — `setRTS(True)` 호출 시 핀은 LOW

#### 4.2.3 시작 시퀀스 (`_do_startup`)

JRT U81 PDF 6.2절 규격을 그대로 구현합니다.

```
1. RTS=True, DTR=True   → PWREN LOW   → 모듈 OFF
2. 150ms 대기 (PWREN_OFF_WAIT)
3. RTS=False, DTR=False → PWREN HIGH  → 모듈 ON
4. 500ms 부팅 대기 (BOOT_WAIT)
5. 0x55 전송 (자동 보레이트 감지 요청)
6. 모듈이 0x00 응답 (자신의 주소) → 통신 준비 완료
7. 초기 전압 읽기 (_update_voltage)
```

#### 4.2.4 바이너리 명령 프레임

##### 쓰기 명령 형식 (9바이트)

| 위치 | 0 | 1 | 2-3 | 4-5 | 6-7 | 8 |
|------|---|---|-----|-----|-----|---|
| 의미 | 0xAA | addr | reg | count | data | checksum |

##### 측정 응답 형식 (13바이트)

| 위치 | 0 | 1 | 2-3 | 4-5 | 6-9 | 10-11 | 12 |
|------|---|---|-----|-----|-----|-------|----|
| 의미 | 0xAA | addr | reg | count | distance(mm) | SQ | CS |

##### 명령 상수 정리 ([lds_controller.py:65-85](lds_controller.py:65))

```python
CMD_BAUD_DETECT  = 0x55                    # 자동 보레이트 감지
CMD_LASER_ON     = AA 00 01 BE 00 01 00 01 C1
CMD_LASER_OFF    = AA 00 01 BE 00 01 00 00 C0
CMD_MEASURE_ONCE_AUTO = AA 00 00 20 00 01 00 00 21
CMD_MEASURE_ONCE_SLOW = AA 00 00 20 00 01 00 01 22
CMD_MEASURE_ONCE_FAST = AA 00 00 20 00 01 00 02 23
CMD_MEASURE_CONT_AUTO = AA 00 00 20 00 01 00 04 25
CMD_MEASURE_CONT_SLOW = AA 00 00 20 00 01 00 05 26
CMD_MEASURE_CONT_FAST = AA 00 00 20 00 01 00 06 27
CMD_STOP_CONT    = 0x58 ('X')              # 연속 측정 중지
CMD_READ_STATUS  = AA 80 00 00 80
CMD_READ_VOLTAGE = AA 80 00 06 86          # REG_BAT_VLTG 읽기
```

#### 4.2.5 측정 모드 설정 ([lds_controller.py:76-80](lds_controller.py:76))

```python
MEASURE_MODES = {
    'slow': {'once': CMD_MEASURE_ONCE_SLOW, 'cont': CMD_MEASURE_CONT_SLOW, 'timeout': 8.0},
    'auto': {'once': CMD_MEASURE_ONCE_AUTO, 'cont': CMD_MEASURE_CONT_AUTO, 'timeout': 5.0},
    'fast': {'once': CMD_MEASURE_ONCE_FAST, 'cont': CMD_MEASURE_CONT_FAST, 'timeout': 3.0},
}
```

| 모드 | 타임아웃 | 권장 환경 |
|------|---------|----------|
| `slow` | 8초/회 | 원거리(5~20m), 저반사, 어두운 환경 |
| `auto` | 5초/회 | 일반 실내/실외 |
| `fast` | 3초/회 | 근거리(0~3m), 고반사, 빠른 응답 |

#### 4.2.6 타이밍 상수 ([lds_controller.py:91-101](lds_controller.py:91))

| 상수 | 값 | 용도 |
|------|---|------|
| `PWREN_OFF_WAIT` | 0.15s | PWREN LOW 유지 시간 |
| `BOOT_WAIT` | 0.50s | PWREN HIGH 후 부팅 대기 |
| `BAUD_DETECT_WAIT` | 0.40s | 0x55 전송 후 응답 대기 |
| `BAUD_DETECT_RETRY` | 3회 | 0x55 재시도 횟수 |
| `LASER_STABILIZE` | 0.40s | 레이저 ON 후 광학계 안정화 |
| `ACK_TIMEOUT` | 0.60s | 쓰기 명령 에코 대기 최대 |
| `MEASURE_TIMEOUT` | 5.0s | 기본 측정 타임아웃 |
| `SERIAL_BYTE_TIMEOUT` | 0.10s | `ser.read(1)` 블로킹 최대 |
| `CONT_MAX_FAIL` | 3회 | 연속 측정 실패 → 재초기화 |
| `WEAK_SIGNAL_MAX_RETRY` | 3회 | 0x0008 신호 약함 최대 재시도 |

#### 4.2.7 상태 코드 (`STATUS_CODES`) ([lds_controller.py:38-58](lds_controller.py:38))

| 코드 | 의미 |
|------|------|
| 0x0000 | 정상 |
| 0x0001 | 입력 전력 부족 (≥ 2.2V 필요) |
| 0x0002 | 내부 오류 (무시 가능) |
| 0x0003 | 모듈 온도 너무 낮음 (< -20℃) |
| 0x0004 | 모듈 온도 너무 높음 (> +40℃) |
| 0x0005 | 측정 대상 범위 초과 |
| 0x0006 | 잘못된 측정 결과 |
| 0x0007 | 배경 광도 너무 강함 |
| 0x0008 | **레이저 신호 너무 약함** ★ |
| 0x0009 | 레이저 신호 너무 강함 |
| 0x000A~0x0011 | 하드웨어 오류 |
| 0x000F | 레이저 신호 불안정 |
| 0x0081 | 잘못된 프레임 |

#### 4.2.8 공개 API 메서드

##### 연결 관리

| 메서드 | 설명 |
|--------|------|
| `connect() -> bool` | 포트 열기 + PWREN 제어 + 자동 보레이트 |
| `disconnect()` | 연속 중지 + PWREN OFF + 포트 닫기 |
| `is_connected() -> bool` | 포트 열림 + 초기화 완료 여부 |

##### 전원 제어

| 메서드 | 설명 |
|--------|------|
| `power_on() -> dict` | 레이저 ON + 안정화 대기 (400ms) |
| `power_off() -> dict` | 연속 중지 후 레이저 OFF |

##### 측정

| 메서드 | 설명 |
|--------|------|
| `measure_once() -> dict` | 1회 측정 (자동 폴백/재시도 포함) |
| `start_continuous(callback)` | 연속 측정 시작 (별도 스레드) |
| `stop_continuous()` | 연속 측정 중지 + 레이저 상태 복원 |
| `set_measure_mode(mode)` | 모드 변경 (slow/auto/fast) |

##### 진단

| 메서드 | 설명 |
|--------|------|
| `get_status() -> dict` | 현재 상태 조회 |
| `read_voltage() -> dict` | 공급 전압 읽기 (BCD 디코딩) |

#### 4.2.9 핵심 내부 메서드

##### `_do_measure_once(mode)` — 1회 측정 ([lds_controller.py:457-473](lds_controller.py:457))

```python
1. 모드별 타임아웃 결정
2. 입력 버퍼 비우기 (이전 잔여 데이터 제거)
3. 측정 명령 전송
4. _read_measure_frame()으로 응답 수신
5. 타임아웃 시 None 반환 (호출자가 재초기화 결정)
```

##### `_continuous_loop()` — 연속 측정 루프 ([lds_controller.py:475-528](lds_controller.py:475))

```python
1. 연속 측정 명령 1회 전송
2. while not self._stop_event.is_set():
3.   프레임 수신 시도 (모드별 타임아웃)
4.   타임아웃 시 fail_count++
5.   fail_count >= CONT_MAX_FAIL → _reinitialize() + 재시작
6.   성공 시 fail_count=0 + callback 호출
7. 종료 시 입력 버퍼 정리
```

##### `_read_measure_frame()` — 응답 프레임 수신 및 파싱 ([lds_controller.py:579-650](lds_controller.py:579))

3가지 프레임 형태를 처리합니다:

| 형태 | 길이 | 헤더 | 의미 |
|------|------|------|------|
| 정상 측정 | 13B | 0xAA | 거리 + SQ |
| 단형 오류 | 9B | 0xEE | 상태 코드만 |
| 장형 오류 | 13B | 0xEE + count=4 | 거리 + 상태 코드 (저신뢰도) |

**저신뢰도 결과** ([lds_controller.py:621-629](lds_controller.py:621)): 장형 오류 프레임에서 거리가 30~25000mm 범위라면 `low_confidence: True`로 반환합니다.

##### `_reinitialize()` — 자동 재초기화 ([lds_controller.py:369-412](lds_controller.py:369))

아이들/전원 순간 차단으로 센서가 리셋된 경우 자동 복구:

```
1. _initialized = False (상태 무효화)
2. 이전 레이저 상태 기억 (prev_powered)
3. 연속 중지 명령 전송 (CMD_STOP_CONT)
4. 입력 버퍼 비우기
5. _do_startup() 재실행 (PWREN 재토글 + 0x55)
6. prev_powered 였다면 LASER_ON 복원
```

##### `_write_cmd_with_retry()` — 자동 재시도 쓰기 ([lds_controller.py:532-548](lds_controller.py:532))

```
1. 1회 쓰기 시도
2. "응답 없음" 오류 시 → _reinitialize() → 1회 재시도
3. 그 외 오류는 그대로 반환
```

##### 전압 읽기 BCD 디코딩 ([lds_controller.py:319-321](lds_controller.py:319))

```python
# v_h, v_l: BCD 인코딩 (예: 0x33 0x19 → 3319mV)
mv = ((v_h >> 4) * 1000 + (v_h & 0x0F) * 100 +
      (v_l >> 4) * 10  + (v_l & 0x0F))
```

#### 4.2.10 자동 폴백 메커니즘

**`measure_once()` 내부 폴백 흐름** ([lds_controller.py:184-226](lds_controller.py:184)):

```
1. 1회 측정 시도
   ├ 성공 → 반환
   └ None (타임아웃) → _reinitialize() + 재측정

2. 0x0008 (신호 약함) 감지 시
   ├ Auto/Fast 모드 → Slow 모드로 1회 자동 재측정
   └ Slow 모드 → WEAK_SIGNAL_MAX_RETRY 회 재시도

3. 측정 완료 후 _restore_laser() 호출
   (센서가 측정 후 레이저를 끌 수 있음 → 명시적 복원)
```

#### 4.2.11 동시성 제어

`threading.Lock`으로 시리얼 접근 직렬화 ([lds_controller.py:114](lds_controller.py:114)):

```python
self._lock = threading.Lock()  # 모든 시리얼 read/write 보호

with self._lock:
    self.ser.reset_input_buffer()
    self.ser.write(cmd)
    resp = self._read_nbytes(...)
```

`threading.Event`로 연속 루프 중지 신호 전달:

```python
self._stop_event = threading.Event()
# 메인 스레드: self._stop_event.set()
# 연속 루프: while not self._stop_event.is_set(): ...
```

#### 4.2.12 에러 헬퍼

```python
def _err(message: str) -> dict:
    """에러 결과 딕셔너리 생성"""
    logger.warning(message)
    return {'success': False, 'error': message}
```
([lds_controller.py:742-745](lds_controller.py:742))

---

### 4.3 requirements.txt — Python 의존성

```
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
pyserial>=3.5
websockets>=12.0
```

| 패키지 | 용도 | 현재 사용 여부 |
|--------|------|---------------|
| `fastapi` | 웹서버 프레임워크 | **미사용** (기존 통합형 시절) |
| `uvicorn` | ASGI 서버 | **미사용** |
| `pyserial` | 시리얼 통신 | **사용 중** ([lds_controller.py:30](lds_controller.py:30)) |
| `websockets` | WebSocket 서버 | **미사용** |

> **참고**: 현재 ROS2 분리 구조에서는 `pyserial`만 실제로 필요합니다. ROS2 패키지(`rclpy`, `std_msgs`)는 ROS2 환경에 기본 포함됩니다.

---

### 4.4 static/index.html — 웹 UI 마크업

> **현재 상태**: 미사용 (참고용). 웹 UI는 서버 PC의 `mainwindow.py`로 이전됨.

#### 4.4.1 구조 요약

```html
<div class="container">
  <header>            <!-- 제목 + 연결 상태 + 포트/전압 뱃지 -->
  <section class="distance-card">  <!-- 거리 표시 (mm/m/SQ) -->
  <section class="control-card">    <!-- 전원 제어 (ON/OFF) -->
  <section class="control-card">    <!-- 측정 제어 (순간/연속) -->
  <section class="control-card">    <!-- 측정 모드 (Slow/Auto/Fast) -->
  <section class="log-card">        <!-- 측정 이력 -->
</div>
<div class="toast">                  <!-- 알림 -->
```

#### 4.4.2 주요 UI 요소

| 요소 ID | 역할 |
|---------|------|
| `connDot`, `connText` | WebSocket 연결 상태 표시 |
| `portBadge`, `voltBadge` | 시리얼 포트명, 공급 전압 |
| `distanceMain`, `distanceSub` | mm 단위 큰 숫자, m 단위 작은 숫자 |
| `lowConfBadge` | 저신뢰도 측정 시 표시 |
| `sqBar`, `sqValue`, `sqGrade` | 신호품질 시각화 |
| `statePower`, `stateCont` | 레이저 ON/OFF, 연속 ON/OFF 필 |
| `btnOn/Off`, `btnOnce`, `btnCont` | 제어 버튼 |
| `btnModeSlow/Auto/Fast` | 모드 전환 버튼 |
| `logList`, `logCount` | 측정 이력 리스트 |

#### 4.4.3 이벤트 핸들러 (인라인)

```html
onclick="sendCmd('power_on')"
onclick="sendCmd('power_off')"
onclick="sendCmd('measure_once')"
onclick="toggleContinuous()"
onclick="setMode('slow')"
onclick="sendCmd('read_voltage')"
onclick="clearLog()"
```

---

### 4.5 static/app.js — WebSocket 클라이언트

> **현재 상태**: 미사용 (참고용)

#### 4.5.1 모듈 구조

```javascript
// 전역 상태
let ws, isPowered, isContinuous, measureMode, logCount, ...

// WebSocket 초기화 (자동 재연결)
initWS()

// 메시지 라우팅
handleMessage(msg) → applyStatus / renderDistance / showToast / ...

// UI 업데이트
applyStatus(status), renderDistance(data), renderSQ(sq), appendLog(data),
updateVoltage(mv), setConnected(connected), showToast(message, type)

// 사용자 액션
sendCmd(cmd, extra), toggleContinuous(), setMode(mode), clearLog()
```

#### 4.5.2 WebSocket 연결 및 자동 재연결

```javascript
ws.onclose = () => {
  setConnected(false);
  ws = null;
  showToast('연결이 끊겼습니다. 재연결 중...', 'error');
  reconnTimer = setTimeout(initWS, 3000);
};
```
([app.js:42-47](static/app.js:42))

#### 4.5.3 메시지 타입별 처리

| `msg.type` | 처리 함수 |
|-----------|----------|
| `status` | `applyStatus(msg)` — 버튼 활성/비활성 + 모드 동기화 |
| `distance` | `renderDistance(msg)` + `appendLog(msg)` |
| `ack` | `showToast(msg.message, success/error)` |
| `voltage` | `updateVoltage(msg.voltage_mv)` + 토스트 |
| `error` | `showToast(msg.message, 'error')` |

#### 4.5.4 신호품질(SQ) 등급 ([app.js:199-209](static/app.js:199))

```javascript
if      (sq <  100) { grade = '우수'; color = 'good';   }  // 초록
else if (sq <  500) { grade = '양호'; color = 'normal'; }  // 파랑
else if (sq < 1000) { grade = '보통'; color = 'warn';   }  // 노랑
else                { grade = '불량'; color = 'bad';    }  // 빨강
```

#### 4.5.5 거리 표시 카드 상태

| 상태 | 색상 | 추가 표시 |
|------|------|----------|
| 정상 | `--accent` (파란색) | 없음 |
| 저신뢰도 | `--yellow` | "※ 저신뢰도" 뱃지 (호흡 애니메이션) |
| 오류 | `--red` | "ERR" + 에러 메시지 |

#### 4.5.6 버튼 활성/비활성 로직 ([app.js:124-133](static/app.js:124))

```javascript
el('btnOn').disabled   = isPowered;                       // ON 시 비활성
el('btnOff').disabled  = !isPowered;                      // OFF 시 비활성
el('btnOnce').disabled = !isPowered || isContinuous;      // 레이저 OFF 또는 연속 중
el('btnCont').disabled = !isPowered;                      // 레이저 OFF 시
```

---

### 4.6 static/style.css — 웹 UI 스타일

> **현재 상태**: 미사용 (참고용)

#### 4.6.1 디자인 시스템

GitHub Dark 테마 기반 색상 팔레트:

| CSS 변수 | 값 | 용도 |
|---------|---|------|
| `--bg` | `#0d1117` | 배경 |
| `--surface` | `#161b22` | 카드 배경 |
| `--surface2` | `#1c2128` | 카드 배경(보조) |
| `--border` | `#30363d` | 테두리 |
| `--text` | `#e6edf3` | 본문 |
| `--text-muted` | `#7d8590` | 회색 텍스트 |
| `--accent` | `#58a6ff` | 파란색(거리 표시) |
| `--green` | `#3fb950` | 초록(정상/ON) |
| `--red` | `#f85149` | 빨강(오류/OFF) |
| `--purple` | `#bc8cff` | 보라(연속 측정) |
| `--yellow` | `#d29922` | 노랑(경고/저신뢰도) |

#### 4.6.2 레이아웃 특징

- **컨테이너 최대 폭**: 560px (모바일 친화)
- **모노스페이스 폰트**: SF Mono / Consolas (거리 표시, 로그)
- **그라데이션 카드**: `distance-card` 배경에 방사형 그라데이션
- **반응형**: 400px 이하에서 폰트/패딩 축소
- **호흡 애니메이션**: `.low-conf-badge`, `.state-pill.cont`
- **페이드인 애니메이션**: `.log-entry`

#### 4.6.3 버튼 색상 매핑

| 클래스 | 색상 | 용도 |
|--------|------|------|
| `btn-green` | 초록 | 레이저 ON |
| `btn-red` | 빨강 | 레이저 OFF |
| `btn-blue` | 파랑 | 순간 측정 |
| `btn-purple` | 보라 | 연속 시작/중지 |
| `btn-mode` | 회색 | 모드 선택(active 시 파랑) |

#### 4.6.4 토스트 시스템

```css
.toast { position: fixed; bottom: 28px; right: 20px; ... }
.toast.show              { opacity: 1; transform: translateY(0); }
.toast.success           { border-color: var(--green); }
.toast.error             { border-color: var(--red); }
.toast.info              { border-color: var(--accent); }
```

---

## 5. 통신 프로토콜 정리

### 5.1 ROS2 토픽 — `perc/lds`

**메시지 타입**: `std_msgs/String` (JSON 인코딩)

#### 5.1.1 명령 메시지 (서버 → 로봇)

| 명령 | JSON | 응답 |
|------|------|------|
| 레이저 ON | `{"cmd": "power_on"}` | `ack` |
| 레이저 OFF | `{"cmd": "power_off"}` | `ack` |
| 순간 측정 | `{"cmd": "measure_once"}` | `distance` 또는 `error` |
| 연속 시작 | `{"cmd": "continuous_on"}` | `ack` + (지속) `distance` 스트림 |
| 연속 중지 | `{"cmd": "continuous_off"}` | `ack` |
| 모드 변경 | `{"cmd": "set_mode", "mode": "slow"}` | `ack` |
| 전압 읽기 | `{"cmd": "read_voltage"}` | `voltage` |

#### 5.1.2 응답 메시지 (로봇 → 서버)

| 타입 | 발행 시점 | 페이로드 |
|------|----------|---------|
| `status` | 2초 주기 + 상태 변경 직후 | `connected`, `port`, `powered`, `continuous`, `initialized`, `measure_mode`, `voltage_mv` |
| `ack` | 명령 처리 완료 시 | `cmd`, `success`, `message` |
| `distance` | 측정 결과 | `success`, `mm`, `m`, `signal_quality`, `timestamp`, (옵션)`low_confidence` |
| `voltage` | 전압 읽기 응답 | `success`, `voltage_v`, `voltage_mv`, `message` |
| `error` | 오류 발생 시 | `message` |

#### 5.1.3 메시지 구분 규칙

```
cmd 필드만 존재 → 명령 (로봇 노드가 처리)
type 필드만 존재 → 응답 (서버 인터페이스가 처리)
각 측은 자기가 보낸 메시지를 무시 (type 필드 체크로 필터)
```

### 5.2 시리얼 프로토콜 — JRT U81

**시리얼 설정**: 19200 bps, 8N1, no flow control

#### 5.2.1 핵심 명령 흐름

```
[연결]
  PWREN OFF → 150ms → PWREN ON → 500ms 부팅 →
  0x55 전송 → 0x00 응답 수신 → 초기화 완료

[레이저 ON]
  AA 00 01 BE 00 01 00 01 C1 → 에코 응답 → 400ms 안정화

[측정 (Slow)]
  AA 00 00 20 00 01 00 01 22 →
  AA 00 00 20 00 01 [D3 D2 D1 D0] [SQH SQL] CS

[연속 측정 시작]
  AA 00 00 20 00 01 00 05 26 →
  (지속) 13B 측정 프레임 스트림

[연속 중지]
  0x58 ('X') 전송 → 스트림 종료
```

---

## 6. 핵심 동작 시나리오

### 6.1 정상 1회 측정 시나리오

```
1. main.py: topic_callback 수신 {"cmd":"measure_once"}
2. main.py: _dispatch 스레드 시작
3. main.py: lds.measure_once() 호출
4. lds_controller: _do_measure_once('slow')
5. lds_controller: 시리얼에 명령 전송 → 13B 응답 수신
6. lds_controller: _parse_measure_frame → {'success':True, 'mm':1519, 'm':1.519, 'signal_quality':315, ...}
7. lds_controller: _restore_laser() (레이저 상태 복원)
8. main.py: _publish({'type':'distance', ...})
9. 토픽으로 결과 발행
10. 0.1초 대기 후 status 발행
```

### 6.2 신호 약함 자동 폴백 시나리오

```
1. measure_once() 호출 (Auto 모드)
2. _do_measure_once('auto') → 0x0008 오류 응답
3. "신호 너무 약함" 감지 → Slow 모드로 자동 재측정
4. _do_measure_once('slow') → 성공 또는 추가 재시도 (최대 3회)
5. 결과 반환 + _restore_laser()
```

### 6.3 통신 끊김 자동 복구 시나리오

```
1. 임의의 명령 전송 → 응답 없음 (타임아웃)
2. _write_cmd_with_retry() 또는 _do_measure_once → None 반환
3. _reinitialize() 호출
   - is_powered 기억 (prev_powered)
   - CMD_STOP_CONT 송신 (잔류 연속 모드 정리)
   - _flush_input()
   - _do_startup() (PWREN 재토글 + 0x55)
   - prev_powered=True 였다면 LASER_ON 재전송
4. 명령 재시도
5. 성공/실패 결과 반환
```

### 6.4 연속 측정 중 센서 리셋 시나리오

```
1. start_continuous(callback) → _continuous_thread 시작
2. _continuous_loop: 연속 명령 전송 + 측정 프레임 스트림 수신
3. (센서 리셋 발생 — 응답 끊김)
4. fail_count++ (3회까지 누적)
5. fail_count >= CONT_MAX_FAIL → _reinitialize()
6. 재초기화 성공 + 레이저 ON 복원 → 연속 명령 재전송
7. fail_count=0 리셋 후 정상 동작 재개
```

---

## 7. 에러 처리 및 자동 복구 메커니즘

### 7.1 다층 방어 구조

```
┌──────────────────────────────────────────────────┐
│  Layer 1: 단일 명령 재시도 (_write_cmd_with_retry) │
│   → 응답 없음 시 재초기화 후 1회 재시도            │
├──────────────────────────────────────────────────┤
│  Layer 2: 측정 재시도 (measure_once)              │
│   → 타임아웃 시 재초기화 후 재측정                 │
│   → 0x0008 신호 약함 시 Slow 모드 폴백             │
│   → Slow 모드에서도 약함 시 최대 3회 재시도        │
├──────────────────────────────────────────────────┤
│  Layer 3: 연속 측정 자동 복구 (_continuous_loop)  │
│   → 연속 3회 타임아웃 시 재초기화 + 재시작         │
├──────────────────────────────────────────────────┤
│  Layer 4: 자동 재초기화 (_reinitialize)           │
│   → PWREN 재토글 → 0x55 재전송 → 레이저 상태 복원  │
└──────────────────────────────────────────────────┘
```

### 7.2 상태 추적 변수

| 변수 | 용도 |
|------|------|
| `self._initialized` | 0x55 자동 보레이트 감지 완료 여부 |
| `self.is_powered` | 레이저 ON 상태 (재초기화 시 복원 기준) |
| `self.is_continuous` | 연속 측정 진행 중 여부 |
| `self.measure_mode` | 현재 측정 모드 (slow/auto/fast) |
| `self.voltage_mv` | 마지막 전압 측정값 (mV) |

### 7.3 진단 정보

`get_status()`가 반환하는 진단 정보:

```python
{
    'connected':    True/False,    # is_connected()
    'port':         '/dev/ttyLDS',
    'powered':      True/False,    # is_powered
    'continuous':   True/False,    # is_continuous
    'initialized':  True/False,    # _initialized
    'measure_mode': 'slow'/'auto'/'fast',
    'voltage_mv':   3300,          # 또는 None
}
```

---

## 8. 개발 시 주의사항

### 8.1 USB-Serial 권한 (Linux)

```bash
# 사용자를 dialout 그룹에 추가
sudo usermod -aG dialout $USER

# Docker에서 실행 시
docker run --device /dev/ttyUSB0 --privileged ...
```

### 8.2 udev 규칙 (포트 고정)

매번 USB 재연결 시 `/dev/ttyUSB0`, `/dev/ttyUSB1` 변동을 막으려면:

```
# /etc/udev/rules.d/99-lds.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyLDS"
```

이후 `/dev/ttyLDS`로 고정 접근 가능 (현재 코드의 기본값).

### 8.3 RTS/DTR 동작 차이

- **pyserial**: `setRTS(True)` → 핀은 LOW (inverted)
- **U81 모듈**: PWREN HIGH = 모듈 ON

→ 코드는 `setRTS(False)` + `setDTR(False)` 시 모듈을 켭니다 ([lds_controller.py:342-343](lds_controller.py:342)).

### 8.4 측정 모드 변경 제약

```python
# 연속 측정 중에는 모드 변경 불가
def set_measure_mode(self, mode: str) -> dict:
    if self.is_continuous:
        return _err('연속 측정 중에는 모드를 변경할 수 없습니다.')
```

### 8.5 시리얼 동시 접근

모든 시리얼 read/write는 반드시 `self._lock` 보호하에 수행해야 합니다:

```python
with self._lock:
    self.ser.reset_input_buffer()
    self.ser.write(cmd)
    resp = self._read_nbytes(...)
```

### 8.6 전압 모니터링 권장

```
공급 전압 < 2.8V → 0x0008 신호 약함 오류 빈발 가능
권장 조치:
- USB 케이블/포트 교체
- 외부 3.3V 전원 공급
- USB 허브 사용 시 셀프파워 허브 권장
```

### 8.7 체크섬 검증

현재 구현은 측정 프레임 체크섬 불일치 시 **로그만 남기고 데이터는 사용**합니다 ([lds_controller.py:725-728](lds_controller.py:725)):

```python
expected_cs = sum(frame[1:12]) & 0xFF
if frame[12] != expected_cs:
    logger.debug("체크섬 불일치: 수신 0x%02X, 계산 0x%02X (데이터는 사용)", ...)
```

엄격한 검증이 필요하면 이 부분을 수정하여 `None` 반환하도록 변경 가능합니다.

### 8.8 ROS2 토픽 발행 빈도

- `publish_status`: 2초 주기 (`status_timer`)
- 연속 측정: 모드별 1~3 Hz 정도 예상
- 명령 응답: 즉시

DDS 큐 크기는 10으로 설정 ([main.py:51-53](main.py:51)). 고빈도 측정 시 큐 오버플로우 주의.

### 8.9 ROS2 패키지 빌드

`main.py`는 단순 Python 스크립트로 실행 가능하지만, 정식 ROS2 패키지로 만들려면 다음이 필요합니다:

```
lsd_interface/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/lsd_interface
└── lsd_interface/
    ├── __init__.py
    ├── main.py
    └── lds_controller.py
```

현재 폴더 구조에는 `package.xml`이 없으므로 **단독 스크립트 모드**로 운용됩니다.

### 8.10 static/ 폴더의 향후 처리

현재 `static/` 폴더는 **미사용**입니다. 향후 옵션:

1. **삭제**: 분리 구조 확정 시 깔끔하게 정리
2. **유지**: 디버깅용 단독 웹 UI로 보존 (FastAPI 추가 시 활용 가능)
3. **분리**: `legacy/` 또는 `tools/` 하위로 이동

`requirements.txt`의 `fastapi`/`uvicorn`/`websockets`도 미사용이므로 정리 고려 대상입니다.

---

## 부록: 빠른 참조 카드

### A. 명령 → 토픽 매핑

```bash
# 레이저 ON
ros2 topic pub --once /perc/lds std_msgs/msg/String '{data: "{\"cmd\":\"power_on\"}"}'

# 1회 측정
ros2 topic pub --once /perc/lds std_msgs/msg/String '{data: "{\"cmd\":\"measure_once\"}"}'

# 연속 측정 시작 (5초간)
ros2 topic pub --once /perc/lds std_msgs/msg/String '{data: "{\"cmd\":\"continuous_on\"}"}'
sleep 5
ros2 topic pub --once /perc/lds std_msgs/msg/String '{data: "{\"cmd\":\"continuous_off\"}"}'

# 모드 변경
ros2 topic pub --once /perc/lds std_msgs/msg/String '{data: "{\"cmd\":\"set_mode\",\"mode\":\"fast\"}"}'

# 응답 모니터링
ros2 topic echo /perc/lds
```

### B. 핵심 클래스/메서드 위치 색인

| 항목 | 파일:라인 |
|------|----------|
| `LDSNode.__init__` | [main.py:37](main.py:37) |
| `LDSNode.topic_callback` | [main.py:62](main.py:62) |
| `LDSNode._dispatch` | [main.py:86](main.py:86) |
| `LDSController.__init__` | [lds_controller.py:103](lds_controller.py:103) |
| `LDSController.connect` | [lds_controller.py:121](lds_controller.py:121) |
| `LDSController.measure_once` | [lds_controller.py:184](lds_controller.py:184) |
| `LDSController.start_continuous` | [lds_controller.py:228](lds_controller.py:228) |
| `LDSController._do_startup` | [lds_controller.py:330](lds_controller.py:330) |
| `LDSController._reinitialize` | [lds_controller.py:369](lds_controller.py:369) |
| `LDSController._continuous_loop` | [lds_controller.py:475](lds_controller.py:475) |
| `LDSController._read_measure_frame` | [lds_controller.py:579](lds_controller.py:579) |
| `LDSController._parse_measure_frame` | [lds_controller.py:708](lds_controller.py:708) |

### C. 의존성 트리

```
main.py
├── rclpy (ROS2)
├── std_msgs.msg.String (ROS2)
└── lds_controller.LDSController
    └── serial (pyserial)
```

### D. 측정 모드 선택 가이드

```
물체 거리 + 환경                 → 권장 모드
─────────────────────────────────────────────
0~3m, 밝은 환경, 빠른 응답 필요  → fast
3~5m, 일반 실내                  → auto
5~20m 또는 저반사/어두움          → slow (기본값)
신호 약함 오류 발생               → slow + 재시도 자동
```

---

**문서 작성**: Claude Code (Opus 4.7)
**기반 자료**: 폴더 내 모든 소스 코드 + 실행가이드.md
