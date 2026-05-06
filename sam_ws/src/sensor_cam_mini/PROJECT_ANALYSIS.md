# MiniPC ROS2 Workspace

---

## 목차

1. [전체 아키텍처 개요](#1-전체-아키텍처-개요)
2. [패키지 구성](#2-패키지-구성)
3. [메인 시스템 ① `sensor_cam_mini`](#3-메인-시스템--sensor_cam_mini)
   - 3.1 [패키지 구조](#31-패키지-구조)
   - 3.2 [`camera_node.cpp` — FLIR GigE 카메라 노드](#32-camera_nodecpp--flir-gige-카메라-노드)
   - 3.3 [`lidar_node.cpp` — Livox 라이다 PCD 수집 노드](#33-lidar_nodecpp--livox-라이다-pcd-수집-노드)
   - 3.4 [`MultiActuator.py` — 카메라 커버 액추에이터 제어](#34-multiactuatorpy--카메라-커버-액추에이터-제어)
   - 3.5 [`PythonLibMightyZap_PC.py` — MightyZap 시리얼 라이브러리](#35-pythonlibmightyzap_pcpy--mightyzap-시리얼-라이브러리)
   - 3.6 [`flaskapp.py` — (구버전, 비활성)](#36-flaskapppy--구버전-비활성)
   - 3.7 [Launch 파일](#37-launch-파일)
4. [메인 시스템 ② `lsd_interface`](#4-메인-시스템--lsd_interface)
   - 4.1 [패키지 구조](#41-패키지-구조)
   - 4.2 [`main.py` — LDS ROS2 브릿지 노드](#42-mainpy--lds-ros2-브릿지-노드)
   - 4.3 [`lds_controller.py` — JRT U81 시리얼 드라이버](#43-lds_controllerpy--jrt-u81-시리얼-드라이버)
   - 4.4 [`static/` — (구버전 웹 UI, 참고용)](#44-static--구버전-웹-ui-참고용)
5. [지원 패키지](#5-지원-패키지)
   - 5.1 [`flir_camera_driver`](#51-flir_camera_driver)
   - 5.2 [`livox_ros2_driver`](#52-livox_ros2_driver)
   - 5.3 [`vision_opencv`](#53-vision_opencv)
6. [ROS2 토픽/서비스 맵](#6-ros2-토픽서비스-맵)
7. [데이터 플로우](#7-데이터-플로우)
8. [빌드 및 실행 가이드](#8-빌드-및-실행-가이드)
9. [주요 설계 포인트와 트러블슈팅 노트](#9-주요-설계-포인트와-트러블슈팅-노트)

---

## 1. 전체 아키텍처 개요

이 워크스페이스는 **로봇 검사 시스템(Robot inspection system)** 의 *miniPC* 측 ROS2 코드입니다.
하나의 카메라 모듈에 **GigE 산업용 카메라 + Livox 라이다 + 레이저 거리계(LDS) + 카메라 커버 액추에이터** 가 통합되어 있고, 각 센서를 ROS2 노드로 추상화해 상위 서버 PC가 토픽으로 원격 제어합니다.

```
┌────────────────────────────────────────────────────────────────────────┐
│                    서버 PC (mainwindow.py · 별도 저장소)                │
│   - Flask/PyQt 웹 UI                                                    │
│   - 영상 스트림 수신 (HTTP MJPEG)                                       │
│   - perc/lds, perc/cover, /camera_control 토픽 발행                     │
│   - /lidar_collect 서비스 호출 → PCD 파일 수집                          │
└──────────────────────────────────┬─────────────────────────────────────┘
                                   │   ROS2 DDS  (FastRTPS)
                                   │   + HTTP   (영상 스트림 :9100)
                                   │   + SCP    (PCD 파일 회수)
┌──────────────────────────────────┴─────────────────────────────────────┐
│                miniPC (Docker / Ubuntu) — 본 워크스페이스                │
│                                                                          │
│  ┌────────────────────┐    ┌──────────────────┐    ┌──────────────────┐ │
│  │ sensor_cam_mini    │    │ lsd_interface    │    │ livox_ros2_driver│ │
│  │  ├ camera_node     │    │  └ main.py       │    │  └ lidar 노드     │ │
│  │  ├ lidar_node      │    │      (ROS2 브릿지)│    │   /livox/lidar_1  │ │
│  │  └ MultiActuator   │    │                   │    └──────────────────┘ │
│  └────────┬───────────┘    └────────┬─────────┘                         │
│           │                          │                                   │
│  ┌────────┼──────────────────────────┼──────────────────────────────┐  │
│  │  USB ↔ /dev/ttyACTUATOR (MightyZap, 6채 액추에이터)              │  │
│  │  USB ↔ /dev/ttyLDS      (CH340 + JRT U81 레이저 거리계)          │  │
│  │  GigE ↔ FLIR Blackfly S (S/N 18566179, 192.168.2.1)             │  │
│  │  Eth2 ↔ Livox LiDAR     (Mid-70 등)                               │  │
│  └─────────────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────────────┘
```

**핵심 설계 원칙**

| 원칙 | 적용 부분 |
|------|-----------|
| 단일 토픽 양방향 통신 | `lsd_interface` — `perc/lds` (JSON over std_msgs/String) |
| 실패 시 자동 재초기화 | `camera_node` watchdog, `lds_controller` `_reinitialize()` |
| 비동기 실행 + 잠금 보호 | `MultiActuator` (asyncio), `camera_node` (mutex+cv) |
| PCL 비의존 PCD 저장 | `lidar_node.cpp` — 표준 C++만으로 binary PCD 출력 |
| Docker 볼륨 경유 파일 회수 | `/root/sam_ws → /home/sam-cam-s1/sam_ws` SCP 접근 |

---

## 2. 패키지 구성

```
src/
├── sensor_cam_mini/          ★ 메인 1 — 카메라/라이다/액추에이터 통합
├── lsd_interface/            ★ 메인 2 — 레이저 거리계 ROS2 브릿지
│
├── flir_camera_driver/       — (참고) 공식 FLIR Spinnaker 드라이버
├── livox_ros2_driver/        — Livox LiDAR 공식 드라이버
└── vision_opencv/            — cv_bridge / image_geometry
```

| 패키지 | 빌드 타입 | 언어 | 주요 노드/모듈 |
|--------|-----------|------|--------------|
| **sensor_cam_mini** | `ament_cmake` (+ Python install) | C++17 / Python3 | `camera_node`, `lidar_node`, `MultiActuator.py` |
| **lsd_interface** | (순수 Python, colcon 미사용) | Python3 | `main.py` (ROS2 노드) |
| flir_camera_driver/spinnaker_camera_driver | `ament_cmake` | C++ | `camera_driver_node` (참고용, miniPC에서는 직접 미실행) |
| livox_ros2_driver | `ament_cmake_auto` | C++ | `livox_ros2_driver_node` |
| vision_opencv/cv_bridge | `ament_cmake_ros` | C++ + Python | 의존성 라이브러리 |

> **sensor_cam_mini 의 카메라 노드는 자체 Spinnaker SDK 직접 호출**입니다.
> `flir_camera_driver` 는 ROS2 표준 드라이버이지만 본 시스템에서는 **사용하지 않고 참고만 합니다.**
> 이유: 3600x3600 해상도, 1fps, MJPEG TCP 직접 스트리밍, ForceIP 자동화 등 커스텀 기능 필요.

---

## 3. 메인 시스템 ① `sensor_cam_mini`

### 3.1 패키지 구조

```
sensor_cam_mini/
├── CMakeLists.txt              ← C++17, Spinnaker, GStreamer, OpenCV-CUDA, Livox 의존
├── package.xml                 ← ROS2 패키지 메타 (Apache-2.0)
│
├── src/                        ← C++ 노드 (ROS2 executable)
│   ├── camera_node.cpp         ★ FLIR GigE 카메라 + MJPEG HTTP 서버
│   └── lidar_node.cpp          ★ Livox PointCloud2 → 로컬 PCD 파일 저장
│
├── sensor_cam_mini/            ← Python 모듈 (ament_python_install)
│   ├── __init__.py             ← 패키지 상수 (YOLO 경로 등)
│   ├── MultiActuator.py        ★ 카메라 커버 액추에이터 제어 (MightyZap)
│   ├── PythonLibMightyZap_PC.py← MightyZap 시리얼 프로토콜 라이브러리
│   └── flaskapp.py             ← (전체 주석 처리됨, 폐기됨)
│
└── launch/                     ← ROS2 런치 파일
    ├── sensor_cam_mini.launch.py  ★ camera_node + lidar_node 동시 실행
    ├── act.launch.py              ★ MultiActuator.py 단독 실행
    └── flaskapp.launch.py         ← (비어있음, 폐기됨)
```

### CMake 의존성 핵심

```cmake
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui cudaimgproc cudaarithm)
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.14)
include_directories(/opt/spinnaker/include)
link_directories(/opt/spinnaker/lib)
```

* **OpenCV CUDA 모듈** 활성 → GPU 가속 이미지 처리 가능 (현재 코드는 CPU 인코딩만 사용)
* **GStreamer** ≥1.14 → 추후 RTSP/하드웨어 인코더 확장 여지
* **Spinnaker SDK** → FLIR 공식 SDK 직접 링크
* **executables**: `camera_node`, `lidar_node`
* **Python install**: `flaskapp.py`, `MultiActuator.py`, `PythonLibMightyZap_PC.py`

### 3.2 `camera_node.cpp` — FLIR GigE 카메라 노드

#### 역할
FLIR **Blackfly S BFS** GigE Vision 카메라를 직접 제어해 **3600×3600 Mono8 1fps** 영상을 획득하고, **HTTP MJPEG 서버(:9100)** 로 실시간 스트리밍하며 ROS2 토픽으로 원격 명령(노출/게인)을 받습니다.

#### 클래스: `CameraTcpStreamerNode` (rclcpp::Node)

##### 생성자 흐름 (l. 38–117)
```
1. 노드명: "camera_node_<카메라_이름>" (기본: "camera_node_Robot_Local")
2. /camera_status/<카메라_이름> 토픽 (1Hz 헬스 발행)
3. Spinnaker System::GetInstance() → CameraList 획득
4. 시리얼 번호 "18566179" 매칭 카메라 선택 (없으면 index 0)
5. configureNetworkForGigE()    — Linux 소켓/MTU 튜닝
6. autoForceIP()                — 카메라 IP 자동 보정 (192.168.2.1로)
7. configurePersistentIP()      — 영구 IP 저장
8. setupCameraAndStart()        — 해상도/노출/스트림 설정 + BeginAcquisition
9. server_thread_ 시작           — HTTP MJPEG 리스닝 :9100
10. /camera_control 구독         — 외부 명령 처리
11. 1초 주기 timer_              — grabAndStoreFrame()
12. 30초 주기 watchdog_timer_    — 프레임 정체 감시 → 자동 재초기화
```

##### 주요 메서드

| 메서드 | 역할 | 주의/특이사항 |
|--------|------|--------------|
| `configureNetworkForGigE()` | `eth2` MTU 1500 강제, `rmem_max=25MB` 등 sysctl 튜닝 | USB-Ethernet (RTL8153) 가정 — 점보 프레임 미지원이므로 큰 소켓 버퍼로 보상 |
| `autoForceIP()` | 카메라가 다른 서브넷이면 ForceIP 명령으로 192.168.2.1/24 강제 | 5회 재시도 (2초 간격), 동일 서브넷이면 스킵 |
| `configurePersistentIP()` | 카메라에 영구 IP 192.168.2.1/24 저장 | Init → 노드맵 → DeInit 순서 필수 |
| `setupCameraAndStart()` | 카메라 파라미터 설정 + BeginAcquisition | 핵심 GenICam 설정 ⤵ |
| `cameraControlCallback()` | `/camera_control` 토픽으로 게인/노출 변경 | 명령 형식: `"<카메라이름>:set_gain:5.0"` 등 |
| `grabAndStoreFrame()` | 1초마다 GetNextImage → JPEG 인코딩 → 공유 버퍼 저장 | 10초 타임아웃, 연속 실패 10회 → 재초기화 |
| `watchdogCheck()` | 30초 주기. 60초간 프레임 없으면 재초기화 | `last_good_frame_time_` 갱신 |
| `spinnakerFullReinit()` | EndAcquisition → DeInit → ReleaseInstance → 처음부터 재시작 | autoForceIP / configurePersistentIP 재실행 포함 |
| `startHttpServer()` | TCP 9100 LISTEN → 클라이언트 별 thread detach | `multipart/x-mixed-replace` 헤더 |
| `handleClientConnection()` | 새 프레임 도착 시 condition_variable 으로 wake → MJPEG 전송 | 최대 5초 대기 후 keepalive |

##### 핵심 GenICam 설정 (l. 461–571)

```cpp
StreamBufferHandlingMode = "NewestOnly"     // 오래된 프레임 폐기
StreamDefaultBufferCount = 10               // USB NIC 지연 변동 대비
StreamPacketResendEnable = true             // 패킷 재전송 (Image incomplete 방지)
StreamPacketResendMaxRequests = 1000
StreamPacketResendTimeout = 2000 ms

GevSCPSPacketSize  = 1400                   // MTU 1500 - 헤더 여유
GevSCPD            = 25000 ns               // 인터패킷 딜레이
GevHeartbeatTimeout = 10000 ms              // 기본 3초 → 10초

AcquisitionMode = "Continuous"
TriggerMode     = "Off"
ExposureAuto    = "Continuous"
Resolution      = 3600 × 3600 (Mono8)
Offset          = (936, 24)                 // 5472×3648 센서 중앙 크롭
AcquisitionFrameRate = 1.0 fps
```

> **주의**: `OffsetX/Y` 는 `Width/Height` 변경 *이전* 에 0으로 리셋해야 하고,
> 이후 다시 중앙 오프셋을 설정합니다. (l. 539–551)

##### 동시성 모델

```cpp
std::mutex frame_mutex_;
std::condition_variable frame_cv_;
std::shared_ptr<std::vector<uchar>> latest_jpeg_buffer_;
uint64_t frame_seq_;   // 각 클라이언트가 마지막 송신 시퀀스 추적
```

* `grabAndStoreFrame()` (single-thread, 1Hz) → `frame_mutex_` 잡고 `latest_jpeg_buffer_` 갱신
* `handleClientConnection()` (per-client thread) → `frame_cv_.wait_for(... [seq > last_sent_seq])` 로 Spurious wakeup 방지
* `shared_ptr<vector<uchar>>` 로 클라이언트가 송신 중인 동안에도 다음 프레임을 안전하게 갱신 가능

##### 스트리밍 예
```
http://<miniPC_IP>:9100        ← 브라우저로 직접 접속하면 MJPEG 재생
```

##### main()
```cpp
camera_name = "Robot_Local"
server_port = 9100
```
*하드코딩됨* — 필요 시 다중 카메라 지원을 위해 argv 파싱이나 ROS2 파라미터로 변경하는 것이 일반적이지만, 현재는 단일 카메라 시스템이라 단순화.

### 3.3 `lidar_node.cpp` — Livox 라이다 PCD 수집 노드

#### 역할
`livox_ros2_driver` 가 발행하는 `/livox/lidar_1` (PointCloud2) 를 구독하다가, 외부 서비스(`/lidar_collect`) 호출로 **수집 시작/중지**를 제어. 중지 시 누적 포인트를 **Binary PCD 파일** 로 저장하고, 파일 경로를 응답으로 반환.

#### 클래스: `LidarNode` (rclcpp::Node)

##### 멤버
```cpp
std::atomic<bool>           collecting_;          // 수집 중 플래그
std::atomic<size_t>         point_count_;         // 누적 카운터
std::vector<std::array<float,3>> accumulated_points_;  // x,y,z (NaN 제외)
std::mutex                  points_mutex_;
std::string                 storage_dir_ = "/root/sam_ws/storage/lidar_img1";
```

##### 동작 시나리오

```
[서버] /lidar_collect (data=true) 호출
   ↓
collectServiceCallback() — accumulated_points_ 초기화 + reserve(10M)
   ↓
[Livox] /livox/lidar_1 메시지 도착마다 lidarCallback() 가 NaN 제거 후 누적
   ↓
[서버] /lidar_collect (data=false) 호출
   ↓
save_pcd_file()                — Binary PCD 저장 (12B/point)
cleanup_old_pcd_files()        — 300초 이상 PCD 자동 삭제
   ↓
응답: "STOPPED:<파일경로>:<포인트수>"
```

##### Binary PCD 직접 작성 (PCL 비의존)

`lidar_node.cpp:142–163`
```
헤더(텍스트):
  # .PCD v0.7 - Point Cloud Data file format
  FIELDS x y z
  SIZE 4 4 4
  TYPE F F F
  COUNT 1 1 1
  WIDTH <num_points>
  HEIGHT 1
  VIEWPOINT 0 0 0 1 0 0 0
  POINTS <num_points>
  DATA binary

본문(이진): num_points × 12바이트 (float32 × 3)
```

* `std::vector<std::array<float,3>>` 는 메모리상 연속 배치이므로 단일 `file.write()` 로 본문을 통째로 씀.
* PCL 의존 시 빌드 시간/이미지 크기 부담 → 표준 C++만으로 처리.
* 파일명: `lidar_YYYYMMDD_HHMMSS_<3자리ms>.pcd`

##### 저장 경로 설계

```
Jetson Docker 컨테이너 내부 :  /root/sam_ws/storage/lidar_img1
        ↕  볼륨 마운트
Jetson 호스트 OS               :  /home/sam-cam-s1/sam_ws/storage/lidar_img1
        ↕  SCP / scp
서버 PC (mainwindow.py)        :  PCD 파일 회수
```

##### 자동 정리 로직
* **시작 시**: `cleanup_storage_folder()` — 기존 모든 `.pcd` 삭제
* **중지 시**: `cleanup_old_pcd_files()` — 300초(5분) 이상 된 PCD 삭제

→ 파일 시스템이 무한히 차지 않도록 함. 단점: 5분 이상 데이터가 필요하면 수동 백업 필요.

##### lidarCallback() 의 PointCloud2 파싱

```cpp
// PointCloud2 의 각 필드(field) 의 offset 을 동적 검색
for (auto& field : msg->fields) {
    if (field.name == "x") x_offset = field.offset;
    // ...
}
// memcpy 로 단일 포인트 추출
std::memcpy(&x, point_ptr + x_offset, sizeof(float));
```

* `pcl_conversions` 비의존 — 메시지 구조체만 사용
* `point_step` (한 포인트의 바이트 수) 와 `field.offset` 으로 어떤 형식이든 호환

### 3.4 `MultiActuator.py` — 카메라 커버 액추에이터 제어

#### 역할
`/perc/cover` 토픽 (`std_msgs/String`) 으로 `"카메라이름:open"` / `"카메라이름:close"` 명령을 받아, **MightyZap 직선 액추에이터** 6대 중 매핑된 ID를 비동기로 이동시켜 **카메라 보호 커버를 여닫는** 노드.

#### 핵심 매핑

`MultiActuator.py:18–29`
```python
SERIAL_PORT = '/dev/ttyACTUATOR'
BAUD_RATE   = 57600

CAMERA_ACTUATOR_MAP = {
    'Robot_Local':     [1],
    'Gentri_Global1':  [2],
    'Gentri_Global2':  [3],
    'Gentri_Global3':  [4],
    'Gentri_Global4':  [5],
    'Robot_Spare':     [9],
}

OPEN_POSITIONS   = [3670]   # MightyZap GoalPosition
CLOSED_POSITIONS = [0]
```

> miniPC 단독으로는 본인 카메라(`Robot_Local`) 의 ID=1 만 사용하지만, 코드 자체는 다중 카메라 시스템에서 재사용 가능하도록 구조화.

#### 상태 머신

```
camera_states  : { camera_name: bool(open=True / close=False) }
camera_moving  : { camera_name: bool(이동 중) }
```

##### `cam_cover_cb()` 처리 순서 (`MultiActuator.py:66–124`)

```
1. msg.data 파싱: "카메라이름:open|close"
2. CAMERA_ACTUATOR_MAP 에서 액추에이터 ID 조회
3. 이동 중이면 ForceEnable=0 으로 강제 중단 (override)
4. 현재 상태와 동일하면 무시 (중복 방지)
5. camera_moving = True, camera_states 갱신
6. asyncio.run(move_actuators()) — GoalPosition 명령 + 4.5초 대기
7. 4.5초 후 ForceEnable=0 (홀딩 토크 해제)
8. camera_moving = False
```

##### 비동기 이동 로직

```python
async def move_actuators(self, camera_name, target_positions):
    tasks = []
    for aid, tpos in zip(actuator_ids, target_positions):
        MightyZap.GoalPosition(aid, tpos)
        tasks.append(self.wait_for_position(camera_name, aid, tpos))
    await asyncio.gather(*tasks)        # 동시 이동 + 동시 대기

async def wait_for_position(...):
    await asyncio.sleep(4.5)            # 단순 시간 대기 (피드백 폐쇄루프 없음)
    MightyZap.ForceEnable(actuator_id, 0)
```

* 위치 피드백이 아닌 **고정 4.5초 대기** — 액추에이터 풀 스트로크 시간 기준
* 다중 ID 동시 이동 가능 (단일 카메라가 여러 액추에이터를 갖는 경우)
* 종료 시 (l. 182–186) 모든 액추에이터 ForceEnable=0 → 안전 OFF

#### `asyncio.run()` 호출 시 주의

`asyncio.run()` 은 **새 이벤트 루프를 생성하고 닫는** 호출이므로, ROS2 콜백마다 매번 호출하면 약간의 오버헤드가 있습니다. 다만 이 노드는 명령 빈도가 낮으므로(사람이 누르는 버튼 수준) 문제 되지 않습니다.

### 3.5 `PythonLibMightyZap_PC.py` — MightyZap 시리얼 라이브러리

#### 역할
[MightyZap 직선 액추에이터](https://www.mightyzap.com/) 의 시리얼 통신 프로토콜(Dynamixel 유사)을 구현한 저수준 라이브러리.

#### 패킷 구조

```
[0xFF][0xFF][0xFF][ID][LEN][INST][PARAM..][CHECKSUM]
                          ↑     ↑
                          │     └ INSTRUCTION (PING/READ/WRITE/...)
                          └ LEN = TxBuffer_index - 4
CHECKSUM = (~ Σ bytes[3:end]) & 0xFF
```

#### 명령 코드 (`PythonLibMightyZap_PC.py:5–13`)

```python
MIGHTYZAP_PING          = 0xf1
MIGHTYZAP_READ_DATA     = 0xf2
MIGHTYZAP_WRITE_DATA    = 0xf3
MIGHTYZAP_REG_WRITE     = 0xf4
MIGHTYZAP_ACTION        = 0xf5
MIGHTYZAP_RESET         = 0xf6
MIGHTYZAP_RESTART       = 0xf8
MIGHTYZAP_FACTORY_RESET = 0xf9
MIGHTYZAP_SYNC_WRITE    = 0x73
```

#### 자주 쓰이는 고수준 함수

| 함수 | 시그니처 | 의미 |
|------|---------|------|
| `OpenMightyZap(portname, BaudRate)` | 포트 열기 | timeout=0.1s |
| `CloseMightyZap()` | 포트 닫기 | |
| `GoalPosition(bID, position)` | 0x86 레지스터 쓰기 (2B LE) | 목표 위치 |
| `PresentPosition(bID)` | 0x8C 레지스터 읽기 → 9B 응답 | 현재 위치 |
| `ForceEnable(bID, on)` | (헤더에서 사용됨, 본문 미공개) | 토크 on/off |
| `Acceleration / Deceleration / ShortStrokeLimit / LongStrokeLimit` | 각 레지스터 쓰기 | 모션 파라미터 |
| `Sync_write_data(addr, data, size)` | 0xFE ID 로 동기 쓰기 | 다축 동시 명령 |

#### 사용상 주의

* `MZap = serial.Serial()` 이 모듈 전역 → **단일 포트만 동시 사용 가능**
* `RxBuffer` 도 전역 → 멀티스레드 동시 호출 시 race condition 가능 (현재 시스템은 단일 콜백 직렬화로 해결)
* `ReceivePacket` 의 timeout 은 5회 polling (각 read 0.1s) → 약 0.5초

### 3.6 `flaskapp.py` — (구버전, 비활성)

`sensor_cam_mini/sensor_cam_mini/flaskapp.py` 는 **전체 라인이 주석 처리된 폐기 파일**입니다.

원래 의도:
* TCP 9100 으로 **카메라 노드의 JPEG 바이트 수신** → Flask `/video_feed` 로 MJPEG 재방송
* `/cam_command` (String) 구독 → `rec_on/off, save_on/off, detect_on/off, MAX_FRAME, READY, ...` 명령 처리
* `/imgpro_report` 발행으로 처리 결과 반환

현재 구조에서는:
* **카메라 노드가 직접 HTTP MJPEG 서버 운영** (l. 726–814) → 중계 Flask 불필요
* 명령 처리는 서버 PC `mainwindow.py` 로 이관

→ 파일 삭제 후보. 단 launch 파일에서 install 대상으로 등록되어 있으므로 (CMakeLists.txt:97) 삭제 시 함께 정리 필요.

### 3.7 Launch 파일

#### `sensor_cam_mini.launch.py` (메인)
```python
camera_node + lidar_node 을 동시 실행
  - tcp_server_port: 기본 9100 (camera_node 인자)
  - RMW_IMPLEMENTATION = rmw_fastrtps_cpp
```

* `camera_node` 는 `argv[1]` 로 포트 받음 (현재 main()은 사용 안 함, hard-coded 9100)
* `lidar_node` 는 인자 없음

#### `act.launch.py`
```python
MultiActuator.py 단독 실행
```
보통 카메라/라이다와 별도 터미널에서 실행. 액추에이터 시리얼 포트가 필요하므로 Docker `--device /dev/ttyACTUATOR` 필요.

#### `flaskapp.launch.py`
**전체 주석 처리됨, 빈 LaunchDescription 반환**. 폐기.

---

## 4. 메인 시스템 ② `lsd_interface`

### 4.1 패키지 구조

```
lsd_interface/
├── main.py                  ★ ROS2 노드 (perc/lds 토픽 ↔ 시리얼)
├── lds_controller.py        ★ JRT U81 바이너리 시리얼 프로토콜 드라이버
├── requirements.txt         ← fastapi, uvicorn, pyserial, websockets
├── 실행가이드.md             ← 한글 실행/트러블슈팅 가이드
│
├── static/                  ← 구버전 단독 웹 UI (참고용, 미사용)
│   ├── index.html
│   ├── style.css
│   └── app.js
└── __pycache__/             ← 컴파일된 .pyc (cpython-38)
```

> ⚠️ **주의**: 이 패키지는 colcon 빌드 대상이 아닙니다. `package.xml`/`setup.py`/`CMakeLists.txt` 가 모두 **없습니다**. 그냥 `python3 main.py` 로 직접 실행하는 구조.
>
> `requirements.txt` 에 fastapi/uvicorn/websockets 가 있지만 **현재 main.py 에서는 import하지 않습니다** (구버전 잔재). 실제 필수 의존성은 **rclpy + pyserial** 둘.

### 4.2 `main.py` — LDS ROS2 브릿지 노드

#### 역할
**JRT U81 레이저 거리 센서**(USB-Serial CH340 → `/dev/ttyLDS`)를 ROS2 토픽 `perc/lds` 와 양방향 연결하는 브릿지 노드.

#### 클래스: `LDSNode` (rclpy.node.Node)

##### 초기화 순서 (`main.py:37–58`)
```
1. LDSController(port='/dev/ttyLDS', baudrate=19200) 생성
2. lds.connect()      ← PWREN 토글 + 0x55 자동 보레이트 감지
3. publisher 생성     : perc/lds (std_msgs/String, depth=10)
4. subscription 생성  : perc/lds (자기 자신도 구독!)
5. status_timer 생성  : 2초 주기로 publish_status() 호출
```

#### 양방향 통신 규약

`perc/lds` 토픽 하나로 **명령**과 **응답** 양쪽 다 흐릅니다. 구분은 JSON 필드:

```
명령 (서버 → 로봇):  { "cmd":  "..." }    — 'cmd' 필드 존재
응답 (로봇 → 서버):  { "type": "..." }    — 'type' 필드 존재
```

`topic_callback()` (`main.py:62–82`) 에서 자기가 보낸 응답은 무시:
```python
if 'type' in data:
    return    # 자기 자신이 publish 한 응답 메시지 → 무시
```

#### 명령 라우팅 (`_dispatch()`)

| `cmd` | 호출 메서드 | 응답 타입 |
|-------|-------------|----------|
| `power_on` | `lds.power_on()` | `ack` |
| `power_off` | `lds.power_off()` | `ack` |
| `measure_once` | `lds.measure_once()` | `distance` (성공) / `error` (실패) |
| `continuous_on` | `lds.start_continuous(_cb)` | `ack` (콜백마다 `distance` 추가 발행) |
| `continuous_off` | `lds.stop_continuous()` | `ack` |
| `set_mode` | `lds.set_measure_mode(mode)` | `ack` |
| `read_voltage` | `lds.read_voltage()` | `voltage` |
| 기타 | — | `error` ("알 수 없는 명령") |

* **시리얼 통신은 블로킹** → 별도 daemon 스레드(`threading.Thread(target=self._dispatch, ...)`) 에서 처리해 ROS2 콜백 큐 막힘 방지
* 상태 변경 명령(`power_on`, `power_off`, `continuous_on/off`, `set_mode`) 직후 0.1초 대기 후 `publish_status()` 강제 호출 → UI 즉시 반영

#### 상태 발행 (2초 주기)

```python
{ "type":"status",
  "connected":   bool,
  "port":        "/dev/ttyLDS",
  "powered":     bool,    # 레이저 on/off
  "continuous":  bool,    # 연속 측정 중
  "initialized": bool,    # 0x55 자동 보레이트 감지 완료
  "measure_mode":"slow"|"auto"|"fast",
  "voltage_mv":  int|None }
```

### 4.3 `lds_controller.py` — JRT U81 시리얼 드라이버

#### 역할
JRT U81 레이저 거리 센서의 **바이너리 프로토콜**(PDF 6장 기준)을 완전 구현한 컨트롤러. 시리얼 포트(`19200 8N1`) 와 **PWREN/RST 신호**(USB-Serial 의 RTS/DTR 라인)를 함께 제어해 모듈 전원/리셋까지 다룬다.

#### 하드웨어 신호 매핑

```
JRT U81 모듈        ←→     USB Interface Board (CH340)
─────────────────────────────────────────────────────
PWREN  (전원 EN)    ←→     RTS  (HIGH = ON, LOW = OFF)
nRST   (Reset)      ←→     DTR  (HIGH = 정상, LOW = 리셋)
TXD                 ←→     RXD  (교차)
RXD                 ←→     TXD  (교차)
```

> Linux pyserial 의 `setRTS(True)` 는 **RTS 핀을 LOW** 로 설정하므로 (active-low 관례),
> 코드의 `setRTS(True) → PWREN LOW → 모듈 OFF` 라는 주석은 정확합니다.

#### 시작 시퀀스 (`_do_startup()` — l. 330–365)

```
1. RTS=True, DTR=True               → PWREN/nRST LOW → 완전 OFF
2. sleep(0.15)                      → PWREN OFF 유지
3. RTS=False, DTR=False             → PWREN HIGH → 부팅 시작
4. sleep(0.50)                      → 부팅 완료 대기
5. ser.write(b'\x55')               → 자동 보레이트 감지 트리거
6. 응답 1바이트 == 0x00 인지 확인     → OK
7. (실패 시) 최대 3회 재시도
8. _update_voltage()                → 초기 전압 읽기
```

* JRT U81 은 **별도 보레이트 설정 명령이 없고**, 처음 받은 0x55 의 비트 타이밍으로 보레이트를 자동 감지합니다.
* 응답 0x00 은 **모듈의 디폴트 주소**.

#### 주요 명령 프레임 (l. 65–85)

```python
CMD_BAUD_DETECT       = 0x55
CMD_LASER_ON          = AA 00 01 BE 00 01 00 01 C1   # 레지스터 0x01BE = 0x01
CMD_LASER_OFF         = AA 00 01 BE 00 01 00 00 C0   # 레지스터 0x01BE = 0x00
CMD_MEASURE_ONCE_SLOW = AA 00 00 20 00 01 00 01 22
CMD_MEASURE_ONCE_AUTO = AA 00 00 20 00 01 00 00 21
CMD_MEASURE_ONCE_FAST = AA 00 00 20 00 01 00 02 23
CMD_MEASURE_CONT_*    = ...                          # 연속 측정 (slow/auto/fast)
CMD_STOP_CONT         = 0x58 ('X')                   # 연속 중지
CMD_READ_STATUS       = AA 80 00 00 80
CMD_READ_VOLTAGE      = AA 80 00 06 86               # REG_BAT_VLTG (BCD mV)
```

체크섬 = `(addr ~ data 합) & 0xFF`

#### 측정 응답 프레임 (13B, l. 708–739)

```
[AA][addr][regH][regL][cntH][cntL][D3][D2][D1][D0][SQH][SQL][CS]
 0    1     2     3    4    5    6   7   8   9   10   11   12

Distance (mm) = big-endian uint32 (bytes 6~9)
Signal Quality = big-endian uint16 (bytes 10~11)
Checksum = sum(bytes[1:12]) & 0xFF
```

> SQ 값이 **낮을수록 우수** (잡음 적음). 임계: <100 우수, <500 양호, <1000 보통, ≥1000 불량.

#### 오류 프레임 (단형 9B / 장형 13B, l. 579–639)

```
단형: [EE][addr][regH][regL][00][01][errH][errL][CS]
       ↑ count=1 → 거리 데이터 없음, 상태코드만

장형: [EE][addr][regH][regL][00][04][D3][D2][D1][D0][errH][errL][CS]
       ↑ count=4 → 거리 데이터 + 상태코드 (저신뢰도 측정값)
```

* `count` 필드(bytes 4-5) 로 길이 분기
* 장형 오류에 거리값이 있고 30~25000mm 범위면 `low_confidence: True` 로 반환 (사용자에게 경고와 함께 표시)

#### 상태 코드 (`STATUS_CODES`, l. 38–58)

| 코드 | 의미 |
|------|------|
| 0x0000 | 정상 |
| 0x0001 | 입력 전력 부족 (≥ 2.2V 필요) |
| 0x0005 | 측정 대상 범위 초과 |
| 0x0006 | 잘못된 측정 결과 |
| 0x0007 | 배경 광도 너무 강함 |
| **0x0008** | **레이저 신호 너무 약함** ← 자동 폴백 트리거 |
| 0x000F | 레이저 신호 불안정 |
| 0x000A~0x0011 | 하드웨어 오류 1~7 |

#### 자동 복구 로직

##### A. 명령 타임아웃 → `_reinitialize()` (l. 369–412)
```
1. _initialized = False
2. 연속 중지(0x58) 명령 시도
3. 입력 버퍼 플러시
4. PWREN 재토글 + 0x55 재전송 (= _do_startup())
5. 이전 is_powered 가 True 였으면 LASER_ON 재전송 + 안정화 대기
```

##### B. 0x0008 신호 약함 → Slow 모드 자동 폴백 (l. 202–221)
```
- 현재 모드가 slow 가 아니면 → slow 로 1회 재측정
- slow 모드에서 또 0x0008 → WEAK_SIGNAL_MAX_RETRY(3) 회 재시도
  (광자 누적이 시간에 따라 통합되어 성공 확률 향상)
```

##### C. 측정 후 레이저 자동 OFF 방지 (`_restore_laser()`)
센서가 측정 완료 후 내부적으로 레이저를 끌 수 있으므로, `is_powered` 플래그가 True 면 명시적으로 LASER_ON 재전송.

##### D. 연속 측정 실패 누적 → 자동 재초기화 (l. 491–515)
```python
fail_count = 0
while not self._stop_event.is_set():
    frame = self._read_measure_frame(timeout=cfg['timeout'])
    if frame is None:                  # 타임아웃
        fail_count += 1
        if fail_count >= CONT_MAX_FAIL(3):
            self._reinitialize()
            self.ser.write(cfg['cont'])  # 연속 측정 재시작
            fail_count = 0
```

#### 측정 모드 비교

| 모드 | `cmd` 명령 | 타임아웃 | 광자 누적 시간 | 권장 환경 |
|------|------------|---------|---------------|----------|
| slow | `0xAA 00 00 20 00 01 00 01 22` | 8.0초 | 길다 | 원거리(5~20m), 저반사, 어두운 환경 |
| auto | `0xAA 00 00 20 00 01 00 00 21` | 5.0초 | 적응형 | 일반 |
| fast | `0xAA 00 00 20 00 01 00 02 23` | 3.0초 | 짧다 | 근거리(0~3m), 고반사, 빠른 응답 |

#### 동시성 모델

```python
self._lock = threading.Lock()       # 시리얼 IO 직렬화
self._stop_event = threading.Event() # 연속 측정 인터럽트
self._continuous_thread             # 연속 측정 데몬 스레드
```

* `with self._lock:` 으로 모든 시리얼 read/write 보호
* `_stop_event.is_set()` 을 `_read_until_header()` 루프 내에서 0.1초마다 체크 → 응답 빠른 중단

#### 상태 머신

```
            connect()                       _reinitialize()
   ┌──────────────────────►   INITIALIZED  ◄────────┐
   │                              │                  │
[port not open]               power_on()              │
   ▲                              ▼                  │
   │                          LASER ON          [타임아웃 / 응답없음]
disconnect()                      │
                          ┌───────┼───────┐
                          ▼       ▼       ▼
                      measure  continuous  set_mode
                                    │
                              continuous_off
                                    ▼
                            stop_continuous()
                                    │
                                    └─► LASER ON (복원) → power_off → LASER OFF
```

### 4.4 `static/` — 구버전 웹 UI (참고용)

`static/index.html`, `style.css`, `app.js` 는 **현재 시스템에서 사용되지 않습니다.**

원래는 `lsd_interface` 가 FastAPI/uvicorn 기반 단독 웹 서버였고, WebSocket(`/ws`) 으로 직접 LDS 를 제어하는 패널 UI 였습니다 (`requirements.txt` 의 fastapi/uvicorn/websockets 가 이 잔재).

현재는:
* UI 는 **서버 PC 의 `mainwindow.py`** 에서 통합 제공
* `lsd_interface` 는 **순수 ROS2 브릿지**로만 동작

다만 코드 자체는 잘 만들어져 있어 **단독 디버그 UI** 로 유용합니다. 파일별 구성:

| 파일 | 라인 | 내용 |
|------|------|------|
| `index.html` | 111 | 패널 레이아웃 — 거리 표시 / 전원 / 측정 / 모드 / 이력 |
| `style.css` | 500 | GitHub Dark 테마, 반응형, 신호품질 색상 등급 |
| `app.js` | 327 | WebSocket(`/ws`) 클라이언트, 자동 재연결, 토스트 알림 |

UI 구성:
* **거리 메인 카드**: `1 519 mm` 큰 글씨 + `1.519 m` 보조 + 신호품질 바 + 저신뢰도 배지
* **전원 카드**: 레이저 ON/OFF 버튼
* **측정 카드**: 순간 측정 / 연속 측정 토글
* **모드 카드**: Slow / Auto / Fast 라디오
* **이력 카드**: 최근 50건 측정값 (시간순 prepend)
* **토스트**: 우하단 알림 (success/error/info)

> 이 UI 를 ROS2 브릿지 환경에서 살리려면 `main.py` 에 `fastapi` 서버를 다시 추가하거나, `mainwindow.py` 의 화면을 그대로 쓰면 됩니다.

---

## 5. 지원 패키지

### 5.1 `flir_camera_driver`

> **역할**: ROS2 표준 FLIR Spinnaker 드라이버 (참고용, 본 시스템에서는 미사용)

```
flir_camera_driver/
├── flir_camera_description/    — 카메라 메시 등 description
├── flir_camera_msgs/           — 사용자 정의 ROS 메시지 (ImageMetaData)
├── spinnaker_camera_driver/    ★ 메인 드라이버
│   ├── include/spinnaker_camera_driver/
│   │   ├── camera.hpp
│   │   ├── camera_driver.hpp
│   │   ├── exposure_controller.hpp
│   │   ├── image.hpp
│   │   └── spinnaker_wrapper.hpp
│   ├── src/
│   │   ├── camera.cpp
│   │   ├── camera_driver.cpp
│   │   ├── camera_driver_node.cpp     ← ROS2 executable
│   │   ├── spinnaker_wrapper.cpp
│   │   └── spinnaker_wrapper_impl.cpp
│   ├── config/
│   │   ├── blackfly.yaml          — Blackfly 시리즈 파라미터 매핑
│   │   ├── blackfly_s.yaml        — Blackfly S
│   │   ├── chameleon.yaml
│   │   ├── flir_ax5.yaml
│   │   ├── grasshopper.yaml
│   │   └── oryx.yaml
│   └── launch/
│       ├── driver_node.launch.py
│       └── multiple_cameras.launch.py
└── spinnaker_synchronized_camera_driver/  — 다중 카메라 동기화 (Master/Follower)
```

**주요 구성요소**

| 컴포넌트 | 역할 |
|----------|------|
| `SpinnakerWrapper` | Spinnaker C++ SDK 의 Pimpl 래퍼 (heavy SDK 헤더 격리) |
| `CameraDriver` | ROS2 Node 베이스, 파라미터 ↔ GenICam 노드 매핑 |
| `ExposureController` | 자동 노출 컨트롤 (Master/Follower 동기화 시 master 가 노출 시간 계산 → follower 에 전파) |
| `*_camera.yaml` | YAML 로 ROS2 파라미터 ↔ Spinnaker GenICam 노드 이름 / 타입 매핑 |
| `synchronizer.hpp` | `time_estimator`, `time_keeper` 로 다중 카메라 PTP/TimeSync |

> **본 시스템(sensor_cam_mini) 에서 이 드라이버를 안 쓰는 이유**:
> - 단일 카메라, 1fps 저속 운영
> - **HTTP MJPEG 직접 스트리밍** 필요 (image_transport 보다 단순)
> - **Auto ForceIP** 자동 보정 (Spinnaker 표준 드라이버에는 명시적 노출 없음)
> - **3600x3600 + 중앙 크롭** 등 하드코딩 운영 파라미터

### 5.2 `livox_ros2_driver`

> **역할**: Livox LiDAR (Mid-40, Mid-70, Avia, HAP 등) 의 PointCloud2 발행 드라이버

```
livox_ros2_driver/
├── livox_interfaces/         — 커스텀 메시지 (CustomMsg / CustomPoint)
├── livox_ros2_driver/
│   ├── livox_ros2_driver/    — 핵심 소스
│   │   ├── lddc.{cpp,h}      — Livox Data Distribution Center (publisher 관리)
│   │   ├── ldq.{cpp,h}       — Lidar Data Queue
│   │   ├── lds.{cpp,h}       — LiDAR Data Source 베이스
│   │   ├── lds_lidar.{cpp,h} — 직접 연결 모드
│   │   ├── lds_hub.{cpp,h}   — Livox Hub 모드
│   │   ├── lds_lvx.{cpp,h}   — LVX 파일 재생 모드
│   │   ├── lvx_file.{cpp,h}  — LVX 포맷 파서
│   │   └── livox_ros2_driver.cpp ← main()
│   ├── common/
│   │   ├── comm/             — sdk_protocol, gps_protocol
│   │   ├── FastCRC/          — CRC8/16/32 빠른 계산
│   │   ├── rapidjson/        — JSON 파싱 (config 파일)
│   │   └── rapidxml/         — XML 파싱
│   ├── timesync/
│   │   ├── timesync.{cpp,h}  — PPS/UART 동기화
│   │   └── user_uart/        — UART 시각 패킷
│   └── launch/
│       ├── livox_lidar_launch.py     — 단일 LiDAR
│       ├── livox_lidar_msg_launch.py — CustomMsg 출력
│       ├── livox_lidar_rviz_launch.py
│       ├── livox_hub_launch.py       — Hub 통합
│       └── livox_template_launch.py
└── livox_sdk_vendor/         — Livox-SDK 빌드 래퍼
```

#### 주요 launch 파라미터 (`livox_lidar_launch.py`)

```python
xfer_format    = 0      # 0 = PointCloud2(PointXYZRTL), 1 = CustomMsg
multi_topic    = 0      # 0 = 모든 LiDAR 동일 토픽, 1 = LiDAR 별 토픽
data_src       = 0      # 0 = lidar 직접, 1 = hub
publish_freq   = 10.0   # Hz
output_type    = 0
frame_id       = 'livox_frame'
lvx_file_path  = '/home/livox/livox_test.lvx'
```

#### 발행 토픽
```
/livox/lidar         (PointCloud2)             — 기본
/livox/lidar_<id>    (PointCloud2)             — multi_topic=1 시
```

본 시스템의 `lidar_node.cpp` 는 `/livox/lidar_1` 을 구독합니다 → **multi_topic=1** 로 운영하거나, livox_ros2_driver 가 ID=1 LiDAR 만 발행하도록 config JSON 으로 설정한 상태.

### 5.3 `vision_opencv`

> **역할**: ROS2 ↔ OpenCV 변환

```
vision_opencv/
├── cv_bridge/                — sensor_msgs/Image ↔ cv::Mat 변환 (C++/Python)
│   ├── include/cv_bridge/cv_bridge.h
│   ├── src/cv_bridge.cpp
│   ├── python/cv_bridge/core.py
│   └── test/...
├── image_geometry/           — pinhole/stereo 카메라 모델
│   ├── include/image_geometry/pinhole_camera_model.h
│   ├── include/image_geometry/stereo_camera_model.h
│   └── src/...
├── opencv_tests/             — 통합 테스트 (얼굴 인식 등)
└── vision_opencv/            — 메타패키지
```

* `cv_bridge` 는 `sensor_cam_mini/package.xml` 에 의존성으로 등록되어 있으나, 현재 `camera_node.cpp` 는 **cv_bridge 없이 OpenCV 직접 사용** (l. 688: `cv::Mat mono(...)` 으로 raw 버퍼 wrap).
* 의존성으로만 남겨두고 실제 호출 없음 → 추후 `sensor_msgs/Image` 발행으로 변경 시 사용 예정.

---

## 6. ROS2 토픽/서비스 맵

### 발행 (Publish)

| 노드 | 토픽 | 타입 | 주기/시점 |
|------|------|------|-----------|
| `camera_node_Robot_Local` | `/camera_status/Robot_Local` | std_msgs/String | 1 Hz |
| `lidar_node` | (없음 — 라이다 노드는 sub만) | — | — |
| `livox_ros2_driver_node` | `/livox/lidar_1` | sensor_msgs/PointCloud2 | publish_freq (10Hz) |
| `lds_controller_node` | `perc/lds` | std_msgs/String (JSON) | 2초 + 이벤트 |
| `actuator_controller` | (없음) | — | — |

### 구독 (Subscribe)

| 노드 | 토픽 | 핸들러 |
|------|------|--------|
| `camera_node_Robot_Local` | `/camera_control` | `cameraControlCallback` (게인/노출) |
| `lidar_node` | `/livox/lidar_1` | `lidarCallback` (NaN 제거 + 누적) |
| `lds_controller_node` | `perc/lds` | `topic_callback` ('cmd' 만 처리) |
| `actuator_controller` | `perc/cover` | `cam_cover_cb` ("이름:open/close") |

### 서비스 (Service)

| 노드 | 서비스 | 타입 | 동작 |
|------|--------|------|------|
| `lidar_node` | `/lidar_collect` | std_srvs/SetBool | true=수집시작, false=중지+PCD저장 |

### HTTP

| 서비스 | 포트 | 경로 | 설명 |
|--------|------|------|------|
| `camera_node` MJPEG | 9100 | `/` | `multipart/x-mixed-replace` MJPEG 스트림 |

### 외부 시리얼

| 노드 | 디바이스 | 보레이트 | 프로토콜 |
|------|---------|---------|---------|
| `actuator_controller` | `/dev/ttyACTUATOR` | 57600 | MightyZap (Dynamixel-like) |
| `lds_controller_node` | `/dev/ttyLDS` | 19200 | JRT U81 binary |

---

## 7. 데이터 플로우

### 7.1 영상 스트리밍 데이터 플로우

```
FLIR Blackfly S (GigE)
     │
     │  GigE Vision (UDP, 1400B 패킷)
     ▼
  eth2 (USB-Ethernet RTL8153, MTU 1500)
     │
     │  Linux 소켓 (rmem_max=25MB)
     ▼
[camera_node] grabAndStoreFrame() (1Hz)
     │
     ├─► cv::Mat (CV_8UC1, 3600×3600)
     ├─► cv::imencode(".jpg", quality=60)
     ├─► std::shared_ptr<vector<uchar>>
     │
     ├─► std::mutex / std::condition_variable
     │     └─ frame_seq_++ → notify_all()
     ▼
[handleClientConnection] (per-client thread)
     │  HTTP multipart/x-mixed-replace
     ▼
TCP :9100  ──► 서버 PC 브라우저 / OpenCV / VLC
```

### 7.2 라이다 PCD 수집 플로우

```
Livox LiDAR
   │  Ethernet (livox_sdk_vendor)
   ▼
[livox_ros2_driver_node] /livox/lidar_1 (PointCloud2 @ 10Hz)
   │
   ▼
[lidar_node] lidarCallback()
   │
   │  if (collecting_):
   │      x,y,z 추출 (NaN 제외) → accumulated_points_ 누적
   ▼
[서버 PC] /lidar_collect 서비스 (data=false)
   │
   ▼
[lidar_node] save_pcd_file()
   │  Binary PCD 저장 (12B/point)
   ▼
/root/sam_ws/storage/lidar_img1/lidar_YYYYMMDD_HHMMSS_NNN.pcd
   │
   │  Docker 볼륨 마운트
   ▼
/home/sam-cam-s1/sam_ws/storage/lidar_img1/...
   │
   │  SCP / scp
   ▼
서버 PC
```

### 7.3 LDS 거리 측정 플로우

```
[웹 UI / mainwindow.py] 사용자가 "측정" 클릭
   │
   ▼
[서버 PC] perc/lds publish: {"cmd":"measure_once"}
   │
   ▼  ROS2 DDS
   ▼
[lds_controller_node] topic_callback() → _dispatch()
   │
   │  threading.Thread() 별도 실행 (시리얼 블로킹 방지)
   ▼
[LDSController.measure_once()]
   │  1. self._lock 획득
   │  2. ser.reset_input_buffer()
   │  3. ser.write(CMD_MEASURE_ONCE_SLOW)
   │  4. _read_measure_frame(timeout=8.0)
   │     - 0xAA 헤더 발견 시 12B 추가 read
   │     - 0xEE 발견 시 오류 프레임 파싱
   │     - 타임아웃 시 _reinitialize() 후 1회 재시도
   │  5. 0x0008 신호약함이면 slow 모드로 폴백
   │  6. _restore_laser() — 레이저 ON 상태 복원
   │
   │  결과: { success, mm, m, signal_quality, low_confidence?, timestamp }
   ▼
[LDSNode._publish()] perc/lds publish: {"type":"distance", ...}
   │
   ▼  ROS2 DDS
   ▼
[서버 PC] perc/lds 구독 → UI 거리값 갱신
```

### 7.4 카메라 커버 제어 플로우

```
[서버 PC] perc/cover publish: "Robot_Local:open"
   │
   ▼  ROS2 DDS
   ▼
[actuator_controller] cam_cover_cb()
   │
   │  1. CAMERA_ACTUATOR_MAP['Robot_Local'] = [1]
   │  2. desired_flag = True (open)
   │  3. camera_states['Robot_Local'] != True 확인
   │  4. camera_moving = True
   │  5. asyncio.run(move_actuators('Robot_Local', [3670]))
   │     ├─ MightyZap.GoalPosition(1, 3670)   — 명령 전송
   │     ├─ asyncio.sleep(4.5)                — 이동 시간
   │     └─ MightyZap.ForceEnable(1, 0)       — 토크 OFF
   │  6. camera_moving = False
   ▼
/dev/ttyACTUATOR (UART 57600)
   │
   ▼
MightyZap 액추에이터 ID=1  → 카메라 커버 OPEN
```

---

## 8. 빌드 및 실행 가이드

### 8.1 환경 요구사항

| 항목 | 요구 |
|------|------|
| OS | Ubuntu 20.04 / 22.04 (Jetson 의 경우 L4T) |
| ROS2 | Foxy / Humble |
| Python | 3.8+ |
| Spinnaker SDK | `/opt/spinnaker/` 설치 |
| OpenCV | CUDA 모듈 포함 빌드 (Jetson) |
| GStreamer | ≥ 1.14 |
| Livox-SDK | 시스템 라이브러리 |

### 8.2 시리얼 udev 규칙 (권장)

`/etc/udev/rules.d/99-sam-cam.rules`:
```
# JRT U81 LDS (CH340)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  ATTRS{serial}=="<LDS_SERIAL>", SYMLINK+="ttyLDS", MODE="0666"

# MightyZap 액추에이터
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", \
  SYMLINK+="ttyACTUATOR", MODE="0666"
```

이 규칙으로 코드의 `/dev/ttyLDS`, `/dev/ttyACTUATOR` 가 항상 같은 장치를 가리킴.

### 8.3 빌드 (colcon)

```bash
cd ~/sam_ws
source /opt/ros/humble/setup.bash

# sensor_cam_mini 빌드 (livox_ros2_driver, vision_opencv 의존)
colcon build --packages-select livox_interfaces livox_sdk_vendor livox_ros2_driver \
                                cv_bridge image_geometry sensor_cam_mini \
             --symlink-install

source install/setup.bash

# lsd_interface 는 colcon 빌드 대상 아님 — 그냥 python3 으로 실행
```

### 8.4 실행

#### 터미널 1: Livox 드라이버 (라이다 데이터 발행)
```bash
source /opt/ros/humble/setup.bash
source ~/sam_ws/install/setup.bash
ros2 launch livox_ros2_driver livox_lidar_launch.py
```

#### 터미널 2: 카메라 + 라이다 수집 노드
```bash
ros2 launch sensor_cam_mini sensor_cam_mini.launch.py
```
* MJPEG 스트림: `http://<miniPC_IP>:9100`
* `/camera_status/Robot_Local` 토픽으로 헬스 모니터

#### 터미널 3: 액추에이터 제어
```bash
ros2 launch sensor_cam_mini act.launch.py
# 또는: ros2 run sensor_cam_mini MultiActuator.py
```

#### 터미널 4: LDS 브릿지
```bash
cd ~/sam_ws/src/lsd_interface
python3 main.py
```

### 8.5 동작 검증 명령

```bash
# 토픽 리스트
ros2 topic list

# 카메라 상태 (1Hz, "normal" 또는 "error:...")
ros2 topic echo /camera_status/Robot_Local

# LDS 상태 (2초 간격)
ros2 topic echo /perc/lds

# LDS 명령 직접 발행 (CLI 테스트)
ros2 topic pub --once /perc/lds std_msgs/String '{data: "{\"cmd\":\"power_on\"}"}'

# 라이다 수집 시작/중지
ros2 service call /lidar_collect std_srvs/srv/SetBool '{data: true}'
ros2 service call /lidar_collect std_srvs/srv/SetBool '{data: false}'

# 카메라 커버 명령
ros2 topic pub --once /perc/cover std_msgs/String '{data: "Robot_Local:open"}'
ros2 topic pub --once /perc/cover std_msgs/String '{data: "Robot_Local:close"}'
```

---

## 9. 주요 설계 포인트와 트러블슈팅 노트

### 9.1 카메라 (`camera_node.cpp`)

| 증상 | 원인 / 조치 |
|------|------------|
| **Image incomplete** 경고 | GigE 패킷 손실. `StreamPacketResendEnable=true` + `GevSCPD=25000ns` 가 이미 적용. NIC 변경 시 `GevSCPSPacketSize` 를 MTU-100 수준으로 조정 |
| **카메라 검출 안됨** | `autoForceIP()` 가 192.168.2.1 로 ForceIP 시도. eth2 인터페이스의 호스트 IP 가 192.168.2.x/24 인지 확인 |
| **60초간 프레임 없음** | `watchdogCheck()` 가 자동 재초기화. `last_good_frame_time_` 갱신 시점은 `grabAndStoreFrame()` 성공 시 |
| **MTU 9000** 에 잘못 설정됨 | 코드가 자동 감지 후 1500 으로 복원 (`configureNetworkForGigE()`) |
| **소켓 권한 오류** | `sysctl -w net.core.rmem_max=...` 가 실패 (root 필요). Docker `--privileged` 또는 `sysctl -p` 사전 설정 |
| **MJPEG 클라이언트 끊김** | `accept()` 후 thread detach 라 한 클라이언트 끊겨도 다른 클라이언트 영향 없음. `MSG_NOSIGNAL` 로 SIGPIPE 차단 |

### 9.2 라이다 (`lidar_node.cpp`)

| 증상 | 원인 / 조치 |
|------|------------|
| **PCD 파일이 안 보임** | `/root/sam_ws/storage/lidar_img1` 가 Docker 볼륨 마운트 되어 있는지 확인 |
| **5분 후 파일 사라짐** | `cleanup_old_pcd_files()` 가 300초 임계 — 필요 시 코드 수정 |
| **빈 PCD (0 points)** | NaN 만 들어옴 → Livox 드라이버 발행 확인. `ros2 topic echo /livox/lidar_1` 로 메시지 검증 |
| **수집 시작 후 즉시 중지** | `accumulated_points_.reserve(10M)` 메모리 확보 실패 (10MB × 12B = 120MB). RAM 부족 시 reserve 값을 줄여야 함 |

### 9.3 LDS (`lds_controller.py`)

| 증상 | 원인 / 조치 |
|------|------------|
| **자동 보레이트 감지 실패** | RTS/DTR 권한 부족 (Docker `--privileged` 필요), 또는 USB 케이블이 충전 전용 |
| **0x0008 신호 약함 반복** | 이미 slow 모드로 자동 폴백 + 3회 재시도. 그래도 실패 시 타겟 반사율 확인 / 거리 단축 / 전압 측정 (≥2.8V) |
| **공급 전압 < 2.8V** | USB 케이블/허브 문제. 외부 3.3V 전원 권장 |
| **연속 측정이 멈춤** | `CONT_MAX_FAIL=3` 회 타임아웃 → 자동 재초기화. 5번 실패 시 영구 종료 |
| **포트 점유 (`/dev/ttyLDS busy`)** | `sudo fuser /dev/ttyLDS` → `sudo kill <PID>` |
| **'connected'=False 가 계속 표시** | `_initialized` 플래그가 False. `_do_startup()` 의 0x55 응답이 0x00 인지 확인 |

### 9.4 액추에이터 (`MultiActuator.py`)

| 증상 | 원인 / 조치 |
|------|------------|
| **이동이 안 됨** | `ForceEnable(aid, 1)` 호출이 어디서도 없음 — `GoalPosition` 명령만으로 자동 활성화되는 MightyZap 펌웨어 의존. 재현 불가 시 `ForceEnable(aid, 1)` 명시 호출 추가 |
| **이동 중 다음 명령 무시** | `camera_moving[name]==True` 일 때 ForceEnable=0 으로 강제 중단 후 새 명령 처리 (현재 코드 l. 100–106) |
| **동일 명령 반복 → 무동작** | `current_state == desired_flag` 체크 (l. 109–111) — 정상 동작 |
| **포트 권한** | Docker `--device /dev/ttyACTUATOR` 필요 |

### 9.5 ROS2 도메인 / 네트워킹

| 증상 | 원인 / 조치 |
|------|------------|
| **서버 PC 가 토픽 못 봄** | `ROS_DOMAIN_ID` 가 양쪽 동일한지 확인 (기본 0) |
| **DDS Discovery 실패** | 방화벽 7400-7500 UDP 열기. 또는 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (현재 launch 파일이 강제) |
| **다중 NIC 환경에서 토픽 안보임** | FastRTPS XML 프로파일로 인터페이스 지정 필요 |

---

## 부록 A: 파일별 라인 카운트 (대략)

| 파일 | 라인 수 | 주요 책임 |
|------|--------|-----------|
| `sensor_cam_mini/src/camera_node.cpp` | 860 | GigE 카메라 + MJPEG 서버 + 워치독 |
| `sensor_cam_mini/src/lidar_node.cpp` | 306 | PointCloud2 수집 + Binary PCD 저장 |
| `sensor_cam_mini/sensor_cam_mini/MultiActuator.py` | 192 | 액추에이터 6채 비동기 제어 |
| `sensor_cam_mini/sensor_cam_mini/PythonLibMightyZap_PC.py` | ~600 | MightyZap 시리얼 프로토콜 |
| `sensor_cam_mini/sensor_cam_mini/flaskapp.py` | 284 | (전체 주석, 폐기) |
| `lsd_interface/main.py` | 227 | LDS ROS2 브릿지 노드 |
| `lsd_interface/lds_controller.py` | 746 | JRT U81 바이너리 시리얼 드라이버 |
| `lsd_interface/static/index.html` | 111 | (구버전) 단독 웹 UI HTML |
| `lsd_interface/static/style.css` | 500 | 단독 웹 UI 스타일 |
| `lsd_interface/static/app.js` | 327 | 단독 웹 UI WebSocket 클라이언트 |

## 부록 B: 외부 인터페이스 요약 (Cheat Sheet)

```
┌─────────────────┬──────────────────────┬──────────────────────────────────────┐
│ 노드            │ 외부 인터페이스      │ 형식                                  │
├─────────────────┼──────────────────────┼──────────────────────────────────────┤
│ camera_node     │ HTTP :9100           │ multipart/x-mixed-replace MJPEG       │
│                 │ /camera_control      │ "<카메라이름>:set_gain:<float>"       │
│                 │ /camera_control      │ "<카메라이름>:set_auto_exposure:<On|Off>" │
│                 │ /camera_control      │ "<카메라이름>:set_exposure:<float us>"│
│                 │ /camera_status/<name>│ "normal" / "error:<msg>"             │
├─────────────────┼──────────────────────┼──────────────────────────────────────┤
│ lidar_node      │ /lidar_collect       │ SetBool: true=시작, false=저장+중지  │
│                 │  → response.message  │ "STOPPED:<경로>:<포인트수>"          │
├─────────────────┼──────────────────────┼──────────────────────────────────────┤
│ actuator_ctrl   │ /perc/cover          │ "<카메라이름>:open" / ":close"       │
├─────────────────┼──────────────────────┼──────────────────────────────────────┤
│ lds_controller  │ /perc/lds (cmd)      │ JSON: {"cmd":"power_on"} 등          │
│                 │ /perc/lds (응답)     │ JSON: {"type":"status|distance|ack|voltage|error", ...} │
└─────────────────┴──────────────────────┴──────────────────────────────────────┘
```

## 부록 C: 메시지 예시

### LDS 명령 / 응답

```jsonc
// ─── 서버 → 로봇 ─────
{ "cmd": "power_on" }
{ "cmd": "power_off" }
{ "cmd": "measure_once" }
{ "cmd": "continuous_on" }
{ "cmd": "continuous_off" }
{ "cmd": "set_mode", "mode": "slow" }   // slow|auto|fast
{ "cmd": "read_voltage" }

// ─── 로봇 → 서버 ─────
{ "type": "status",
  "connected": true, "port": "/dev/ttyLDS",
  "powered": true, "continuous": false, "initialized": true,
  "measure_mode": "slow", "voltage_mv": 3300 }

{ "type": "ack", "cmd": "power_on", "success": true,
  "message": "레이저 ON 완료" }

{ "type": "distance", "success": true,
  "mm": 1519, "m": 1.519,
  "signal_quality": 315, "timestamp": 1746489600.0 }

{ "type": "distance", "success": true,
  "mm": 8420, "m": 8.420,
  "signal_quality": 9999,           // SQ=9999 = 오류 프레임에서 추출된 값
  "low_confidence": true,
  "timestamp": 1746489605.0 }

{ "type": "voltage", "success": true,
  "voltage_v": 3.300, "voltage_mv": 3300, "message": "3.300 V" }

{ "type": "error", "message": "센서가 연결되어 있지 않습니다." }
```

### 카메라 명령 (`/camera_control`)
```
"Robot_Local:set_gain:5.0"
"Robot_Local:set_auto_exposure:On"
"Robot_Local:set_auto_exposure:Off"
"Robot_Local:set_exposure:10000"     // 10ms in microseconds
```

### 카메라 커버 (`/perc/cover`)
```
"Robot_Local:open"
"Robot_Local:close"
"Gentri_Global1:open"
```

---