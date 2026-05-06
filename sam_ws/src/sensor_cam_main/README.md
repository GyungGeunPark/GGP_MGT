# `mainwindow.py` 코드 상세 분석 문서

> **파일 경로**: `sensor_cam_main/sensor_cam_main/mainwindow.py`
> **총 라인 수**: 2,468 줄
> **역할**: ROS2 기반 다중 카메라/LiDAR 통합 관리 시스템 (Flask 웹 인터페이스)
> **작성자**: jyp7781@ff00ff.kr (Magenta Robotics)

---

## 목차

1. [전체 개요](#1-전체-개요)
2. [시스템 아키텍처](#2-시스템-아키텍처)
3. [Import 모듈 분석](#3-import-모듈-분석)
4. [전역 함수 및 객체](#4-전역-함수-및-객체)
5. [`CameraManager` 클래스 상세 분석](#5-cameramanager-클래스-상세-분석)
6. [Flask 웹 서버 구조](#6-flask-웹-서버-구조)
7. [HTML/JavaScript UI 분석](#7-htmljavascript-ui-분석)
8. [ROS2 통신 구조](#8-ros2-통신-구조)
9. [동시성 및 스레드 관리](#9-동시성-및-스레드-관리)
10. [데이터 흐름도](#10-데이터-흐름도)
11. [실행 진입점](#11-실행-진입점)
12. [주요 특징 및 개선 포인트](#12-주요-특징-및-개선-포인트)

---

## 1. 전체 개요

### 1.1 프로그램 목적
`mainwindow.py`는 **5대의 카메라 + LiDAR 센서 + LDS(Laser Distance Sensor)**를 통합 관리하는 **인식 프로그램 매니저**입니다.

**핵심 기능:**
- 다중 카메라 영상 캡처 / 스트리밍 / 녹화
- LiDAR 포인트클라우드(PCD) 수집 및 자동 전송
- 카메라 커버 개폐 제어
- LDS(JRT U81) 레이저 거리 센서 제어
- Detection(측정 모드) 트리거
- 실시간 시스템 상태 모니터링
- 웹 기반 GUI (Flask + HTML/JS)

### 1.2 사용 기술 스택

| 분야 | 기술 |
|------|------|
| 미들웨어 | **ROS2 (rclpy)** + `rmw_fastrtps_cpp` |
| 웹 프레임워크 | **Flask** |
| 영상 처리 | **OpenCV (cv2)** |
| 동시성 | **threading**, **Semaphore**, **Lock** |
| 직렬화 | **JSON**, **struct (binary PCD)** |
| 파일 전송 | **SCP / SSH (subprocess)** |
| 프론트엔드 | **HTML5 + CSS3 + Vanilla JavaScript (Fetch API)** |

### 1.3 관리 대상 카메라 목록

| 카메라 이름 | 카메라 IP | 해상도 | FPS |
|------------|-----------|--------|-----|
| `Robot_Local`     | 192.168.3.150:9100 | 4000×4000 | 1 |
| `Gantry_Global1`  | 192.168.3.151:9101 | 4000×4000 | 1 |
| `Gantry_Global2`  | 192.168.3.152:9102 | 1200×1200 | 1 |
| `Gantry_Global3`  | 192.168.3.153:9103 | 4000×4000 | 1 |
| `Gantry_Global4`  | 192.168.3.154:9104 | 4000×4000 | 1 |

각 카메라는 Jetson 보드(IP `192.168.3.15X`)에 연결되며, LiDAR PCD는 Jetson의 `/home/sam-cam-s1/sam_ws/storage/lidar_img1`에 저장됩니다.

---

## 2. 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                        브라우저(Client)                      │
│   [HTML + CSS + JavaScript Fetch API]                       │
└────────────────────────┬────────────────────────────────────┘
                         │ HTTP (port 5030~5034)
                         ▼
┌─────────────────────────────────────────────────────────────┐
│             Flask Web Server (mainwindow.py)                 │
│   - 라우트: /img_save, /pc_save_start, /lds/command 등       │
│   - HTML_TEMPLATE 렌더링                                     │
└────────────────────────┬────────────────────────────────────┘
                         │ Python 메서드 호출
                         ▼
┌─────────────────────────────────────────────────────────────┐
│              CameraManager (ROS2 노드 + 상태관리)            │
│   ┌────────────────┐  ┌──────────────┐  ┌──────────────┐    │
│   │ ROS2 Pub/Sub   │  │ Camera Threads│  │ LDS State   │    │
│   │ - cam_command  │  │ - capture_loop│  │ - lds topic │    │
│   │ - cover        │  │ - reconnect   │  │             │    │
│   │ - lidar_collect│  │               │  │             │    │
│   └────────┬───────┘  └───────┬───────┘  └──────┬──────┘    │
└────────────┼──────────────────┼─────────────────┼───────────┘
             │                  │                 │
             ▼                  ▼                 ▼
   ┌──────────────────┐  ┌────────────┐   ┌─────────────┐
   │  Jetson 카메라    │  │   OpenCV   │   │  LDS Node   │
   │  (HTTP MJPEG)    │  │  VideoCap  │   │  (perc/lds) │
   └──────────────────┘  └────────────┘   └─────────────┘
             │
             │ SCP / SSH
             ▼
   ┌──────────────────┐
   │ Jetson PCD 저장소 │
   │  *.pcd 파일      │
   └──────────────────┘
```

---

## 3. Import 모듈 분석

```python
# 표준 라이브러리
import os, sys, math, json, cv2, time, datetime
import threading, subprocess, struct
from threading import Semaphore

# ROS2 관련
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Flask
from flask import Flask, request, render_template_string, jsonify, send_from_directory
```

### 3.1 핵심 의존성

| 모듈 | 용도 |
|------|------|
| `cv2` (OpenCV) | 카메라 캡처, 이미지/비디오 저장 |
| `rclpy` | ROS2 Python 클라이언트 |
| `std_msgs.msg.String` | ROS2 문자열 메시지 (JSON으로 시리얼라이즈) |
| `std_srvs.srv.SetBool` | LiDAR 수집 시작/중지 서비스 |
| `subprocess` | SCP/SSH 명령 실행 |
| `struct` | Binary PCD 파일 파싱 |
| `Semaphore` | 동시 PCD 파싱 제한 (최대 2개) |
| `Flask` | 웹 서버 + REST API |

---

## 4. 전역 함수 및 객체

### 4.1 `save_pcd(points, filename)` 함수 — Line 28~62

PCD(Point Cloud Data) v0.7 포맷의 ASCII 파일을 작성하는 유틸리티 함수입니다.

**특징:**
- **배치 쓰기** 기법으로 성능 최적화 (10,000줄 단위 버퍼링)
- 빈 포인트 클라우드도 정상 저장 (`HEIGHT 1`, `POINTS 0`)
- 헤더에 `VIEWPOINT 0 0 0 1 0 0 0` 명시 (단위 쿼터니언 + 원점)

**PCD 헤더 구조:**
```
# .PCD v0.7 - Point Cloud Data file format
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH <num_points>
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS <num_points>
DATA ascii
```

### 4.2 전역 변수
- `app = Flask(__name__)` — Flask 애플리케이션 객체
- `iface = CameraManager()` — 시스템 전역 매니저 인스턴스 (Line 1390)
- `HTML_TEMPLATE` — Jinja2 템플릿 문자열 (HTML+CSS+JS 인라인)

---

## 5. `CameraManager` 클래스 상세 분석

`CameraManager`는 이 프로그램의 **핵심 엔진** 클래스입니다. 5대 카메라, LiDAR, LDS 센서를 통합 관리하며, ROS2 노드 역할을 동시에 수행합니다.

### 5.1 `__init__` 생성자 (Line 65~299)

#### (1) ROS2 노드 초기화
```python
rclpy.init()
self.node = rclpy.create_node('imgpro_node')
```

#### (2) QoS 프로파일 설정
```python
qos_profile_sensor_data = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
```
> 센서 데이터에 적합한 QoS — 신뢰성보다 실시간성을 우선시.

#### (3) 퍼블리셔 생성

| 토픽명 | 메시지 타입 | 용도 |
|--------|-------------|------|
| `/cam_command`     | String | 카메라 일반 명령 |
| `perc/cover`       | String | 카메라 커버 개폐 명령 |
| `/camera_control`  | String | 노출/게인 등 제어 |
| `perc/lds`         | String | LDS 레이저 거리센서 명령 |

#### (4) 서비스 클라이언트
```python
self.lidar_collect_client = self.node.create_client(SetBool, '/lidar_collect')
```
> Jetson 측 `lidar_node`의 `/lidar_collect` 서비스에 `SetBool` 요청을 보내 LiDAR 데이터 수집을 시작/중지합니다.

#### (5) 구독자 생성

| 토픽 | 콜백 | 용도 |
|------|------|------|
| `/camera_status/<cam>` × 5 | `camera_status_callback` | 카메라 상태 수신 |
| `perc/lds`          | `lds_topic_callback` | LDS 응답 수신 |
| `/camera_topic`     | `report_callback` | 카메라 보고 수신 |

#### (6) 카메라별 상태 딕셔너리 (`self.camera_states`)
```python
{
    'cap': None,                     # cv2.VideoCapture 객체
    'is_recording_images': False,    # 이미지 저장 트리거
    'is_recording_video': False,     # 비디오 녹화 중
    'is_collecting_pcl': False,      # PCL 수집 중
    'pcl_points': [],                # 수집된 포인트 누적
    'pcl_files_collected': 0,        # 수집된 PCD 파일 수
    'cover_state': 'closed',         # 커버 상태
    'auto_exposure': 'Off',          # 자동노출
    'exposure_value': 10000.0,       # 노출값
    'gain_value': 6.0,               # 게인값
    'last_saved_img_path': '',       # 마지막 이미지 경로
    'last_saved_pcd_path': '',       # 마지막 PCD 경로
    'last_detection_path': ''        # 마지막 detection 경로
}
```

#### (7) 백그라운드 스레드 시작
- **`spin` 스레드**: ROS2 메시지 처리 (`rclpy.spin_once`)
- **`status_check_loop` 스레드**: 5초마다 카메라 타임아웃 검사
- **각 카메라별 `capture_loop` 스레드**: 영상 캡처 + 자동 재연결

### 5.2 카메라 캡처 메서드

#### `initialize_all_cameras()` — Line 301
- 각 카메라에 대해 `cv2.VideoCapture(URL)` 시도
- 실패해도 **계속 진행** (재연결은 `capture_loop`에서 담당)
- 카메라마다 독립된 락(`cap_locks[cam_name]`) 생성

#### `capture_loop(cam_name)` — Line 331
**주요 책임:**
1. 카메라 프레임을 0.05초 간격으로 폴링
2. 프레임 수신 성공 시 `system_status[cam].status = 'normal'`
3. `is_recording_images=True`이면 즉시 JPG 저장 후 플래그 리셋
4. 연결 실패 시 10초마다 자동 재연결 (`try_reconnect_camera`)
5. `shutdown_requested`가 True이면 종료 + `cap.release()`

```python
while not self.shutdown_requested:
    with self.cap_locks[cam_name]:
        cap = self.camera_states[cam_name]['cap']
        ret, frame = cap.read() if cap else (False, None)

    if ret:
        # 정상 프레임 처리
    else:
        # 10초마다 재연결 시도
        if current_time - last_reconnect_attempt > 10:
            self.try_reconnect_camera(cam_name)
```

#### `try_reconnect_camera(cam_name)` — Line 386
- 기존 `cap.release()` 후 새 `VideoCapture()` 생성
- 락 보호 영역 안에서 안전하게 교체

### 5.3 PCD 파일 처리 메서드

#### `parse_point_cloud_file(filepath, cam_name)` — Line 460
PCD 파일을 ASCII 또는 Binary로 자동 파싱합니다.

**ASCII 모드:**
- 1MB 청크 단위로 파일 읽기
- 마지막 불완전한 줄은 다음 청크에 연결 (`f.seek(...)`)
- 100,000 포인트마다 1ms 휴식 (CPU 양보)

**Binary 모드:**
- `struct.iter_unpack('<fff', binary_data)` (little-endian float ×3)
- 단순/빠른 일괄 읽기

**오류 처리:**
- 헤더에 `WIDTH/HEIGHT/DATA` 누락 시 빈 리스트 반환
- 알 수 없는 DATA 타입 → 로그 후 무시

#### `collect_latest_pcl_files(cam_name)` — Line 411 (Fallback)
폴더 모니터링이 실패할 때 사용하는 **fallback 메커니즘**입니다.
- PCD 파일을 mtime 기준 최신순 정렬
- `Robot_Local`은 1개, 그 외는 5개 선택
- 세마포어(`pcd_parse_semaphore`)로 동시 파싱 제한

#### `collect_and_save_single_pcl(cam_name)` — Line 541
폴더 감시(File Watcher) 방식으로 PCD 파일을 감지/병합/저장합니다.

**알고리즘:**
1. 폴더 내 기존 PCD 파일 목록 스냅샷 (`files_at_start`)
2. 0.5초마다 폴더 스캔
3. 새 파일 발견 시 **파일 크기 변화** 추적
4. 4회 연속(2초) 크기 변화가 없으면 "수집 완료"로 판단
5. 파싱 + 포인트 누적
6. 60초간 새 파일이 없으면 fallback 모드 전환
7. 목표 개수 도달 시 `save_pcd()`로 병합 저장

> **주의**: 이 메서드는 코드에 정의되어 있으나, 현재 메인 흐름인 SCP 방식(`pc_save_start/stop`)에서는 직접 사용되지 않습니다.

### 5.4 SCP 기반 PCD 전송 (방안 B)

#### `_convert_docker_path_to_host(docker_path)` — Line 913
Jetson 도커 컨테이너 내부 경로(`/root/sam_ws/...`)를 호스트 경로(`/home/sam-cam-s1/sam_ws/...`)로 변환합니다.

#### `_fetch_pcd_via_scp(jetson_ip, remote_path, local_path)` — Line 920
```bash
scp -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
    sam-cam-s1@<IP>:<remote> <local>
```
- 60초 타임아웃 기본값
- `subprocess.run(capture_output=True, text=True)` 사용
- 반환값: `bool` (성공 여부)

#### `_cleanup_old_pcd_on_jetson(jetson_ip, pcd_dir)` — Line 946
SSH로 Jetson에 접속해 **5분 이상 된 PCD 파일을 삭제**합니다.
```bash
find <dir> -name "*.pcd" -mmin +5 -delete
```

### 5.5 PCL(포인트클라우드) 수집 흐름

#### `pc_save_start(cam_name, duration_sec=0)` — Line 965

**처리 단계:**
1. 카메라별 락 획득 (`pcl_collection_locks[cam]`)
2. `/lidar_collect` 서비스 호출 → Jetson에서 누적 시작
3. 상태 플래그 `is_collecting_pcl = True` 설정
4. `duration_sec > 0`이면 **자동 정지 타이머 스레드** 생성

```python
def auto_stop():
    time.sleep(duration_sec)
    if self.camera_states[cam_name]['is_collecting_pcl']:
        self.pc_save_stop(cam_name)
threading.Thread(target=auto_stop, daemon=True).start()
```

#### `pc_save_stop(cam_name)` — Line 1051

**처리 단계:**
1. `/lidar_collect` 서비스 중지 호출 (타임아웃 120초)
2. 응답 파싱: `"STOPPED:/path/to/file.pcd:12345678"`
3. SCP로 Jetson → 서버PC PCD 전송
4. 백그라운드에서 Jetson 오래된 파일 정리

**서비스 응답 포맷:**
```
STOPPED:<remote_pcd_path>:<point_count>
```

#### `pc_save_start_all() / pc_save_stop_all()` — Line 1010, 1120
모든 카메라에 대해 일괄 시작/정지. 각각 개별 자동정지 타이머가 별도 스레드로 동작합니다.

### 5.6 LDS (Laser Distance Sensor) 통신

LDS는 **JRT U81** 모델을 사용하며, 별도 ROS 노드와 `perc/lds` 토픽으로 JSON 메시지를 주고받습니다.

#### LDS 상태 구조
```python
self.lds_state = {
    'connected': False,
    'powered': False,
    'continuous': False,
    'measure_mode': 'slow',     # slow | auto | fast
    'voltage_mv': None,
    'last_distance': None,
    'last_error': None,
    'distance_history': []      # 최대 50개 보관
}
```

#### `lds_topic_callback(msg)` — Line 787
들어온 JSON 메시지를 `type` 필드로 분기 처리:

| `type` | 처리 |
|--------|------|
| `status`   | 연결/전원/연속 모드 업데이트 |
| `distance` | 거리값 + history에 추가 (최대 50개) |
| `voltage`  | 전압(mv) 업데이트 |
| `ack`      | (무시) |
| `error`    | 에러 메시지 저장 |

#### `send_lds_command(cmd, **kwargs)` — Line 829
LDS 명령을 JSON으로 직렬화 후 `perc/lds` 토픽에 발행합니다.

```python
payload = {'cmd': 'power_on'}  # or 'measure_once', 'continuous_on', 'set_mode' 등
self.lds_publisher.publish(String(data=json.dumps(payload)))
```

### 5.7 기타 주요 메서드

| 메서드 | 설명 |
|--------|------|
| `img_save(cam_name)` | 단일 카메라 이미지 캡처 (트리거 → 대기 → 결과 반환) |
| `img_save_all()` | 모든 카메라 일괄 캡처 |
| `detection(cam_name)` | Gain=0으로 변경 → 캡처 → 원래 Gain 복원 |
| `open_cover(cam_name)` | 카메라 커버 열기 명령 |
| `close_cover_all()` | 모든 카메라 커버 닫기 |
| `start_recording(cam_name)` | XVID 코덱으로 AVI 녹화 |
| `stop_recording(cam_name)` | 녹화 중지 (플래그 변경) |
| `get_pcl_status(cam_name)` | PCL 수집 상태 조회 (경과시간 포함) |
| `get_system_status()` | 5개 카메라 상태 일괄 조회 |
| `writeLog(txt)` | 로그 파일 + 메모리 버퍼(최대 200건) |

### 5.8 로그 시스템

```python
self.saveLogPath = '/root/sam_ws/worklog/<YYYYMMDD>/'
self.saveImgPath = '/root/sam_ws/imgData/<YYYYMMDD>/'
self.logFileName = '<saveLogPath>/imgpro_<HHMMSS>.log'
```

`writeLog()`는 두 곳에 기록:
1. 디스크 (영구 보관)
2. 메모리 (`log_messages_perc`, 최대 200건 → Flask 웹UI 표시용)

---

## 6. Flask 웹 서버 구조

### 6.1 라우트 매핑표

| URL | HTTP | 핸들러 | 기능 |
|-----|------|--------|------|
| `/`                       | GET  | `index`                  | 메인 페이지 렌더링 |
| `/images/<filename>`      | GET  | `serve_image`            | 이미지 정적 서빙 |
| `/system_status`          | GET  | `system_status`          | 카메라 상태 JSON |
| `/logs_perc`              | GET  | `get_logs`               | 로그 메시지 JSON |
| `/img_save`               | POST | `route_img_save`         | 이미지 캡처 |
| `/pc_save_start`          | POST | `route_pc_save_start`    | PCL 수집 시작 |
| `/pc_save_start_all`      | POST | `route_pc_save_start_all`| 전체 PCL 시작 |
| `/pc_save_stop`           | POST | `route_pc_save_stop`     | PCL 수집 중지 |
| `/pc_save_stop_all`       | POST | `route_pc_save_stop_all` | 전체 PCL 중지 |
| `/pcl_status/<cam>`       | GET  | `pcl_status`             | PCL 진행상태 |
| `/open_cover`             | POST | `route_open_cover`       | 커버 열기 |
| `/close_cover_all`        | POST | `route_close_cover_all`  | 모든 커버 닫기 |
| `/cover_status`           | GET  | `cover_status`           | 커버 상태 |
| `/start_recording_all`    | POST | `route_start_recording_all` | 전체 녹화 시작 |
| `/stop_recording_all`     | POST | `route_stop_recording_all`  | 전체 녹화 중지 |
| `/detection_all`          | POST | `route_detection_all`    | 모든 카메라 측정 |
| `/detection_images`       | GET  | `detection_images`       | 측정 결과 이미지 경로 |
| `/lds/command`            | POST | `lds_command`            | LDS 제어 명령 |
| `/lds/status`             | GET  | `lds_status`             | LDS 상태 조회 |
| `/lds/history`            | GET  | `lds_history`            | LDS 거리 이력 |

### 6.2 포트 자동 탐색 (`main()` 함수)
```python
base_port = 5030
max_attempts = 5
for i in range(max_attempts):
    port = base_port + i
    if not is_port_in_use(port):
        app.run(host='0.0.0.0', port=port, debug=False)
        return
```
- 5030부터 시작해 5034까지 비어있는 포트를 찾아 실행
- 모두 사용 중이면 사용자에게 `lsof | xargs kill` 명령 안내

---

## 7. HTML/JavaScript UI 분석

### 7.1 UI 섹션 구성

```
┌──────────────────────────────────────┐
│  인식 프로그램 매니저                  │ ← H1 헤더
├──────────────────────────────────────┤
│ ① 시스템 상태 (5개 카메라 OFF/NORMAL/ERROR) │
├──────────────────────────────────────┤
│ ② 카메라 커버 (열기 ×5 + 모두닫기)       │
├──────────────────────────────────────┤
│ ③ 영상 캡처 (저장 ×5)                  │
├──────────────────────────────────────┤
│ ④ 포인트 클라우드 수집                  │
│   - 측정시간 입력 + 시작/정지 ×5        │
│   - 모든 카메라 일괄 시작/정지           │
├──────────────────────────────────────┤
│ ⑤ 녹화 (전체 시작/정지)                 │
├──────────────────────────────────────┤
│ ⑥ 카메라 측정 (Detection)               │
│   - 측정 버튼 + 결과 이미지 그리드 ×5   │
├──────────────────────────────────────┤
│ ⑦ LDS 레이저 거리 센서                 │
│   - 연결/전압/거리 표시                 │
│   - 전원/측정/모드 제어                 │
│   - 측정 이력                           │
├──────────────────────────────────────┤
│ ⑧ 로그 메시지 (실시간 갱신)             │
└──────────────────────────────────────┘
```

### 7.2 주요 CSS 스타일
- **그리드 레이아웃**: `grid-template-columns: repeat(6, 1fr)`
- **버튼 색상 의미론**: primary(파랑) / success(초록) / danger(빨강) / warning(주황) / all(노랑)
- **상태 인디케이터**: 12×12 원형 점 (on=초록, off=빨강, recording=주황)
- **로그 영역**: 어두운 배경(#263238) + 모노스페이스 폰트

### 7.3 JavaScript 폴링 주기

```javascript
setInterval(updateLogs, 1000);          // 1초마다 로그 갱신
setInterval(fetchSystemStatus, 2000);   // 2초마다 카메라 상태 갱신
setInterval(fetchLdsStatus, 1000);      // 1초마다 LDS 상태 갱신
```

PCL 수집 중일 때만 `startPCLTimer()`로 카메라마다 1초 간격 폴링이 추가로 동작합니다.

### 7.4 주요 JS 함수

| 함수 | 호출 백엔드 | 용도 |
|------|-------------|------|
| `openCover(cam)`         | `POST /open_cover`        | 커버 열기 |
| `closeCoverAll()`        | `POST /close_cover_all`   | 모든 커버 닫기 |
| `captureImage(cam)`      | `POST /img_save`          | 이미지 저장 |
| `startPCL(cam)`          | `POST /pc_save_start`     | PCL 시작 |
| `stopPCL(cam)`           | `POST /pc_save_stop`      | PCL 중지 |
| `startPCLAll()`          | `POST /pc_save_start_all` | 전체 PCL 시작 |
| `stopPCLAll()`           | `POST /pc_save_stop_all`  | 전체 PCL 중지 |
| `startPCLTimer(cam)`     | `GET /pcl_status/<cam>`   | 진행상황 폴링 |
| `startRecordingAll()`    | `POST /start_recording_all` | 녹화 시작 |
| `stopRecordingAll()`     | `POST /stop_recording_all`  | 녹화 중지 |
| `detectionAll()`         | `POST /detection_all`     | 측정 트리거 |
| `updateDetectionImages()`| `GET /detection_images`   | 결과 이미지 갱신 |
| `ldsCmd(cmd)`            | `POST /lds/command`       | LDS 명령 |
| `ldsToggleCont()`        | (위와 동일)                | 연속 측정 토글 |
| `ldsSetMode(mode)`       | `POST /lds/command` (cmd=set_mode) | 측정 모드 변경 |
| `fetchLdsStatus()`       | `GET /lds/status`         | LDS 상태 갱신 |

---

## 8. ROS2 통신 구조

### 8.1 발행(Publish) 토픽

| 토픽 | 메시지 | 발행 함수 |
|------|--------|----------|
| `/cam_command`     | String | `command_publisher.publish(...)` |
| `perc/cover`       | String | `open_cover()`, `close_cover_all()` |
| `/camera_control`  | String | `detection()` (gain 변경) |
| `perc/lds`         | String | `send_lds_command()` |

### 8.2 구독(Subscribe) 토픽

| 토픽 | 콜백 | 동작 |
|------|------|------|
| `/camera_status/robot_local` | `camera_status_callback` | 5개 카메라 상태 수신 |
| `/camera_status/gantry_global1~4` | (동일) | (동일) |
| `perc/lds`                    | `lds_topic_callback`     | LDS 상태/거리/전압/오류 |
| `/camera_topic`               | `report_callback`        | 카메라 일반 보고 |

### 8.3 서비스(Service) 클라이언트

| 서비스 | 타입 | 용도 |
|--------|------|------|
| `/lidar_collect` | `SetBool` | LiDAR 수집 시작(true) / 정지(false) |

**서비스 응답 포맷**:
- 성공 시: `STOPPED:<pcd_path>:<point_count>`
- 실패 시: 일반 에러 문자열

### 8.4 메시지 명령 포맷

#### 카메라 명령 (예: `/camera_control`)
```
"<camera_name>:set_gain:<value>"
"Gantry_Global1:set_gain:6.0"
```

#### 커버 명령 (`perc/cover`)
```
"<camera_name>:<action>"
"Robot_Local:open"
"Robot_Local:close"
```

#### LDS 명령 (JSON, `perc/lds`)
```json
{"cmd": "power_on"}
{"cmd": "measure_once"}
{"cmd": "continuous_on"}
{"cmd": "set_mode", "mode": "slow"}
```

---

## 9. 동시성 및 스레드 관리

### 9.1 사용된 동기화 프리미티브

| 변수 | 종류 | 보호 대상 |
|------|------|-----------|
| `cap_locks[cam_name]`             | `Lock` × 5 | 각 카메라의 `cv2.VideoCapture` 객체 |
| `pcl_collection_locks[cam_name]`  | `Lock` × 5 | PCL 수집 상태 변경 |
| `lds_lock`                         | `Lock` | LDS 상태 딕셔너리 |
| `status_lock`                      | `Lock` | system_status 딕셔너리 |
| `pcd_parse_semaphore`              | `Semaphore(2)` | 동시 PCD 파싱 최대 2개 제한 |

### 9.2 백그라운드 스레드 목록

| 스레드 | 메서드 | 라이프사이클 |
|--------|--------|-------------|
| ROS2 spin       | `spin()`              | `__init__` 시작 ~ `shutdown_requested` |
| 상태 체크       | `status_check_loop()` | `__init__` 시작 ~ `shutdown_requested` |
| 카메라 캡처 ×5  | `capture_loop(cam)`   | `__init__` 시작 ~ `shutdown_requested` |
| 비디오 녹화 ×5  | `record_camera()` 내부 | `start_recording` ~ `stop_recording` |
| 자동 정지 ×N    | `auto_stop()` 클로저  | `pc_save_start` 후 N초 |
| Detection 캡처  | `capture_with_gain_zero()` | 일회성 |
| Jetson 정리     | `_cleanup_old_pcd_on_jetson()` | 일회성 |

### 9.3 스레드 안전성 패턴

**카메라 프레임 읽기**:
```python
with self.cap_locks[cam_name]:
    cap = self.camera_states[cam_name]['cap']
    ret, frame = cap.read()
```

**LDS 상태 읽기**:
```python
with self.lds_lock:
    return dict(self.lds_state)  # 깊은 복사로 안전 반환
```

**PCD 파싱 동시성 제한**:
```python
with self.pcd_parse_semaphore:
    points = self.parse_point_cloud_file(filepath)
```

---

## 10. 데이터 흐름도

### 10.1 이미지 캡처 흐름
```
[브라우저: 저장 버튼]
      │ POST /img_save
      ▼
[Flask: route_img_save]
      │ iface.img_save(cam)
      ▼
[CameraManager: img_save]
      │ is_recording_images = True
      │ (대기 최대 2초)
      ▼
[capture_loop 스레드]
      │ ret, frame = cap.read()
      │ cv2.imwrite(path, frame)
      │ is_recording_images = False
      ▼
[img_save 반환]
      │ {message, file_path}
      ▼
[브라우저: 결과 표시]
```

### 10.2 PCL 수집 흐름 (방안 B: SCP)
```
[브라우저: 시작 버튼 + duration]
      │ POST /pc_save_start (cam, duration)
      ▼
[CameraManager: pc_save_start]
      │ /lidar_collect 서비스 호출 (start=True)
      ▼
[Jetson lidar_node]
      │ 포인트 누적 시작 (로컬)
      │
      │ ⏱ duration 초 후
      ▼
[auto_stop 타이머 스레드]
      │ pc_save_stop(cam) 자동 호출
      ▼
[CameraManager: pc_save_stop]
      │ /lidar_collect 서비스 호출 (start=False)
      │ ← "STOPPED:<path>:<count>"
      │ SCP로 Jetson → 서버PC 전송
      │ Jetson 오래된 PCD 정리 (백그라운드)
      ▼
[브라우저: 폴링으로 결과 수신]
```

### 10.3 LDS 거리 측정 흐름
```
[브라우저: 순간 측정 버튼]
      │ POST /lds/command {cmd: measure_once}
      ▼
[Flask: lds_command]
      │ iface.send_lds_command('measure_once')
      ▼
[ROS Publisher: perc/lds]
      │ {"cmd": "measure_once"}
      ▼
[LDS 노드]
      │ JRT U81 센서 측정
      ▼
[ROS Subscriber: perc/lds]
      │ {"type": "distance", "mm": 1234, ...}
      ▼
[lds_topic_callback]
      │ lds_state['last_distance'] = data
      │ distance_history.append(data)
      ▼
[브라우저: 1초마다 GET /lds/status]
      │ 화면 갱신
```

---

## 11. 실행 진입점

### 11.1 단일 인스턴스 생성
```python
iface = CameraManager()  # Line 1390 — 모듈 로드 시 즉시 실행
```
> **주의**: `CameraManager()`가 모듈 임포트 시점에 ROS2 초기화 + 5개 스레드 시작을 수행하므로, 모듈을 두 번 임포트해선 안됩니다.

### 11.2 `main()` 함수
```python
def main():
    base_port = 5030
    max_attempts = 5
    for i in range(max_attempts):
        port = base_port + i
        if not is_port_in_use(port):
            app.run(host='0.0.0.0', port=port, debug=False)
            return
    print("[Flask] ERROR: Could not find available port...")
```

### 11.3 ROS2 Launch 통합
launch 파일(`launch/sensor_cam_main.launch.py`):
```python
Node(
    package='sensor_cam_main',
    executable='mainwindow.py',
    output='screen',
    additional_env={'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}
)
```

### 11.4 실행 방법
```bash
# ROS2 워크스페이스 빌드 후
source install/setup.bash
ros2 launch sensor_cam_main sensor_cam_main.launch.py

# 또는 직접 실행
ros2 run sensor_cam_main mainwindow.py
```
이후 브라우저에서 `http://<server_ip>:5030` 접속.

---

## 12. 주요 특징 및 개선 포인트

### 12.1 강점

1. **견고한 카메라 재연결** — `try_reconnect_camera`로 네트워크 끊김에 자동 대응
2. **세분화된 락 구조** — 카메라마다 독립 락이라 데드락 위험 낮음
3. **이중 안전장치** — 폴더 감시 실패 시 fallback으로 최신 PCD 자동 사용
4. **자동 포트 탐색** — 5030~5034 범위 자동 선택으로 충돌 회피
5. **CPU 양보** — `time.sleep(0.001)`로 다른 스레드에 기회 부여
6. **상태 타임아웃** — 5초 동안 업데이트 없으면 자동 OFF 처리
7. **로그 이중화** — 파일 + 메모리 동시 보관

### 12.2 잠재적 문제점

| 항목 | 설명 |
|------|------|
| **모듈 사이드 이펙트** | `iface = CameraManager()`가 import 시 자동 실행 → 테스트 어려움 |
| **하드코딩된 경로** | `/root/sam_ws/...`가 코드에 박혀 있음 → 환경변수 사용 권장 |
| **HTML 인라인** | 2,000줄에 가까운 HTML이 Python 파일 안에 → 별도 템플릿 분리 권장 |
| **글로벌 객체 의존** | Flask 라우트가 `iface` 글로벌에 직접 의존 → DI 패턴 도입 가능 |
| **에러 처리 일관성** | 일부는 빈 dict 반환, 일부는 `False` 반환 → 표준화 필요 |
| **`pc_save_start_all` 시그니처** | `route_pc_save_start`의 `iface.pc_save_start_all(duration_sec=...)` 호출이 실제 시그니처(`durations` dict)와 불일치 (Line 2345) |

### 12.3 개선 제안

1. **CSS/JS 외부 파일 분리**
   ```python
   from flask import url_for
   # static/css/main.css, static/js/main.js
   ```
2. **환경 변수 기반 설정**
   ```python
   self.saveLogPath = os.environ.get('LOG_PATH', '/root/sam_ws/worklog/')
   ```
3. **로깅 모듈 사용**
   ```python
   import logging
   logger = logging.getLogger('sensor_cam_main')
   ```
4. **타입 힌트 추가**
   ```python
   def img_save(self, cam_name: str) -> dict:
   ```
5. **단위 테스트 작성**
   - `parse_point_cloud_file` 등 순수 함수부터 시작
6. **WebSocket 도입**
   - 1초 폴링 대신 서버푸시로 대역폭 절감

---

## 13. 부록: 클래스/함수 인덱스 (라인 번호 기준)

| 라인 | 식별자 | 종류 |
|------|--------|------|
| 28   | `save_pcd`                           | 함수 |
| 64   | `CameraManager`                      | 클래스 |
| 65   | `__init__`                           | 메서드 |
| 301  | `initialize_all_cameras`             | 메서드 |
| 331  | `capture_loop`                       | 메서드 |
| 386  | `try_reconnect_camera`               | 메서드 |
| 411  | `collect_latest_pcl_files`           | 메서드 (fallback) |
| 460  | `parse_point_cloud_file`             | 메서드 |
| 541  | `collect_and_save_single_pcl`        | 메서드 (file watcher) |
| 729  | `spin`                               | 메서드 (ROS2) |
| 736  | `call_lidar_collect_service`         | 메서드 |
| 764  | `camera_status_callback`             | 콜백 |
| 774  | `status_check_loop`                  | 메서드 |
| 787  | `lds_topic_callback`                 | 콜백 |
| 829  | `send_lds_command`                   | 메서드 |
| 837  | `get_lds_state`                      | 메서드 |
| 842  | `get_lds_history`                    | 메서드 |
| 847  | `report_callback`                    | 콜백 |
| 850  | `createFolder`                       | 메서드 |
| 861  | `writeLog`                           | 메서드 |
| 871  | `get_system_status`                  | 메서드 |
| 881  | `img_save`                           | 메서드 |
| 905  | `img_save_all`                       | 메서드 |
| 913  | `_convert_docker_path_to_host`       | 메서드 |
| 920  | `_fetch_pcd_via_scp`                 | 메서드 |
| 946  | `_cleanup_old_pcd_on_jetson`         | 메서드 |
| 965  | `pc_save_start`                      | 메서드 |
| 1010 | `pc_save_start_all`                  | 메서드 |
| 1051 | `pc_save_stop`                       | 메서드 |
| 1120 | `pc_save_stop_all`                   | 메서드 |
| 1187 | `get_pcl_status`                     | 메서드 |
| 1213 | `get_pcl_status_all`                 | 메서드 |
| 1219 | `detection`                          | 메서드 |
| 1263 | `detection_all`                      | 메서드 |
| 1269 | `open_cover`                         | 메서드 |
| 1283 | `close_cover_all`                    | 메서드 |
| 1294 | `get_cover_status`                   | 메서드 |
| 1300 | `start_recording`                    | 메서드 |
| 1362 | `start_recording_all`                | 메서드 |
| 1368 | `stop_recording`                     | 메서드 |
| 1384 | `stop_recording_all`                 | 메서드 |
| 1390 | `iface = CameraManager()`            | 전역 인스턴스 |
| 1392 | `HTML_TEMPLATE`                      | 상수 |
| 2312 | `index`                              | Flask 라우트 |
| 2319 | `serve_image`                        | Flask 라우트 |
| 2323~2440 | `system_status`, `get_logs`, `route_*`, `pcl_status`, `cover_status`, `lds_*`, `detection_images` | Flask 라우트들 |
| 2442 | `is_port_in_use`                     | 함수 |
| 2448 | `main`                               | 함수 |
| 2468 | `if __name__ == '__main__'`          | 진입점 |

---

> ⚙️ **mainwindow.py 분석 완료** — 이 문서는 `mainwindow.py` 코드의 정적 분석을 기반으로 작성되었습니다. 실행 환경(ROS2, 카메라/LiDAR 하드웨어, Jetson 배치)에 대한 통합 테스트는 별도 검증이 필요합니다.

---

# 📦 패키지 전체 파일 분석 (확장)

이하 섹션은 `mainwindow.py` 외에 패키지에 포함된 모든 파일들에 대한 추가 분석입니다. 이 패키지는 **두 개의 독립된 실행 모드**로 구성됩니다:

1. **운영 모드 (mainwindow.py)** — 실시간 5대 카메라 + LiDAR + LDS 통합 관리 GUI
2. **이미지 분석 모드 (detection.py)** — SAM + DepthAnythingV2 기반 단일 이미지의 3D 포인트클라우드 생성

---

## 14. `detection.py` — SAM + Depth 기반 3D 포인트클라우드 생성기

> **파일 경로**: `sensor_cam_main/sensor_cam_main/detection.py`
> **총 라인 수**: 413 줄
> **역할**: 단일 이미지 입력 → SAM 세그멘테이션 → DepthAnythingV2 깊이 추정 → 벽(wall) 영역 3D 포인트클라우드 생성
> **실행 방식**: CLI 스크립트 (`setup.py`의 `entry_points`에 `detection` 명령으로 등록)

### 14.1 파이프라인 개요

```
[입력 이미지 (.jpg 등)]
        │
        ▼
┌──────────────────────────────────┐
│ 1. 모델 로딩                       │
│    - SAM (vit_b)                 │
│    - DepthAnythingV2 (vitb)      │
└──────────────────────────────────┘
        │
        ▼
┌──────────────────────────────────┐
│ 2. 이미지 리사이즈                 │
│    max_dim = 512 (메모리 보호)     │
│    BGR → RGB 변환                  │
└──────────────────────────────────┘
        │
        ▼
┌──────────────────────────────────┐
│ 3. SAM Automatic Mask Generation │
│    - points_per_side=32          │
│    - pred_iou_thresh=0.88        │
│    - 다수의 마스크 생성              │
└──────────────────────────────────┘
        │
        ▼
┌──────────────────────────────────┐
│ 4. Depth 추정                     │
│    DepthAnythingV2 → 깊이 맵      │
└──────────────────────────────────┘
        │
        ▼
┌──────────────────────────────────┐
│ 5. 장비 부품(equipment part) 식별  │
│    - 가장자리 제외, 중앙 위치, 면적>2000│
│    - depth 표준편차 기반 마스크 정제 │
└──────────────────────────────────┘
        │
        ▼
┌──────────────────────────────────┐
│ 6. 가장 큰 부품 선택               │
│    sort by area (desc) → [0]     │
└──────────────────────────────────┘
        │
        ▼
┌──────────────────────────────────┐
│ 7. PointCloudGenerator (d2p.py)  │
│    - create_wall_point_cloud()   │
│    - 평면 RANSAC fit             │
│    - PLY + npy.gz 저장            │
└──────────────────────────────────┘
        │
        ▼
┌──────────────────────────────────┐
│ 8. 시각화 출력                     │
│    - 512×512 분석 이미지            │
│    - 원본 해상도 outline           │
│    - 3D scatter plot             │
└──────────────────────────────────┘
```

### 14.2 CLI 인자 (argparse)

```python
parser.add_argument('image_path', type=str)            # 필수: 입력 이미지 경로
parser.add_argument('--output_dir', default='./detection')
parser.add_argument('--device', default='cuda' if available else 'cpu')
```

**실행 예:**
```bash
ros2 run sensor_cam_main detection /path/to/image.jpg --output_dir ./detection
```

### 14.3 핵심 단계별 분석

#### (1) 모델 로딩 — Line 56~73
```python
share_dir = get_package_share_directory('sensor_cam_main')

# SAM
sam_checkpoint = os.path.join(share_dir, "models", "sam_vit_b_01ec64.pth")
sam_model = sam_model_registry["vit_b"](checkpoint=sam_checkpoint)
predictor = SamPredictor(sam_model)

# Depth Anything v2
depth_checkpoint = os.path.join(share_dir, "models", "depth_anything_v2_vitb.pth")
depth_model = load_depth_model(depth_checkpoint, device=device)
```
> ROS2의 `ament_index`를 통해 `share/sensor_cam_main/models/`에서 모델 가중치를 로딩합니다.

#### (2) 메모리 보호 리사이즈 — Line 96~110
```python
max_dim = 512
if max(oh, ow) > max_dim:
    scale = max_dim / float(max(oh, ow))
    new_h, new_w = int(oh * scale), int(ow * scale)
    image_for_sam = cv2.resize(image_rgb, (new_w, new_h))
```
> 4000×4000 이미지를 직접 처리하면 GPU OOM 발생 → 512로 강제 다운스케일.

#### (3) OOM 자동 복구 메커니즘 — Line 142~170
```python
try:
    masks = mask_generator.generate(image_for_sam)
except RuntimeError as e:
    if "CUDA out of memory" in str(e):
        # 384로 더 작게 리사이즈, points_per_side=16으로 축소
        torch.cuda.empty_cache()
        ...
```
> 첫 시도가 OOM이면 자동으로 더 작은 해상도/단순 설정으로 재시도합니다.

#### (4) `is_equipment_part(mask, image_center, depth_map)` 함수 — Line 175~209

마스크가 "장비 부품"인지 판단하는 휴리스틱:

| 조건 | 기준 |
|------|------|
| **가장자리 제외**  | bbox가 이미지 가장자리 5% 안쪽에 있어야 함 |
| **중앙 근처**      | 중심 거리가 `min(W,H) * 0.5` 이내 |
| **충분한 면적**    | 픽셀 수 > 2000 |

세 조건 모두 만족하면 부품으로 채택.

#### (5) Depth 기반 마스크 정제 — Line 258~272
```python
mean_depth = np.mean(valid_depths)
std_depth = np.std(valid_depths)
depth_threshold = 1.0 * std_depth
refined_mask = np.abs(full_depth_map - mean_depth) < depth_threshold
refined_mask = refined_mask & selected_mask

if np.sum(refined_mask) > 0.7 * np.sum(selected_mask):
    selected_mask = refined_mask  # 70% 이상 유지되면 정제 마스크 채택
```
> 평균 깊이 ± 1σ 범위 안에 있는 픽셀만 남겨 잡음 제거.

### 14.4 출력 결과물 (`<output_dir>/<image_name>/`)

| 파일 | 설명 |
|------|------|
| `equipment_parts_debug_512.jpg`        | 모든 부품 bbox + 라벨 시각화 |
| `equipment_parts_visualization_512.jpg`| 마스크 컬러 오버레이 |
| `<name>_analysis_512.png`               | 4분할 분석 (선택부품/마스크/depth/masked-depth) |
| `<name>_outline_original.jpg`           | 원본 해상도 이미지에 contour |
| `<name>_pointcloud_preview.png`         | 포인트클라우드 프리뷰 |
| `<name>_wall.ply`                       | Wall 포인트클라우드 (Open3D PLY) |
| `<name>_wall.npy.gz`                    | 구조화 numpy 배열 (gzip) |
| `<name>_included_pixels.png`            | wall_mask 적용된 픽셀 |
| `<name>_final_visualization_3d.png`     | 3D scatter plot |

---

## 15. `d2p.py` — `PointCloudGenerator` 클래스 (Depth-to-Point Cloud)

> **파일 경로**: `sensor_cam_main/sensor_cam_main/d2p.py`
> **총 라인 수**: 532 줄
> **역할**: 깊이 맵 + 카메라 캘리브레이션 → 3D 포인트 클라우드 변환
> **클래스**: `PointCloudGenerator`

### 15.1 클래스 초기화

```python
def __init__(self, save_path=None, camera_matrix=None, dist_coeffs=None):
    self.save_path = save_path or './detection/'
    os.makedirs(self.save_path, exist_ok=True)

    share_dir = get_package_share_directory('sensor_cam_main')
    npz_path  = os.path.join(share_dir, 'calibration', 'chessboard_calibration.npz')
    yaml_path = os.path.join(share_dir, 'calibration', 'Calibration_result_d455.yaml')

    self.calibrator = CameraCalibrator()
    self.calibrator.load_calibration(npz_path)
    self.camera_params = self.calibrator.load_camera_params(yaml_path)
```

**기본 매개변수:**
| 변수 | 값 | 의미 |
|------|----|------|
| `scale_factor`   | 2.0    | 일반 스케일 |
| `min_depth`      | 0.1 m  | 최소 깊이 |
| `max_depth`      | 5.0 m  | 최대 깊이 |
| `z_offset`       | 2.0 m  | Z축 오프셋 (카메라 위치 보정) |
| `y_offset`       | 0.7 m  | Y축 오프셋 |

### 15.2 메서드 일람

| 메서드 | 라인 | 설명 |
|--------|------|------|
| `__init__`                                    | 23   | save_path + 캘리브레이션 로드 |
| `calculate_view_angle(image_path)`            | 66   | 체스보드로부터 카메라 pitch 추출 |
| `create_point_cloud(...)`                     | 91   | 일반 포인트클라우드 생성 |
| `calculate_camera_angle_from_intrinsics(...)` | 233  | Intrinsics 기반 시야각/틸트 추정 |
| `load_camera_params(yaml_file)`               | 272  | YAML/NPY fallback으로 카메라 파라미터 로드 |
| `create_wall_point_cloud(...)`                | 302  | 벽 마스크 적용 + RANSAC 평면 fit |
| `visualize_point_cloud(pcd, output_path)`     | 473  | Open3D 오프스크린 렌더링 |
| `calculate_scale_factor(...)`                 | 494  | 실거리 기반 스케일 계산 |
| `depth_correction(depth_map, focal_length)`   | 512  | 광학 왜곡 반경 보정 |

### 15.3 `create_point_cloud()` 핵심 알고리즘 — Line 91~231

**스케일 팩터 결정 로직:**
```python
scale_factor_z = 100  # 기본값
if real_distance > 0:
    center_depth = np.median(depth_map[H/3:2H/3, W/3:2W/3])
    computed_scale = real_distance / center_depth
    if 10 < computed_scale < 1000:
        scale_factor_z = computed_scale
```

**포인트 변환 공식 (틸트 보정 포함):**
```python
z_scaled  = depth / scale_factor_z          # 깊이 정규화
z_adjusted = z_scaled + z_offset            # 카메라 위치 보정

x_adjusted = (u - W/2) / scale_factor_x     # 픽셀 → 미터 (수평)
y_adjusted = (v / scale_factor_y - y_offset) * cos(tilt)
```

**적응형 깊이 범위 결정:**
- `adaptive_min = max(0.001, percentile(z, 1))`
- `adaptive_max = min(20.0, percentile(z, 99))`
> 1~99 백분위로 이상치 제거.

### 15.4 `create_wall_point_cloud()` — Line 302~471

벽 영역에 특화된 포인트클라우드 생성:

1. **마스크 + 깊이맵 크기 정합** — 원본 해상도로 리사이즈
2. **카메라 각도 계산** — `calculate_camera_angle_from_intrinsics`
3. **Intrinsics 기반 스케일**:
   ```python
   scale_factor_x = fx / 3.5
   scale_factor_y = fy / 5.0
   ```
4. **포인트 변환 (틸트 보정)**:
   ```python
   z_corrected = z_scaled * cos(tilt)
   y_adjusted  = y_adjusted * cos(tilt) - y_offset
   ```
5. **후처리**:
   - `estimate_normals()` — KDTree 기반 법선 추정
   - `remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)` — 통계적 이상치 제거
   - `segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)` — RANSAC 평면 검출
6. **벽 방향 계산**:
   ```python
   wall_angle = arccos(plane_normal · [0,0,1])
   ```

### 15.5 출력 데이터 구조 (NPY 구조화 배열)

```python
dtype = [
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
    ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
    ('normal_x', '<f4'), ('normal_y', '<f4'), ('normal_z', '<f4')
]
```
> Wall 모드는 법선 벡터까지 포함, 일반 모드는 위치+색상만.

---

## 16. `cam_cal.py` — `CameraCalibrator` 클래스

> **파일 경로**: `sensor_cam_main/sensor_cam_main/cam_cal.py`
> **총 라인 수**: 351 줄
> **역할**: 체스보드/ChArUco 기반 카메라 내·외부 파라미터 캘리브레이션
> **클래스**: `CameraCalibrator`

### 16.1 인스턴스 변수

```python
self.camera_matrix     = None  # 3x3 K matrix
self.dist_coeffs       = None  # [k1, k2, p1, p2, k3]
self.calibration_error = None  # 평균 재투영 오차
```

### 16.2 메서드 일람

| 메서드 | 라인 | 설명 |
|--------|------|------|
| `calibrate_with_chessboard(image_paths, board_size=(4,6), square_size=32.0)` | 15  | 체스보드 캘리브레이션 |
| `calibrate_with_charuco(image_paths, ...)`                                    | 100 | ChArUco 보드 캘리브레이션 |
| `save_calibration(filepath)`                                                   | 177 | npz로 저장 |
| `load_calibration(filepath)`                                                   | 190 | npz에서 로드 |
| `measure_board_distance(image, ...)`                                          | 208 | 체스보드까지 거리 측정 + 시각화 |
| `measure_board_pose(image, ...)`                                              | 285 | 체스보드 회전/이동 벡터 |
| `load_camera_params(yaml_path)`                                               | 318 | YAML에서 fx, fy, cx, cy 로드 |

### 16.3 체스보드 캘리브레이션 흐름

```python
# 1. 3D 좌표 생성 (보드의 격자점)
objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:bw, 0:bh].T.reshape(-1, 2)
objp = objp * square_size  # mm 단위

# 2. 코너 검출
ret, corners = cv2.findChessboardCorners(
    gray, board_size,
    flags=ADAPTIVE_THRESH + NORMALIZE_IMAGE + FAST_CHECK
)

# 3. 서브픽셀 정밀화
corners2 = cv2.cornerSubPix(
    gray, corners, (11, 11), (-1, -1),
    (TERM_CRITERIA_EPS + MAX_ITER, 30, 0.001)
)

# 4. calibrateCamera 호출
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# 5. 재투영 오차 계산
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
```

### 16.4 ChArUco 캘리브레이션 — Line 100~175

```python
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard(
    (squares_x, squares_y),    # 5x7 사각형
    float(square_length),      # 32mm
    float(marker_length),      # 24mm
    aruco_dict
)
detector = cv2.aruco.ArucoDetector(aruco_dict)
marker_corners, marker_ids, _ = detector.detectMarkers(gray)
```
> ChArUco는 체스보드 + ArUco 마커 결합으로 부분 검출 시에도 정확도 유지.

### 16.5 거리 측정 (`measure_board_distance`) — Line 208~284

```python
ret, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, dist_coeffs)
distance = np.linalg.norm(tvec)  # mm 단위
```
- 좌표축 시각화: X(빨강), Y(초록), Z(파랑)
- 화면에 `Distance: XXX.Xmm` 표시

### 16.6 YAML 파라미터 파서 — Line 318~336

YAML 파일 포맷이 단순 CSV 문자열이므로 직접 파싱:
```python
camera_matrix = [float(x) for x in calib_data['CameraMatrix'].split(',')]
distortion_coeffs = [float(x) for x in calib_data['DistortionCoefficient'].split(',')]
```

반환 키: `focal_length_x`, `focal_length_y`, `center_x`, `center_y`, `distortion_coeffs`

> ⚠️ **주의**: `d2p.py`의 `load_camera_params`는 다른 키 이름(`fx`, `fy`, `cx`, `cy`)을 기대합니다 → 키 불일치는 잠재 버그입니다.

---

## 17. `depth_utils.py` — DepthAnythingV2 모델 유틸리티

> **파일 경로**: `sensor_cam_main/sensor_cam_main/depth_utils.py`
> **총 라인 수**: 49 줄
> **역할**: DepthAnythingV2 모델 로딩 + 깊이 추론 헬퍼

### 17.1 함수 분석

#### `load_depth_model(encoder, device='cuda')` — Line 9~41

**모델 컨피그 매핑:**
| 인코더 | features | out_channels |
|--------|----------|--------------|
| `vits` | 64       | [48, 96, 192, 384] |
| `vitb` | 128      | [96, 192, 384, 768] |
| `vitl` | 256      | [256, 512, 1024, 1024] |
| `vitg` | 384      | [1536, 1536, 1536, 1536] |

> 코드는 `vitb`(ViT-Base) 설정을 하드코딩으로 사용합니다.

**디바이스 자동 선택:**
```python
DEVICE = 'cuda' if torch.cuda.is_available() else \
         'mps'  if torch.backends.mps.is_available() else 'cpu'
```
> Mac M1/M2의 MPS 백엔드도 지원.

**전처리 파이프라인:**
```python
transform = transforms.Compose([
    Resize(width=518, height=518,
           keep_aspect_ratio=True,
           ensure_multiple_of=14,
           resize_method="minimal",
           image_interpolation_method=cv2.INTER_CUBIC),
    NormalizeImage(mean=[0.485, 0.456, 0.406],
                   std=[0.229, 0.224, 0.225]),  # ImageNet 통계
    PrepareForNet(),
])
```
> ViT는 patch_size=14를 사용하므로 518×518은 37×37 패치가 됩니다.

#### `infer_depth(model, image)` — Line 43~49

```python
with torch.no_grad():
    depth = model.infer_image(image, input_size=518)
    depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
    depth = depth.astype(np.uint8)
```
> 깊이를 0-255 범위로 정규화 후 uint8 반환 (시각화 용이).

> ⚠️ 매개변수 `encoder`는 실제로는 가중치 파일 경로(`*.pth`)를 의미합니다. 변수명이 오해를 유발할 수 있습니다.

---

## 18. `__init__.py` — 패키지 초기화

> **파일 경로**: `sensor_cam_main/sensor_cam_main/__init__.py`
> **총 라인 수**: 18 줄

```python
import rclpy
import logging

logger = logging.getLogger('sensor_cam_main')
logger.setLevel(logging.INFO)
```

### 18.1 역할
- 패키지 전역 로거 생성
- ROS2 클라이언트 라이브러리 import (실제 init은 각 노드에서)
- 향후 공용 상수(예: `LIDAR_TOPIC`)를 정의할 수 있도록 자리잡기

> 현재는 사실상 **빈 초기화 파일**에 가깝습니다.

---

## 19. `setup.py` — Python 패키지 설치 스크립트

> **파일 경로**: `sensor_cam_main/setup.py`
> **총 라인 수**: 38 줄

### 19.1 핵심 설정

```python
setup(
    name='sensor_cam_main',
    version='0.0.1',
    packages=['sensor_cam_main'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/sensor_cam_main']),
        ('share/sensor_cam_main', ['package.xml']),
        ('share/sensor_cam_main/models',      glob('models/*.pth')),
        ('share/sensor_cam_main/calibration', glob('calibration/*')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'detection = sensor_cam_main.detection:main',
        ],
    },
)
```

### 19.2 분석 포인트

| 항목 | 설명 |
|------|------|
| `data_files` | ROS2 ament 인덱스 등록 + 모델/캘리브레이션 파일 share 디렉토리에 설치 |
| `entry_points` | `detection` CLI 명령으로 `sensor_cam_main.detection:main` 노출 |
| `install_requires` | 최소한의 setuptools만 명시 (실제 의존성은 `package.xml`에서 관리) |

> ⚠️ `setup.py`는 `detection` 진입점만 등록되어 있고, `mainwindow.py`의 진입점은 `CMakeLists.txt`의 `install(PROGRAMS ...)`로 설치됩니다 — **build 시스템 이중 구조**.

---

## 20. `CMakeLists.txt` — ROS2 ament_cmake 빌드 스크립트

> **파일 경로**: `sensor_cam_main/CMakeLists.txt`
> **총 라인 수**: 49 줄

### 20.1 빌드 구성

```cmake
cmake_minimum_required(VERSION 3.8)
project(sensor_cam_main)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
```

### 20.2 의존성 패키지

| `find_package` | 용도 |
|----------------|------|
| `ament_cmake`         | ROS2 빌드 시스템 |
| `rclcpp`              | ROS2 C++ 클라이언트 (현재 미사용, 향후 확장 대비) |
| `sensor_msgs`         | Image, PointCloud2 메시지 |
| `geometry_msgs`       | Pose, Twist 등 |
| `cv_bridge`           | OpenCV ↔ ROS Image 변환 |
| `image_transport`     | 이미지 토픽 압축 전송 |
| `OpenCV` (core/highgui/imgproc/videoio) | 영상 처리 |
| `pcl_conversions`     | PCD ↔ ROS PointCloud2 |
| `ament_cmake_python`  | Python 노드 빌드 지원 |

### 20.3 설치 규칙

```cmake
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  sensor_cam_main/mainwindow.py
  sensor_cam_main/detection.py
  sensor_cam_main/d2p.py
  sensor_cam_main/cam_cal.py
  sensor_cam_main/depth_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/      DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY models/      DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY calibration/ DESTINATION share/${PROJECT_NAME}/calibration)
```

### 20.4 분석 포인트

- **하이브리드 빌드**: `ament_cmake` (CMake) 기반이지만 Python 패키지도 동시에 설치
- **C++ 의존성 미사용**: `rclcpp`/`sensor_msgs` 등이 선언되어 있으나 실제 코드는 모두 Python → 향후 C++ 노드 추가 가능성 시사
- **OpenCV 컴포넌트 명시**: `core, highgui, imgproc, videoio`만 링크 (메모리 절약)

---

## 21. `package.xml` — ROS2 패키지 매니페스트

> **파일 경로**: `sensor_cam_main/package.xml`
> **총 라인 수**: 32 줄

### 21.1 메타데이터

```xml
<package format="3">
  <name>sensor_cam_main</name>
  <version>0.1.0</version>
  <description>
    ROS2 package for camera-LiDAR fusion with YOLO v11 object detection
    using Python and C++ nodes.
  </description>
  <maintainer email="jyp7781@ff00ff.kr">jyp7781</maintainer>
  <license>Apache-2.0</license>
```

### 21.2 의존성 분류

| 범주 | 패키지 |
|------|--------|
| **buildtool_depend** | `ament_cmake`, `ament_cmake_python` |
| **depend** (build+runtime) | `rclcpp`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, `image_transport`, `pcl_conversions`, `pcl_msgs` |
| **exec_depend** (runtime only) | `rclpy`, `python3-onnxruntime`, `python3-opencv`, `python3-pyqt5` |

> **참고**: `python3-pyqt5`가 의존성에 있지만 코드에서 PyQt는 사용하지 않습니다(Flask로 전환된 흔적). `python3-onnxruntime`도 마찬가지로 직접 사용은 보이지 않으나 PyTorch 모델의 ONNX 변환을 대비한 것으로 추정됩니다.

### 21.3 격차 (Gap) 분석

| 명시된 의존성 | 실제 사용 | 비고 |
|--------------|-----------|------|
| `rclcpp`              | ❌ | C++ 미사용 (향후 확장 대비) |
| `sensor_msgs`         | ❌ | 메시지 미사용 |
| `geometry_msgs`       | ❌ | 메시지 미사용 |
| `cv_bridge`           | ❌ | 코드에서 사용 안 됨 |
| `image_transport`     | ❌ | 코드에서 사용 안 됨 |
| `pcl_conversions`     | ❌ | Python에서는 직접 사용 안 함 |
| `python3-pyqt5`       | ❌ | Flask로 대체됨 |
| `python3-onnxruntime` | ❌ | 이번 코드 기준 직접 사용 없음 |

> 향후 정리 시 위 의존성들을 제거하거나, 실제 사용하는 `python3-numpy`, `python3-flask`, `python3-torch` 등을 명시해야 합니다.

---

## 22. `launch/sensor_cam_main.launch.py` — ROS2 Launch 스크립트

> **파일 경로**: `sensor_cam_main/launch/sensor_cam_main.launch.py`
> **총 라인 수**: 7 줄

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_cam_main',
            executable='mainwindow.py',
            output='screen',
            additional_env={'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}
        )
    ])
```

### 22.1 분석

| 옵션 | 의미 |
|------|------|
| `package='sensor_cam_main'`     | 실행할 ROS2 패키지명 |
| `executable='mainwindow.py'`    | `lib/sensor_cam_main/mainwindow.py`를 실행 |
| `output='screen'`               | 표준출력을 터미널에 표시 |
| `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` | DDS 미들웨어를 **eProsima Fast DDS**로 강제 |

> Cyclone DDS와의 호환성 이슈를 회피하기 위해 명시적으로 Fast DDS를 사용합니다.

### 22.2 실행
```bash
ros2 launch sensor_cam_main sensor_cam_main.launch.py
```

---

## 23. `calibration/Calibration_result_d455.yaml` — 카메라 캘리브레이션 데이터

> **파일 경로**: `sensor_cam_main/calibration/Calibration_result_d455.yaml`
> **카메라 모델**: Intel RealSense D455 (이름 추정)

### 23.1 파일 내용 분석

```yaml
CameraMatrix: 636.3392652788157, 0.0, 654.3418233071645, 0.0, 636.4266464742717, 399.58963414918554, 0.0, 0.0, 1.0
DistortionCoefficient: -0.0674558491814021, 0.07674552916772615, -0.002874904836627887, 0.0011617524486503162, -0.029358499256325468
```

### 23.2 카메라 행렬(K matrix) 해석

3×3 형태로 재구성:
```
K = | 636.339   0.000   654.342 |
    |   0.000 636.427   399.590 |
    |   0.000   0.000     1.000 |
```

| 파라미터 | 값 | 의미 |
|---------|-----|------|
| `fx` | 636.339 | 수평 초점거리(픽셀) |
| `fy` | 636.427 | 수직 초점거리(픽셀) |
| `cx` | 654.342 | 주점 X 좌표 |
| `cy` | 399.590 | 주점 Y 좌표 |

### 23.3 왜곡 계수

```
[k1, k2, p1, p2, k3] = [-0.0675, 0.0767, -0.0029, 0.0012, -0.0294]
```

| 계수 | 의미 |
|------|------|
| `k1, k2, k3` | 반경 방향 왜곡 (radial distortion) |
| `p1, p2`     | 접선 방향 왜곡 (tangential distortion) |

### 23.4 활용

`d2p.py`의 `PointCloudGenerator`가 이 파일을 로드해 X/Y 좌표 변환에 사용합니다.

> ⚠️ `cam_cal.py`의 `load_camera_params`는 키를 `focal_length_x` 등으로 반환하지만, `d2p.py`는 `fx, fy, cx, cy`로 접근 → **키 불일치 잠재 버그** (앞서 16.6에서 지적).

---

## 24. `calibration/chessboard_calibration.npz` — 캘리브레이션 캐시

> **파일 형식**: NumPy 압축 아카이브
> **저장 키**: `camera_matrix`, `dist_coeffs`, `calibration_error`
> **생성처**: `cam_cal.py`의 `save_calibration()` (체스보드 캘리브레이션 결과)
> **로드처**: `d2p.py` 초기화 시 `CameraCalibrator.load_calibration(npz_path)`

> 바이너리 파일이므로 정적 분석은 불가하며, 위 키가 들어있다고 가정하고 사용됩니다.

---

## 25. 패키지 전체 파일 의존 관계

```
                            ┌───────────────────┐
                            │   package.xml     │
                            │   CMakeLists.txt  │ ← ROS2 빌드 시스템
                            │   setup.py        │
                            └─────────┬─────────┘
                                      │ 빌드/설치
                                      ▼
       ┌─────────────────────────────────────────────────────┐
       │                                                     │
       ▼                                                     ▼
┌─────────────────────────┐                  ┌──────────────────────────┐
│   mainwindow.py         │                  │     detection.py         │
│   (ROS2 + Flask 운영)    │                  │     (CLI 분석 도구)        │
│                         │                  │                          │
│   - CameraManager       │                  │   imports:               │
│   - 5 cameras           │                  │   - depth_utils.py ─────┐│
│   - LiDAR / LDS         │                  │   - d2p.py        ──────┼┤
│   - Web UI              │                  │   - SAM            (외부)││
└─────────────────────────┘                  │   - DepthAnythingV2(외부)││
                                             └──────────────────────┬──┘│
                                                                    │   │
                                                                    ▼   ▼
                                                ┌────────────────────────────────┐
                                                │     d2p.py (PointCloudGen.)    │
                                                │       imports:                 │
                                                │       - cam_cal.py ────────┐    │
                                                └────────────────────────────┼───┘
                                                                             │
                                                                             ▼
                                                ┌────────────────────────────────┐
                                                │       cam_cal.py               │
                                                │       (CameraCalibrator)       │
                                                │       reads:                   │
                                                │       - chessboard_calibration.npz│
                                                │       - Calibration_result_d455.yaml│
                                                └────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│   launch/sensor_cam_main.launch.py                                          │
│   → mainwindow.py 실행 (additional_env: RMW_IMPLEMENTATION=fastrtps)         │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 26. 두 실행 모드 비교 요약

| 항목 | **운영 모드** | **이미지 분석 모드** |
|------|---------------|---------------------|
| 진입점 | `mainwindow.py` (Launch 파일) | `detection` (entry_points) |
| 입력  | 실시간 카메라 + LiDAR | 단일 이미지 파일 |
| 출력  | 이미지/비디오/PCD 파일 + 웹 UI | 분석 이미지 + 3D 포인트클라우드 |
| 의존 모델 | 없음 (전송/저장 위주) | SAM (vit_b) + DepthAnythingV2 (vitb) |
| GUI    | Flask 웹 인터페이스 (5030+) | matplotlib 정적 이미지 + Open3D 시각화 |
| 동시성 | 다중 스레드 (5+ 카메라 + ROS2) | 단일 스레드 (CLI) |
| 캘리브레이션 | 사용 안 함 | `cam_cal.py` + YAML/NPZ |

---

## 27. 전체 파일 인덱스 및 라인 수 요약

| 파일 | 라인 수 | 종류 | 핵심 역할 |
|------|--------|------|----------|
| `sensor_cam_main/mainwindow.py`   | 2,468 | Python | 운영 GUI (ROS2 + Flask) |
| `sensor_cam_main/d2p.py`          | 532  | Python | PointCloudGenerator |
| `sensor_cam_main/detection.py`    | 413  | Python | SAM+Depth 분석 파이프라인 |
| `sensor_cam_main/cam_cal.py`      | 351  | Python | CameraCalibrator |
| `sensor_cam_main/depth_utils.py`  | 49   | Python | DepthAnythingV2 헬퍼 |
| `sensor_cam_main/__init__.py`     | 18   | Python | 패키지 초기화 |
| `sensor_cam_main/sd.css`          | 0    | CSS    | (빈 파일) |
| `setup.py`                        | 38   | Python | Python 패키지 정의 |
| `CMakeLists.txt`                  | 49   | CMake  | ROS2 빌드 설정 |
| `package.xml`                     | 32   | XML    | ROS2 매니페스트 |
| `launch/sensor_cam_main.launch.py`| 7    | Python | ROS2 launch 정의 |
| `calibration/Calibration_result_d455.yaml` | 2 | YAML | RealSense D455 캘리브레이션 |
| `calibration/chessboard_calibration.npz`   | (binary) | NumPy | 체스보드 캘리브레이션 캐시 |

**Python 코드 총합**: 약 **3,876 줄**

---

## 28. 통합 시스템 개선 권고사항

### 28.1 일관성 문제

1. **카메라 파라미터 키 불일치**
   - `cam_cal.py`: `focal_length_x, focal_length_y, center_x, center_y`
   - `d2p.py`:    `fx, fy, cx, cy`
   - **해결**: 표준 키로 통일 (예: `fx, fy, cx, cy`)

2. **불필요한 의존성**
   - `package.xml`: rclcpp, sensor_msgs, cv_bridge, image_transport, pyqt5, onnxruntime 등 미사용
   - **해결**: 정리 후 실제 사용 패키지만 명시

3. **이중 빌드 시스템**
   - `setup.py`는 `detection` 진입점만, `CMakeLists.txt`는 모든 Python 파일을 PROGRAMS로 설치
   - **해결**: `ament_python` 단일 빌드 시스템으로 단일화 (또는 `ament_cmake_python` 명확히 분리)

### 28.2 코드 품질

| 영역 | 권고 |
|------|------|
| **타입 힌트** | 모든 public 메서드에 `-> ReturnType` 추가 |
| **에러 처리** | `try/except`에서 `pass`나 `bare except` 제거 |
| **상수화** | `192.168.3.150~154` IP, 경로 등을 환경변수/설정파일로 |
| **로깅** | `print()` → `logger.info()` 표준화 |
| **테스트** | `parse_point_cloud_file`, `is_equipment_part` 등 순수 함수부터 단위 테스트 |
| **HTML 분리** | 2,000줄 인라인 HTML → `templates/index.html` |

### 28.3 성능

1. **PCD 파싱 병렬화**
   - 현재 세마포어(2)로 제한 — CPU 코어 수에 맞게 조정
2. **이미지 캡처 폴링 → 이벤트 기반**
   - 0.05초 sleep 루프 → 콜백/큐 기반으로 전환
3. **Flask 폴링 → WebSocket**
   - 1초 폴링이 5개 엔드포인트에 동시 발생 → SSE/WebSocket 도입

### 28.4 보안

1. **SCP/SSH 키 관리** — `StrictHostKeyChecking=no` 사용 중 → 정식 known_hosts 등록 권장
2. **Flask 포트 노출** — `host='0.0.0.0'` → 내부망 한정 또는 인증 추가
3. **하드코딩된 IP** — 외부에 노출 시 위험 → 환경변수화

---

> ⚙️ **전체 패키지 분석 완료** — 이 문서는 `sensor_cam_main` ROS2 패키지의 모든 소스/빌드/캘리브레이션 파일을 정적 분석한 결과입니다. 운영 모드(`mainwindow.py`)와 분석 모드(`detection.py`)는 모델 파일(`*.pth`)과 캘리브레이션 데이터(`.npz`/`.yaml`)를 매개로 통합되어 있으며, ROS2 ament 빌드 시스템으로 함께 배포됩니다.
