# nimg_v3 코드 종합 분석 문서

> **작성일**: 2026-05-06
> **분석 대상**: `nimg_v3` (FoundationPose 기반 6DoF 자세 추정 시스템) v3.0.0
> **작성자**: FurSys AI Team

---

## 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [전체 아키텍처](#2-전체-아키텍처)
3. [디렉토리 구조](#3-디렉토리-구조)
4. [핵심 패키지 (`nimg_v3/`)](#4-핵심-패키지-nimg_v3)
   - [4.1 config 모듈](#41-config-모듈---시스템-설정)
   - [4.2 input 모듈](#42-input-모듈---데이터-로딩)
   - [4.3 measurement 모듈](#43-measurement-모듈---자세-변환--칼만-필터)
   - [4.4 pose 모듈](#44-pose-모듈---6dof-자세-추정)
   - [4.5 pointcloud 모듈](#45-pointcloud-모듈---3d-포인트클라우드)
   - [4.6 output 모듈](#46-output-모듈---결과-내보내기)
   - [4.7 detection / tracking / utils 모듈](#47-detection--tracking--utils-모듈)
   - [4.8 main.py - 통합 측정 시스템](#48-mainpy---통합-측정-시스템)
5. [scripts 폴더](#5-scripts-폴더)
6. [tests 폴더](#6-tests-폴더)
7. [models 및 설정](#7-models-및-설정)
8. [데이터 흐름 (Data Flow)](#8-데이터-흐름-data-flow)
9. [핵심 설계 원칙](#9-핵심-설계-원칙)
10. [사용 시나리오](#10-사용-시나리오)
11. [의존성 및 설치](#11-의존성-및-설치)

---

## 1. 프로젝트 개요

### 1.1 목적

`nimg_v3`는 **NVIDIA FoundationPose**를 기반으로 한 **6DoF (6 Degrees of Freedom) 자세 추정 시스템**입니다. RGB-D 카메라(주로 Intel RealSense D455)에서 입력받은 이미지로부터 객체의 위치(x, y, z)와 회전(roll, pitch, yaw)을 실시간으로 추정합니다.

### 1.2 주요 특징

| 특징 | 설명 |
|------|------|
| **FoundationPose 통합** | NVIDIA의 SOTA 자세 추정 모델 활용 |
| **Model-Free 지원** | CAD 모델 없이 약 16개의 참조 이미지만으로 작동 |
| **Zero-Shot 일반화** | 학습되지 않은 새 객체에도 적용 가능 (Domain Shift 해결) |
| **하이브리드 회전 표현** | 내부는 쿼터니언(안정성), 외부는 오일러(직관성), ROS는 쿼터니언(표준) |
| **Kalman Filter 통합** | 12/13-상태 칼만 필터로 속도/각속도 추정 |
| **YOLO 통합** | 187 클래스의 도장 공정 객체를 자동 탐지 |
| **ROS2 호환** | 포인트 클라우드 및 신호 퍼블리시 (Foxy) |

### 1.3 적용 분야

- 산업 자동화 (도장 공정 등)
- 로봇 비전 (Pick & Place)
- 3D 객체 추적 및 측정
- 실시간 RGBD 스트리밍 분석

---

## 2. 전체 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│                        nimg_v3 시스템                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ┌─────────┐    ┌──────────┐    ┌──────────────┐    ┌────────┐ │
│   │ RGB+    │ -> │  YOLO    │ -> │ FoundationPose│ -> │ Kalman │ │
│   │ Depth   │    │ Detection│    │ (6DoF Pose)   │    │ Filter │ │
│   └─────────┘    └──────────┘    └──────────────┘    └────┬───┘ │
│                        │                  │                │    │
│                        ▼                  ▼                ▼    │
│                  ┌──────────┐    ┌──────────────┐    ┌────────┐ │
│                  │ BBox/Mask│    │  4x4 Pose    │    │ Speed/ │ │
│                  │          │    │  Matrix      │    │AngVel  │ │
│                  └──────────┘    └──────────────┘    └────────┘ │
│                                          │                      │
│                                          ▼                      │
│                              ┌────────────────────────┐         │
│                              │  Pose Converter        │         │
│                              │  (Euler/Quat/Matrix)   │         │
│                              └────────────────────────┘         │
│                                          │                      │
│                                          ▼                      │
│                              ┌────────────────────────┐         │
│                              │  Result Exporter       │         │
│                              │  (CSV / JSON / ROS2)   │         │
│                              └────────────────────────┘         │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. 디렉토리 구조

```
nimg_v3/
├── docs/                                  # 문서
│   ├── NIMG_V3_STRUCTURE_ANALYSIS.md      # 구조 분석 문서
│   ├── pointcloud_interface_guide.md      # 포인트클라우드 GUI 가이드
│   └── CODE_ANALYSIS.md                   # 본 문서
│
├── models/                                # 학습된 모델 저장소
│   ├── foundationpose/                    # FoundationPose 가중치 (외부 다운로드 필요)
│   │   ├── refiner/config.yml             # 정제 네트워크 설정
│   │   └── scorer/config.yml              # 점수 네트워크 설정
│   ├── neural_fields/                     # 학습된 Neural Object Field
│   │   ├── housing_M/reference_images/    # 부품 1 참조 이미지
│   │   └── Wiring_tray/reference_images/  # 부품 2 참조 이미지
│   └── yolo/                              # 파인튜닝된 YOLO 모델
│       └── class187_image85286_v12x_250epochs.pt   # 114MB
│
├── nimg_v3/                               # 코어 Python 패키지
│   ├── __init__.py                        # 패키지 초기화
│   ├── main.py                            # 통합 측정 시스템
│   ├── config/                            # 설정 관리
│   │   ├── system_config.py
│   │   └── default_config.yaml
│   ├── detection/                         # 객체 탐지 (YOLO)
│   ├── input/                             # 데이터 입력/로드
│   │   └── data_loader.py
│   ├── measurement/                       # 측정/변환
│   │   ├── pose_converter.py
│   │   └── pose_kalman_filter.py
│   ├── output/                            # 결과 출력
│   │   └── result_exporter.py
│   ├── pose/                              # 자세 추정
│   │   ├── foundationpose_estimator.py
│   │   ├── foundationpose_wrapper.py
│   │   ├── real_foundationpose.py
│   │   ├── neural_object_field.py
│   │   └── reference_image_loader.py
│   ├── pointcloud/                        # 3D 포인트클라우드
│   │   ├── camera_wrapper.py
│   │   ├── capture_manager.py
│   │   ├── visualizer.py
│   │   ├── ros2_publisher.py
│   │   └── gui.py
│   ├── tracking/                          # 추적 (FoundationPose 내장)
│   └── utils/                             # 유틸리티
│
├── scripts/                               # 실행 스크립트
│   ├── train_neural_field.py              # Neural Field 학습
│   ├── run_measurement.py                 # 측정 실행
│   ├── test_pipeline.py                   # 기본 테스트
│   ├── test_pipeline_extended.py          # 확장 테스트
│   ├── test_velocity_angle_measurement.py # 속도/각도 측정
│   ├── test_absolute_relative_measurement.py # 절대/상대 측정
│   └── streaming/                         # 실시간 스트리밍
│       ├── pointcloud_interface.py        # 포인트클라우드 GUI
│       ├── realsense_6dof_stream.py       # 실시간 6DoF 스트림
│       └── realsense_6dof_stream_video.py # 비디오 기반 스트림
│
├── tests/                                 # 단위 테스트
│   ├── test_pose_converter.py
│   ├── test_pose_kalman_filter.py
│   └── test_reference_image_loader.py
│
├── pointcloud_data/                       # 포인트클라우드 저장
├── test_result/                           # 테스트 결과
├── readme.md                              # 메인 README
├── requirements.txt                       # 의존성 목록
└── setup.py                               # 패키지 설치 스크립트
```

---

## 4. 핵심 패키지 (`nimg_v3/`)

### 4.1 config 모듈 - 시스템 설정

> **위치**: `nimg_v3/config/system_config.py`
> **역할**: 시스템 전역 설정을 데이터클래스로 관리하고 YAML로 직렬화

#### 주요 클래스

##### `CameraConfig`

RealSense D455의 기본 내부 파라미터를 담은 데이터클래스.

```python
@dataclass
class CameraConfig:
    fx: float = 383.883       # 초점 거리 X
    fy: float = 383.883       # 초점 거리 Y
    cx: float = 320.499       # 주점 X
    cy: float = 237.913       # 주점 Y
    width: int = 640          # 이미지 너비
    height: int = 480         # 이미지 높이
    depth_scale: float = 0.001  # depth → meter 변환
    depth_min: float = 0.1    # 최소 거리(m)
    depth_max: float = 10.0   # 최대 거리(m)
    fps: float = 30.0
```

##### `DetectionConfig`

YOLO 객체 탐지 관련 설정.

```python
@dataclass
class DetectionConfig:
    model_path: str = "models/yolo/class187_image85286_v12x_250epochs.pt"
    conf_threshold: float = 0.5
    iou_threshold: float = 0.45
    max_detections: int = 100
    img_size: int = 640
    half_precision: bool = True
```

##### `PoseEstimationConfig`

FoundationPose 동작 모드 및 가중치 설정.

```python
@dataclass
class PoseEstimationConfig:
    model_dir: str = "models/foundationpose"
    mode: str = "model_free"         # "model_based" or "model_free"
    mesh_path: Optional[str] = None
    neural_field_dir: Optional[str] = "models/neural_fields/painting_object"
    use_tensorrt: bool = True
    refine_iterations: int = 5
    tracking_recovery_threshold: float = 0.3
    max_lost_frames: int = 5
```

##### `KalmanFilterConfig`

칼만 필터 노이즈 파라미터.

```python
@dataclass
class KalmanFilterConfig:
    mode: str = "quaternion"  # "euler" 또는 "quaternion"
    process_noise_pos: float = 0.01
    process_noise_vel: float = 0.1
    process_noise_orient: float = 0.1
    process_noise_angular_vel: float = 1.0
    measurement_noise_pos: float = 0.005
    measurement_noise_orient: float = 0.5
    adaptive_noise: bool = True
```

##### `SystemConfig`

위 모든 설정을 통합하는 최상위 데이터클래스.

```python
@dataclass
class SystemConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    detection: DetectionConfig = field(default_factory=DetectionConfig)
    pose_estimation: PoseEstimationConfig = field(default_factory=PoseEstimationConfig)
    kalman_filter: KalmanFilterConfig = field(default_factory=KalmanFilterConfig)
    output: OutputConfig = field(default_factory=OutputConfig)
    device: str = "cuda:0"
    reference_frame_idx: int = 0

    def save(self, filepath)         # YAML로 저장
    @classmethod
    def from_dict(cls, d)            # 딕셔너리에서 복원
```

#### 헬퍼 함수

- `load_config(filepath)`: YAML 파일에서 SystemConfig 로드
- `create_default_config(save_path)`: 기본 설정 생성 및 저장

---

### 4.2 input 모듈 - 데이터 로딩

> **위치**: `nimg_v3/input/data_loader.py`
> **역할**: 디스크의 RGB+Depth 이미지 시퀀스 및 IMU 데이터를 통합 로드

#### `FrameData` 데이터클래스

단일 프레임의 모든 정보를 담는 객체.

```python
@dataclass
class FrameData:
    frame_idx: int
    rgb: np.ndarray              # [H, W, 3] uint8 (BGR)
    depth: np.ndarray            # [H, W] float32 (미터 단위)
    timestamp: float
    accel: Optional[np.ndarray]  # 가속도계 [ax, ay, az]
    gyro: Optional[np.ndarray]   # 자이로 [gx, gy, gz]
    rgb_path: Optional[str]
    depth_path: Optional[str]

    @property
    def has_imu(self) -> bool
    @property
    def depth_valid_ratio(self) -> float    # 유효 depth 픽셀 비율
```

#### `DataLoader` 클래스

```python
class DataLoader:
    def __init__(data_dir, depth_scale=0.001, fps=30.0,
                 depth_min=0.1, depth_max=10.0)
    def load_frame(idx) -> FrameData
    def __iter__()       # 순회 지원
    def __len__()        # 프레임 수
    def get_image_size() -> (height, width)
```

#### 지원 폴더 구조

| 구조 | 설명 |
|------|------|
| **subfolder** | `rgb/`, `depth/` 하위 폴더 사용 |
| **flat** | `color_*`, `depth_*` 같은 폴더에 평탄 저장 |

`imu_data.csv`가 존재하면 자동으로 IMU 데이터를 로드해 프레임별 시간 윈도우로 매칭합니다.

---

### 4.3 measurement 모듈 - 자세 변환 & 칼만 필터

#### 4.3.1 `pose_converter.py`

> **역할**: 6DoF 자세를 다양한 표현(오일러/쿼터니언/회전행렬)으로 변환

##### 핵심 데이터클래스

###### `EulerAngles`
```python
@dataclass
class EulerAngles:
    roll: float    # X축 회전 (도)
    pitch: float   # Y축 회전 (도)
    yaw: float     # Z축 회전 (도)

    def to_radians()
    def to_array() -> np.ndarray
    def normalize() -> EulerAngles    # -180~180 정규화
    def __sub__(other) -> EulerAngles  # 차이 계산 (정규화 포함)
```

###### `Quaternion`
```python
@dataclass
class Quaternion:
    x, y, z, w: float    # scipy/ROS 형식

    def to_array() -> [x, y, z, w]
    def to_array_wxyz() -> [w, x, y, z]
    def normalize() -> Quaternion       # 단위 쿼터니언
    def conjugate() -> Quaternion       # 켤레
    def inverse() -> Quaternion         # 역
    def __mul__(other) -> Quaternion    # 해밀턴 곱
    def angle_to(other) -> float        # 두 자세 간 각도(도)

    @classmethod
    def identity() -> Quaternion
    @classmethod
    def from_axis_angle(axis, angle_deg) -> Quaternion
```

###### `PoseComponents`
```python
@dataclass
class PoseComponents:
    translation: np.ndarray         # [x, y, z]
    euler: EulerAngles              # 사용자용
    quaternion: Quaternion          # ROS/내부용
    rotation_matrix: np.ndarray     # 3x3
    gimbal_lock_warning: bool       # 짐벌 락 경고
```

##### `PoseConverter` 클래스

scipy.spatial.transform 기반의 변환 엔진.

```python
class PoseConverter:
    GIMBAL_LOCK_THRESHOLD = 85.0  # 짐벌 락 감지 임계값

    def pose_matrix_to_components(4x4) -> PoseComponents
    def pose_to_euler(4x4)        -> (translation, EulerAngles)
    def pose_to_quaternion(4x4)   -> (translation, Quaternion)
    def quaternion_to_euler(quat) -> EulerAngles
    def euler_to_quaternion(euler) -> Quaternion
    def rotation_matrix_to_quaternion(R) -> Quaternion
    def quaternion_to_rotation_matrix(quat) -> np.ndarray
    def euler_to_rotation_matrix(euler)  -> np.ndarray
    def rotation_matrix_to_euler(R)      -> EulerAngles

    @staticmethod
    def slerp(q0, q1, t) -> Quaternion         # 구면 선형 보간
    @staticmethod
    def slerp_multi(quats, times, query_time)
    @staticmethod
    def compute_rotation_difference(q1, q2) -> (angle, axis)

    def compute_relative_pose(pose_ref, pose_curr) -> PoseComponents
```

##### 헬퍼 함수

- `compute_angle_change(prev_pose, curr_pose, use_quaternion=True)`: 두 자세 간 각도 변화. 쿼터니언 기반이 짐벌 락에 강건.
- `rotation_matrix_to_6d(R)`: Zhou et al. (CVPR 2019) 6D 연속 표현 (신경망 학습용)
- `sixd_to_rotation_matrix(sixd)`: 6D → 3x3 (Gram-Schmidt 직교화)

#### 4.3.2 `pose_kalman_filter.py`

> **역할**: 6DoF 자세에 대한 12/13-상태 칼만 필터로 속도와 각속도 추정

##### `FilterMode`
```python
class FilterMode(Enum):
    EULER = "euler"           # 12-상태
    QUATERNION = "quaternion" # 13-상태 (권장)
```

##### `PoseKalmanState`
```python
@dataclass
class PoseKalmanState:
    position: np.ndarray            # [x, y, z] m
    velocity: np.ndarray            # [vx, vy, vz] m/s
    orientation_euler: EulerAngles
    orientation_quat: Quaternion
    angular_velocity: np.ndarray    # [wx, wy, wz] deg/s
    speed: float                    # |velocity| m/s
```

##### `PoseKalmanFilter` 클래스

filterpy 기반 칼만 필터.

| 모드 | 상태 차원 | 상태 벡터 |
|------|----------|-----------|
| EULER | 12 | `[x,y,z, vx,vy,vz, roll,pitch,yaw, wx,wy,wz]` |
| QUATERNION | 13 | `[x,y,z, vx,vy,vz, qx,qy,qz,qw, wx,wy,wz]` |

```python
class PoseKalmanFilter:
    def __init__(dt=1/30.0, mode=FilterMode.QUATERNION,
                 process_noise_pos=0.01, ...,
                 measurement_noise_pos=0.005, ...,
                 adaptive_noise=True)

    def initialize(position, orientation)
    def predict() -> PoseKalmanState
    def update(position, orientation) -> PoseKalmanState
    def predict_and_update(position, orientation) -> PoseKalmanState
    def get_state() -> PoseKalmanState
    def update_adaptive_noise(depth_distance)  # 거리² ∝ noise
    def set_dt(dt)
    def reset()

    @property
    def position_uncertainty
    @property
    def orientation_uncertainty
```

##### 핵심 메커니즘

- **각도 래핑 처리** (`_unwrap_angles`): 측정값과 현재 상태의 차이가 180°를 넘지 않도록 조정
- **쿼터니언 부호 일관성** (`_ensure_quaternion_consistency`): q와 -q는 같은 회전이므로 내적이 양수가 되도록 부호 유지
- **쿼터니언 정규화**: 매 predict/update 후 단위 쿼터니언으로 정규화
- **적응형 노이즈**: D455 depth 오차 모델 (1m에서 약 5mm) 기반 거리에 따른 측정 노이즈 자동 조정

##### `AdaptivePoseKalmanFilter`

`PoseKalmanFilter`를 상속받아 이동/정지 상태를 자동 감지하고 노이즈를 조정.

```python
class AdaptivePoseKalmanFilter(PoseKalmanFilter):
    def __init__(motion_threshold=0.01,    # m/s
                 rotation_threshold=1.0,    # deg/s
                 ...)
    # 정지 시 노이즈 감소, 이동 시 기본 노이즈
```

---

### 4.4 pose 모듈 - 6DoF 자세 추정

#### 4.4.1 `foundationpose_estimator.py`

> **역할**: FoundationPose 통합 래퍼. Model-Based / Model-Free 모드 지원

##### Enum

```python
class PoseMode(Enum):
    MODEL_BASED = "model_based"  # CAD 모델 사용
    MODEL_FREE = "model_free"    # 참조 이미지 + Neural Field

class TrackingState(Enum):
    INITIALIZING = "initializing"
    TRACKING = "tracking"
    LOST = "lost"
```

##### `PoseResult`

```python
@dataclass
class PoseResult:
    pose_matrix: np.ndarray      # 4x4
    translation: np.ndarray      # [tx, ty, tz]
    rotation_matrix: np.ndarray  # 3x3
    confidence: float            # 0-1
    tracking_state: TrackingState
    processing_time_ms: float
```

##### `FoundationPoseEstimator` 핵심 메서드

```python
class FoundationPoseEstimator:
    def __init__(model_dir, mode, mesh_path=None,
                 neural_field_dir=None, device='cuda:0',
                 use_tensorrt=True,
                 tracking_recovery_threshold=0.3,
                 max_lost_frames=5,
                 refine_iterations=5)

    def estimate(rgb, depth, mask, intrinsics) -> PoseResult
    # 전체 파이프라인: 가설 생성 → 정제 → 점수
    
    def track(rgb, depth, intrinsics) -> PoseResult
    # 이전 자세 기반 빠른 정제
    
    def process(rgb, depth, mask, intrinsics, force_estimate=False) -> PoseResult
    # 추적 상태에 따라 estimate/track 자동 선택
    
    def reset()
```

##### 추정 파이프라인

```
RGB+Depth+Mask 입력
        ↓
   _validate_inputs()
        ↓
┌─────────────────────────────────────┐
│ 실제 FoundationPose 사용 가능?       │
│  ├─ YES: real_estimator.register()  │
│  └─ NO : _dummy_estimate() (PCA)    │
└─────────────────────────────────────┘
        ↓
 추적 상태 갱신 (TRACKING / LOST)
        ↓
    PoseResult 반환
```

##### 더미 구현

실제 FoundationPose 가중치가 없을 때 작동하는 폴백.

- `_dummy_estimate`: 마스크 영역 depth의 중앙값으로 z, 마스크 중심으로 (x,y) 계산. PCA로 yaw 추정.
- `_dummy_track`: 이전 자세에 1mm 노이즈 추가

##### 팩토리 클래스

```python
class FoundationPoseEstimatorFactory:
    @staticmethod
    def create_model_free(model_dir, neural_field_dir, ...)
    @staticmethod
    def create_model_based(model_dir, mesh_path, ...)
```

#### 4.4.2 `foundationpose_wrapper.py`

> **역할**: 실제 NVIDIA FoundationPose 공식 코드를 nimg_v3 인터페이스로 래핑

`/root/fursys_imgprosessing_ws/src/FoundationPose` 경로에서 다음을 임포트:
- `estimater.FoundationPose`
- `learning.training.predict_score.ScorePredictor`
- `learning.training.predict_pose_refine.PoseRefinePredictor`

##### `RealFoundationPoseEstimator`

```python
class RealFoundationPoseEstimator:
    def __init__(model_dir, mesh_path=None, device='cuda:0',
                 debug=0, debug_dir='/tmp/foundationpose_debug')

    def register(rgb, depth, mask, intrinsics) -> (pose, confidence)
    def track(rgb, depth, prev_pose, intrinsics) -> (pose, confidence)
    def reset()
    def get_mesh()
```

내부적으로 nvdiffrast 컨텍스트를 생성하고, 가중치 체크포인트를 다음 경로에서 로드:
- `<model_dir>/2024-01-11-20-02-45/model_best.pth` (Scorer)
- `<model_dir>/2023-10-28-18-33-37/model_best.pth` (Refiner)

#### 4.4.3 `real_foundationpose.py`

`foundationpose_wrapper.py`와 유사하지만 별도 디렉토리(`FOUNDATION_POSE_DIR = .../FoundationPose`)를 자동 탐색하는 또 다른 진입점. 메시가 없으면 0.1m 박스 더미 메시 생성.

```python
class RealFoundationPoseEstimator:
    def __init__(weights_dir=None, mesh_path=None,
                 device='cuda:0', debug=0)
    def estimate(rgb, depth, mask, K) -> (pose, confidence)
    def track(rgb, depth, K, iteration=2) -> (pose, confidence)
    def _fallback_estimate(...)  # FoundationPose 미가용 시 depth 기반 추정

def create_intrinsic_matrix(fx, fy, cx, cy) -> 3x3 ndarray
```

#### 4.4.4 `neural_object_field.py`

> **역할**: Model-Free FoundationPose의 핵심. 참조 이미지로부터 객체의 3D 표현 학습

##### Neural Field 구조

```
Neural Object Field
├── 기하학 함수 Ω: x → s (SDF)
│   └── 입력: 3D 점 x, 출력: 부호 있는 거리값 s
└── 외관 함수 Φ: (f, n, d) → c
    └── 입력: 기하학 특징 f, 법선 n, 시선 방향 d
    └── 출력: 색상 c
```

##### `NeuralFieldConfig`

```python
@dataclass
class NeuralFieldConfig:
    num_iterations: int = 1000
    batch_size: int = 1024
    learning_rate: float = 1e-3
    hidden_dim: int = 128
    num_layers: int = 4
    use_positional_encoding: bool = True
    positional_encoding_dim: int = 6
    num_samples_per_ray: int = 64
    near_plane: float = 0.1
    far_plane: float = 3.0
    marching_cubes_resolution: int = 128
    iso_level: float = 0.0
```

##### `NeuralObjectField` 핵심 메서드

```python
class NeuralObjectField:
    def __init__(config=None, device='cuda:0')
    def train(reference_images, camera_poses=None, verbose=True) -> stats
    def extract_mesh(resolution=None, iso_level=None) -> trimesh.Trimesh
    def render(camera_pose, intrinsics, image_size) -> (rgb, depth)
    def save(save_dir)        # config.json + metadata.json + weights.pkl + mesh.obj
    def load(load_dir)
```

##### 학습 절차

1. **카메라 자세 추정**: 폴더명에서 각도 정보(예: `top90`, `45deg`)를 파싱하여 카메라 자세 행렬 생성
2. **데이터 준비**: RGB/Depth/Mask를 스택, 카메라 자세와 묶음
3. **학습 시뮬레이션**: 현재는 구조만 정의(stats 반환). 실제 학습은 BundleSDF나 유사 라이브러리 필요.
4. **메시 추출**: scikit-image의 marching_cubes로 SDF에서 삼각형 메시 추출

##### 카메라 자세 생성 (`_create_camera_pose_from_angle`)

구면 좌표에서 카메라 위치를 계산하고 원점을 향해 look-at 변환:

```python
yaw, pitch (deg) → (x, y, z) 위치
forward = -position / |position|
right = cross(up, forward) / |...|
up = cross(forward, right)
pose = [right | up | forward | position]
```

#### 4.4.5 `reference_image_loader.py`

> **역할**: 다각도 참조 이미지 폴더에서 학습용 데이터셋 구축

##### 폴더 구조 가정

```
base_dir/
├── 20251229_093820_front/   # → angle "0"
│   ├── rgb/
│   ├── depth/
│   └── depth_csv/ (선택)
├── 20251229_094115_45/      # → angle "45"
└── 20251229_152111_bottom90/  # → angle "bottom90"
```

##### 핵심 클래스

```python
class ViewAngle(Enum):
    FRONT = 0
    DEG_45 = 45
    ...
    TOP_FRONT = "topfront"
    BOTTOM_270 = "bottom270"

@dataclass
class ReferenceImage:
    rgb: np.ndarray
    depth: np.ndarray
    mask: Optional[np.ndarray]
    view_angle: Optional[str]
    folder_name: str
    timestamp: float

@dataclass
class ReferenceImageSet:
    images: List[ReferenceImage]
    object_name: str
    camera_intrinsics: Optional[Dict[str, float]]

    def get_rgb_stack()   -> [N, H, W, 3]
    def get_depth_stack() -> [N, H, W]
    def get_mask_stack()  -> Optional[N, H, W]
```

##### `ReferenceImageLoader` 메서드

```python
class ReferenceImageLoader:
    ANGLE_PATTERNS = {'front': 0, '45': 45, ..., 'bottom270': 'bottom270'}

    def __init__(base_dir, depth_scale=0.001,
                 exclude_patterns=['_test'], camera_intrinsics=None)

    def load_all_views(images_per_view=1,
                       sample_strategy='middle')  # 'first' | 'middle' | 'random'
    def load_specific_views(view_angles)
    def generate_masks_from_depth(ref_set,
                                  depth_threshold=0.1,
                                  min_depth=0.3, max_depth=2.0)
    def get_available_angles() -> [angle, ...]
    def get_statistics() -> dict
```

##### Depth 기반 자동 마스크 생성

1. 유효 depth 범위 (min~max) 마스킹
2. 중앙값 기준 전경/배경 분리
3. 모폴로지 연산 (Open + Close)으로 노이즈 제거
4. 가장 큰 contour만 유지

---

### 4.5 pointcloud 모듈 - 3D 포인트클라우드

> **역할**: Intel RealSense D455 기반 포인트 클라우드 캡처/처리/시각화/ROS2 퍼블리시

#### 4.5.1 `camera_wrapper.py`

```python
class RealSenseCameraWrapper:
    def __init__(width=640, height=480, fps=30)
    
    # 카메라 제어
    def start() -> bool          # 자동 폴백 설정 시도
    def stop()
    
    # 데이터 획득
    def read() -> (rgb, depth)   # BGR uint8, float32 미터
    def get_pointcloud(apply_filter=True) -> o3d.PointCloud
    def get_pointcloud_from_rgbd(rgb, depth) -> o3d.PointCloud
    def get_intrinsics() -> Dict
```

##### Depth 필터 체인

| 필터 | 설명 |
|------|------|
| **Decimation** | 2배 다운샘플링 (성능 향상) |
| **Spatial** | 공간 평활 (filter_smooth_alpha=0.5) |
| **Temporal** | 시간 평활 (flicker 감소) |

설정 시도 순서: 사용자 요청 → 640x480@30fps → 640x480@15fps

#### 4.5.2 `capture_manager.py`

```python
class PointCloudCaptureManager:
    def __init__(camera, save_dir="./pointcloud_data",
                 preprocessing_enabled=True)

    def capture_pointcloud() -> o3d.PointCloud
    def save_pointcloud(pcd, filename=None, format="ply") -> filepath
    def load_pointcloud(filepath) -> o3d.PointCloud
    
    def preprocess_pointcloud(pcd, voxel_size=0.005,
                              remove_outliers=True,
                              estimate_normals=True)
    
    def get_saved_list() -> List[Dict]
    def delete_pointcloud(filepath) -> bool
    def get_pointcloud_info(pcd) -> Dict     # bbox, center 등
    def cleanup_old_pointclouds(max_count=10)
```

##### 전처리 파이프라인

1. **Statistical Outlier Removal**: 평균 거리에서 표준편차 2배 이상 이격된 점 제거
2. **Voxel Downsampling**: 5mm 복셀 격자
3. **Normal Estimation**: 반경 0.1m / 최대 30개 이웃

#### 4.5.3 `visualizer.py`

```python
class PointCloudVisualizer:
    def __init__(point_size=1.0, coordinate_frame_size=0.3,
                 background_color=[0.1, 0.1, 0.1])

    def visualize_pointcloud(pcd, window_name, ...)
    def visualize_multiple(pointclouds, colors, ...)
    def create_coordinate_frame(size, origin)
    def save_screenshot(pcd, output_path, width=1920, height=1080)
```

#### 4.5.4 `ros2_publisher.py`

ROS2 sensor_msgs/PointCloud2 퍼블리셔 (싱글톤).

```python
class ROS2PointCloudPublisher:
    def __init__(topic_name="pc_point", frame_id="camera_link")
    def init_ros2_node() -> bool
    def publish_pointcloud(pcd) -> bool
    def shutdown()
    
    @staticmethod
    def is_ros2_available() -> bool
```

PointCloud2 메시지 형식:
- 필드: x (float32), y (float32), z (float32), rgb (uint32)
- point_step: 16 bytes/point
- 색상은 BGR → RGB로 변환 후 packed uint32

#### 4.5.5 `gui.py`

PyQt5 기반 GUI (`PointCloudInterfaceGUI`).

##### 주요 위젯

- 카메라 미리보기 (640x480 라벨, 30fps QThread 스트림)
- 캡처 버튼 (녹색)
- ROS2 퍼블리시 버튼 (파랑)
- 3D 시각화 버튼 (주황)
- 전처리 토글 체크박스
- 저장된 포인트클라우드 리스트 (파일명 / 점 개수 / 크기 표시)
- 삭제 버튼
- 상태바 (카메라/ROS2 상태, FPS, 저장 개수)

##### 워커 스레드

```python
class CameraWorker(QThread):
    frame_ready = pyqtSignal(np.ndarray)
    error_occurred = pyqtSignal(str)
    
    def run()  # 30fps로 카메라 프레임 emit
```

##### 진입점

```python
def run_gui(save_dir=None):
    app = QApplication(sys.argv)
    window = PointCloudInterfaceGUI(save_dir=save_dir)
    window.show()
    app.exec_()
```

---

### 4.6 output 모듈 - 결과 내보내기

> **위치**: `nimg_v3/output/result_exporter.py`

#### `ResultExporter` 클래스

```python
class ResultExporter:
    def __init__(output_dir, prefix="nimg_v3", format="csv")
    def add_result(result)         # to_dict() 호출 가능 객체 또는 dict
    def add_results(results)
    def save(filename=None) -> filepath
    def clear()
    def get_summary() -> Dict
```

##### CSV 저장 시 평탄화

`{'pose': {'euler': {'roll': 30, ...}, ...}, ...}` →
`{'pose_euler_roll': 30, ...}`

배열은 길이 4 이하면 인덱스별로 컬럼 분리, 그 외에는 문자열로 저장.

##### 요약 통계

```python
{
    'num_frames': int,
    'speed': {'mean', 'std', 'max'},
    'confidence': {'mean', 'min'}
}
```

---

### 4.7 detection / tracking / utils 모듈

#### detection 모듈

`nimg_v2`의 `YOLODetector`를 동적으로 로드하는 얇은 래퍼:

```python
nimg_v2_path = Path(__file__).parents[3] / 'nimg_v2' / 'nimg_v2'
sys.path.insert(0, str(nimg_v2_path))
from detection.yolo_detector import YOLODetector, Detection
```

#### tracking 모듈

내용 없음. FoundationPose의 내장 추적 기능에 위임.

#### utils 모듈

내용 없음 (확장 포인트).

---

### 4.8 main.py - 통합 측정 시스템

> **위치**: `nimg_v3/main.py`
> **역할**: YOLO + FoundationPose + Kalman Filter를 통합한 단일 진입점

#### `MeasurementResult`

```python
@dataclass
class MeasurementResult:
    # 탐지
    detection_bbox: tuple
    detection_confidence: float
    class_id: int
    class_name: str

    # 자세
    pose_matrix: np.ndarray
    translation: np.ndarray
    euler_angles: EulerAngles
    quaternion: Quaternion
    pose_confidence: float
    tracking_state: str

    # Kalman 상태
    filtered_position: np.ndarray
    velocity: np.ndarray
    speed: float
    angular_velocity: np.ndarray

    # 기준 대비 변화량
    position_change: Optional[np.ndarray]
    angle_change: Optional[np.ndarray]

    # 메타
    frame_idx: int
    timestamp: float
    processing_time_ms: float
    
    def to_dict() -> Dict
```

#### `IntegratedMeasurementSystem`

##### 초기화

```python
class IntegratedMeasurementSystem:
    def __init__(yolo_model_path,
                 foundationpose_model_dir,
                 pose_mode=PoseMode.MODEL_FREE,
                 mesh_path=None,
                 neural_field_dir=None,
                 intrinsics=None,
                 reference_frame_idx=0,
                 fps=30.0,
                 device='cuda:0',
                 kalman_mode=FilterMode.QUATERNION)
```

##### 처리 파이프라인 (`process_frame`)

```
1. 객체 탐지 (YOLO 또는 사전 탐지)
        ↓
2. BBox로부터 마스크 생성
        ↓
3. FoundationPose 추정/추적 (자동 모드 전환)
        ↓
4. PoseConverter로 Euler/Quaternion 변환
        ↓
5. Kalman Filter predict + update
        ↓
6. 거리 기반 적응형 노이즈 갱신
        ↓
7. 기준 프레임 대비 절대 변화량 계산 (position/angle)
        ↓
8. MeasurementResult 반환
```

##### 주요 메서드

```python
def process_frame(rgb, depth, timestamp=None, detection=None) -> MeasurementResult
def process_sequence(frames) -> List[MeasurementResult]
def reset()

@property
def is_reference_set
@property
def is_tracking
@property
def frame_count

@classmethod
def from_config(config: SystemConfig) -> IntegratedMeasurementSystem
```

##### 지연 로딩

YOLO와 FoundationPose는 첫 프레임 처리 시점에 초기화:
- `_init_detector()`: nimg_v2의 YOLODetector 로드 시도, 실패 시 더미 탐지
- `_init_pose_estimator()`: FoundationPoseEstimator 생성

##### 헬퍼 함수

```python
def run_measurement(rgb, depth, config=None) -> Optional[MeasurementResult]
# 단일 프레임 처리용 편의 함수
```

---

## 5. scripts 폴더

### 5.1 `train_neural_field.py`

> 참조 이미지로부터 Neural Object Field 학습

```bash
python scripts/train_neural_field.py \
    --ref_dir /root/fursys_img_251229/extraction \
    --output_dir models/neural_fields/painting_object \
    --num_iterations 1000 \
    --device cuda:0 \
    --generate_masks
```

플로우: ReferenceImageLoader → (옵션 마스크 생성) → NeuralObjectField.train → extract_mesh → save

### 5.2 `run_measurement.py`

> 오프라인 데이터에서 6DoF 측정

```bash
python scripts/run_measurement.py \
    --data_dir /path/to/data \
    --output_dir output \
    --config config.yaml \
    --visualize \
    --max_frames 100
```

DataLoader → IntegratedMeasurementSystem → ResultExporter → (옵션 OpenCV 시각화)

### 5.3 `test_pipeline.py`

> YOLO 탐지 기본 테스트

`PipelineTestRunner` 클래스가 100장 샘플링하여:
- 탐지 시각화 (`test_result/visualizations/`)
- JSON 결과 (`test_result/test_results.json`)
- 클래스별 통계
- 텍스트 요약 + 요약 이미지

### 5.4 `test_pipeline_extended.py`

> YOLO + 참조 이미지 + 칼만 필터 통합 테스트

5단계 테스트:
1. YOLO 모델 로드/검출
2. ReferenceImageLoader 동작
3. 칼만 필터 (60프레임 원형 시뮬레이션)
4. 테스트 이미지/Depth 시각화
5. 결과 저장 (JSON, 로그, 요약 이미지)

칼만 시뮬레이션은 matplotlib로 4-panel 플롯 생성 (XY 궤적, X/Y/Yaw 시계열).

### 5.5 `test_velocity_angle_measurement.py`

> YOLO 없이 Depth 기반 객체 추적

`DepthBasedObjectTracker` 사용:
1. Depth 임계값으로 객체 마스킹
2. 모폴로지 연산
3. 가장 큰 연결 영역 선택
4. 중심 픽셀 + 주변 ROI depth 중앙값으로 3D 위치
5. PCA로 yaw 추정
6. Kalman Filter로 속도/각속도 추정

각 시퀀스마다 6-panel matplotlib 플롯 생성 (X/Y/Z 위치, 속도, Yaw, XY 궤적).

### 5.6 `test_absolute_relative_measurement.py`

> 기준 프레임 대비 절대/상대 변화량 측정

`AbsoluteRelativeMeasurementTest`:
- 첫 프레임을 기준(reference)으로 설정
- 프레임마다 절대 위치/각도 변화 계산
- 연속 프레임 간 상대 변화량 계산
- 3개의 종합 플롯 생성:
  1. `position_velocity.png`: X/Y/Z 위치 + Vx/Vy/Vz 속도
  2. `absolute_changes.png`: 기준 대비 dX/dY/dZ, 거리, Yaw, XY 궤적
  3. `relative_changes.png`: 프레임 간 거리 변화, Yaw 변화, 속도, Yaw 비교

전체 측정값을 `measurements.csv`로 저장.

### 5.7 `streaming/pointcloud_interface.py`

> 포인트클라우드 GUI 진입점

```bash
python scripts/streaming/pointcloud_interface.py \
    --save-dir /path/to/data \
    --no-ros2
```

PyQt5 앱 생성 → `PointCloudInterfaceGUI` 띄움.

### 5.8 `streaming/realsense_6dof_stream.py`

> RealSense D455 실시간 6DoF 자세 추정 스트리밍

핵심 기능:
- GPU 가속 YOLO 세그멘테이션 (YOLOv26 2-class: housing_M, Wiring_tray)
- 실시간 6DoF 자세 추정 + 좌표축 시각화
- Kalman Filter
- YAW 기반 reference 측정
- 신호 분류 (-2, -1, 0, 1, 2, none)
- ROI 필터링 (Angle ROI: (130,80)~(260,250), Speed ROI 별도)
- 다중 객체 추적 (영구 ID 할당, IoU 매칭)
- 객체 우측→좌측 순서 정렬
- ROS2 Foxy `p_s` 토픽 퍼블리시
- TCP 서버 (포트 9999)
- OpenCV 윈도우 (resizable, fullscreen 토글)

조작:
- `q` / `ESC`: 종료
- `s`: 현재 프레임 저장
- `r`: baseline 리셋
- `p`: 일시정지
- `f`: 전체화면 토글

설정 상수:
- `CONFIDENCE_THRESHOLD = 0.95`
- `IOU_THRESHOLD = 0.3`
- `MAX_LOST_FRAMES = 60`

### 5.9 `streaming/realsense_6dof_stream_video.py`

> 비디오 파일 기반 6DoF 스트리밍 (테스트/시뮬레이션용)

`realsense_6dof_stream.py`와 동일한 기능을 RealSense 카메라 대신 RGB+Depth 비디오 파일 쌍으로 무한 루프 재생.

```bash
python scripts/streaming/realsense_6dof_stream_video.py \
    --rgb-video /path/to/rgb.mp4 \
    --depth-video /path/to/depth.mp4 \
    --no-tcp \
    --no-loop
```

---

## 6. tests 폴더

pytest 기반 단위 테스트. **48개 테스트 모두 통과** (PASSED).

### 6.1 `test_pose_converter.py`

| 테스트 클래스 | 검증 항목 |
|---------------|----------|
| `TestEulerAngles` | 생성, 배열 변환, 정규화, 뺄셈 |
| `TestQuaternion` | identity, 정규화, 켤레, axis-angle, 곱셈 |
| `TestPoseConverter` | 항등 자세, 이동, 회전, 왕복 변환 |
| `TestAngleChange` | 두 자세 간 각도 변화 (쿼터니언/오일러) |
| `Test6DRepresentation` | 6D 연속 표현 왕복 변환 |
| `TestGimbalLock` | 짐벌 락 감지 |

### 6.2 `test_pose_kalman_filter.py`

| 테스트 클래스 | 검증 항목 |
|---------------|----------|
| `TestPoseKalmanFilterEuler` | 오일러 모드 dim_x=12, predict/update |
| `TestPoseKalmanFilterQuaternion` | 쿼터니언 모드 dim_x=13, 정규화 유지 |
| `TestKalmanStateOutput` | PoseKalmanState 출력 형식 |
| `TestAdaptiveNoise` | 거리 기반 노이즈 조정 |
| `TestMultipleFrames` | 연속 프레임 추적 |
| `TestFilterReset` | 필터 리셋 |

### 6.3 `test_reference_image_loader.py`

`/root/fursys_img_251229/extraction` 경로가 존재할 때만 실행 (없으면 skip).

| 테스트 항목 |
|----------|
| 로더 초기화 |
| 뷰 폴더 탐색 (rgb 디렉토리 존재 확인) |
| 모든 뷰 로드 (`load_all_views`) |
| ReferenceImage 속성 (RGB uint8 3D, Depth float32 2D) |
| RGB/Depth 스택 생성 |
| 폴더명에서 각도 추출 |
| 제외 패턴 (`_test`) |

### 6.4 테스트 실행

```bash
cd nimg_v3
pytest tests/ -v
pytest tests/test_pose_converter.py -v
pytest tests/ -v --cov=nimg_v3
```

---

## 7. models 및 설정

### 7.1 models 폴더

```
models/
├── foundationpose/        # 외부 다운로드 필요
│   ├── refiner/config.yml
│   └── scorer/config.yml
├── neural_fields/
│   ├── housing_M/
│   │   └── reference_images/
│   │       ├── reference_config.yaml      # 신호 매핑 + yaw 정렬
│   │       ├── labels/                    # 17개 yaw 라벨 텍스트
│   │       └── *.png                       # 17장 (0deg, 45deg, ..., bottom_270deg)
│   └── Wiring_tray/
│       └── reference_images/
│           ├── reference_config.yaml
│           ├── labels/                    # 14개 yaw 라벨
│           └── 20251229_*.png              # 14장
└── yolo/
    └── class187_image85286_v12x_250epochs.pt   # 114MB
```

#### `reference_config.yaml` 구조 (housing_M 예시)

```yaml
class_name: housing_M
class_id: 0
baseline_image: "bottom_90deg_png.rf.1kL6B232ITP4C2PmyGAv.png"
signal_mapping:
  -2: "225deg_..."
  -1: "top_0deg_..."
  0:  "bottom_90deg_..."   # baseline과 동일
  1:  "bottom_90deg_..."
  2:  "top_90deg_..."
none_lower_bound: -36.0
none_upper_bound: 46.0
all_images:
  - filename: "225deg_..."
    yaw: -178.86
  - filename: "top_270deg_..."
    yaw: -176.59
  ... (17개)
```

이 매핑은 실시간 스트리밍에서 객체의 yaw를 5단계 신호(-2 ~ +2)로 분류하는 데 사용됩니다.

### 7.2 YOLO 모델 사양

| 항목 | 값 |
|------|---|
| 아키텍처 | YOLOv12x (Ultralytics) |
| 클래스 수 | 187 |
| 학습 이미지 | 85,286장 |
| 에포크 | 250 |
| 신뢰도 임계값 (기본) | 0.5 |
| 파일 크기 | 114MB |

다운로드: https://magenta.dooray.com/project/pages/4221286911063439348

스트리밍 스크립트는 별도의 YOLOv26 2-class 모델 사용:
- `models/yolo/yolo26_2class_seg_best_260324.pt` (housing_M, Wiring_tray)

### 7.3 default_config.yaml

```yaml
camera:
  fx: 383.883
  fy: 383.883
  cx: 320.499
  cy: 237.913
  width: 640
  height: 480
  depth_scale: 0.001
  fps: 30.0

detection:
  model_path: "models/yolo/class187_image85286_v12x_250epochs.pt"
  conf_threshold: 0.5
  iou_threshold: 0.45

pose_estimation:
  model_dir: "models/foundationpose"
  mode: "model_free"
  neural_field_dir: "models/neural_fields/painting_object"
  use_tensorrt: true
  refine_iterations: 5

kalman_filter:
  mode: "quaternion"
  process_noise_pos: 0.01
  measurement_noise_pos: 0.005
  adaptive_noise: true

device: "cuda:0"
reference_frame_idx: 0
```

---

## 8. 데이터 흐름 (Data Flow)

### 8.1 Offline 측정 (배치 처리)

```
┌──────────────┐     ┌──────────────┐     ┌──────────────────┐
│ DataLoader   │ →  │ FrameData    │ →  │ Integrated       │
│ (rgb/depth)  │     │ (rgb,depth,  │     │ MeasurementSystem│
│              │     │  imu, ts)    │     │                  │
└──────────────┘     └──────────────┘     └────────┬─────────┘
                                                    ↓
                              ┌─────────────────────┴───────────┐
                              ↓                                  ↓
                   ┌──────────────────┐               ┌──────────────────┐
                   │  YOLO Detection  │               │ Reference Pose   │
                   │  (BBox+Mask)     │               │ (frame_idx==ref) │
                   └────────┬─────────┘               └──────────────────┘
                            ↓
                   ┌──────────────────┐
                   │ FoundationPose   │
                   │ (4x4 matrix)     │
                   └────────┬─────────┘
                            ↓
                   ┌──────────────────┐
                   │ PoseConverter    │
                   │ (Euler+Quat)     │
                   └────────┬─────────┘
                            ↓
                   ┌──────────────────┐
                   │ Kalman Filter    │
                   │ (vel+ang_vel)    │
                   └────────┬─────────┘
                            ↓
                   ┌──────────────────┐
                   │ Compute Change   │
                   │ (vs reference)   │
                   └────────┬─────────┘
                            ↓
                   ┌──────────────────┐
                   │ MeasurementResult│
                   └────────┬─────────┘
                            ↓
                   ┌──────────────────┐
                   │ ResultExporter   │
                   │ (CSV / JSON)     │
                   └──────────────────┘
```

### 8.2 Real-time 스트리밍

```
RealSense D455
    ↓
┌─────────────────────────┐
│ RealSenseCameraWrapper  │ → RGB+Depth (30fps)
│ + RS Filters            │
└──────────┬──────────────┘
           ↓
┌─────────────────────────┐
│ YOLOv26 Segmentation    │ → BBox + Mask
│ (GPU)                   │
└──────────┬──────────────┘
           ↓
┌─────────────────────────┐
│ Multi-Object Tracker    │ → TrackedObject (ID 영구)
│ (IoU Matching)          │
└──────────┬──────────────┘
           ↓
┌─────────────────────────┐
│ PCA-based Yaw Estimate  │ → Yaw (deg)
└──────────┬──────────────┘
           ↓
┌─────────────────────────┐
│ Kalman Filter (Quat)    │ → Filtered State
└──────────┬──────────────┘
           ↓
┌─────────────────────────┐
│ Signal Classification   │ → -2 / -1 / 0 / 1 / 2 / none
│ (Reference Yaw Match)   │
└──────────┬──────────────┘
           ↓
    ┌──────┴──────┐
    ↓             ↓
┌────────┐   ┌─────────┐
│ ROS2   │   │  TCP    │
│ /p_s   │   │  :9999  │
└────────┘   └─────────┘
```

### 8.3 PointCloud GUI

```
┌──────────────┐
│ Camera Worker│ (QThread, 30fps)
│ (rgb)        │
└──────┬───────┘
       ↓ pyqtSignal(frame_ready)
┌──────────────┐
│ GUI Preview  │
└──────┬───────┘
       │
       │ Capture 버튼 클릭
       ↓
┌──────────────────┐
│ CaptureManager   │
│ + Preprocessing  │ → SOR, Voxel, Normals
└──────┬───────────┘
       ↓
┌──────────────────┐
│ Save .ply file   │
└──────┬───────────┘
       │
       │ Publish 버튼 클릭
       ↓
┌──────────────────┐
│ ROS2 Publisher   │
│ /pc_point        │
└──────────────────┘
```

---

## 9. 핵심 설계 원칙

### 9.1 회전 표현 전략 (하이브리드)

`euler_vs_quaternion_rotation_analysis.md`에 기반한 설계:

| 사용 위치 | 표현 | 이유 |
|----------|------|------|
| **내부 처리** | 쿼터니언 | 짐벌 락 없음, 수치 안정성, 연속성 |
| **사용자 인터페이스** | 오일러 (도) | 직관적 이해 |
| **ROS 통신** | 쿼터니언 | `geometry_msgs/Pose` 표준 |
| **로깅/저장** | 둘 다 | 호환성 + 디버깅 |
| **신경망 학습** | 6D 연속 표현 | Zhou et al. CVPR 2019 |

### 9.2 짐벌 락 처리

- `PoseConverter.GIMBAL_LOCK_THRESHOLD = 85.0`
- Pitch가 ±90°에 5° 이내로 접근하면 경고 로그
- 두 자세 간 각도 변화는 쿼터니언으로 계산하면 짐벌 락 영향 없음

### 9.3 적응형 노이즈

RealSense D455의 depth 오차는 거리²에 비례:
- 1m → 약 5mm 오차
- 2m → 약 20mm 오차

```python
pos_error = base_error * (depth_distance ** 2)
pos_error = clip(pos_error, 0.002, 0.1)
self.kf.R[:3,:3] = pos_error ** 2 * I
```

### 9.4 Lazy Initialization (지연 로딩)

`IntegratedMeasurementSystem`은 무거운 모델(YOLO, FoundationPose)을 첫 프레임 처리 시점에 초기화. 단순한 단위 테스트에서는 모델 로드를 회피 가능.

### 9.5 Fallback 전략

실제 FoundationPose가 없거나 실패 시:
1. `_dummy_estimate`: 마스크의 depth 중앙값 + PCA 회전
2. 신뢰도는 유효 픽셀 수에 비례

### 9.6 추적 상태 머신

```
INITIALIZING ─┐
              ├─→ TRACKING ─→ (continue tracking)
              │       │
              │       ↓ (confidence < 0.3)
              │      LOST ─→ (max_lost_frames 초과 시)
              └────── (재등록 시도)
```

---

## 10. 사용 시나리오

### 10.1 시나리오 1: 새 객체 등록 (Model-Free)

```bash
# 1. 다각도 참조 이미지 촬영 (8-16개 시점)
# 폴더 구조: front, 45, 90, ..., topfront, ...

# 2. Neural Field 학습
python scripts/train_neural_field.py \
    --ref_dir /path/to/reference_images \
    --output_dir models/neural_fields/new_object \
    --num_iterations 1000 \
    --generate_masks

# 3. 측정 실행
python scripts/run_measurement.py \
    --data_dir /path/to/test_data \
    --output_dir results
```

### 10.2 시나리오 2: 기존 모델로 검증

```bash
# YOLO 검출 테스트
python scripts/test_pipeline.py

# 확장 테스트 (YOLO + 참조 + Kalman)
python scripts/test_pipeline_extended.py

# Depth 기반 절대/상대 측정 (YOLO 없이)
python scripts/test_absolute_relative_measurement.py

# 속도/각도 측정
python scripts/test_velocity_angle_measurement.py
```

### 10.3 시나리오 3: Python API 사용

```python
from nimg_v3.main import IntegratedMeasurementSystem
from nimg_v3.config.system_config import SystemConfig, load_config

# 설정 로드
config = load_config("nimg_v3/config/default_config.yaml")

# 시스템 생성
system = IntegratedMeasurementSystem.from_config(config)

# 단일 프레임 처리
result = system.process_frame(rgb, depth, timestamp=time.time())

if result is not None:
    print(f"Position: {result.translation}")
    print(f"Yaw: {result.euler_angles.yaw:.2f}")
    print(f"Speed: {result.speed:.3f} m/s")
    print(f"Confidence: {result.pose_confidence:.2f}")
    print(f"Tracking: {result.tracking_state}")

# 배치 처리
frames = [{'rgb': rgb1, 'depth': d1}, {'rgb': rgb2, 'depth': d2}, ...]
results = system.process_sequence(frames)
```

### 10.4 시나리오 4: 실시간 스트리밍 (RealSense)

```bash
# 기본 실행
python scripts/streaming/realsense_6dof_stream.py

# 옵션
python scripts/streaming/realsense_6dof_stream.py \
    --width 640 --height 480 --fps 30 \
    --save output.mp4 \
    --no-ros \
    --no-roi
```

### 10.5 시나리오 5: 포인트클라우드 GUI

```bash
python scripts/streaming/pointcloud_interface.py \
    --save-dir ./pointcloud_data
```

GUI에서:
1. **Capture** 버튼으로 포인트클라우드 캡처 → .ply 저장
2. 리스트에서 파일 선택
3. **Publish to ROS2**로 `/pc_point` 토픽 퍼블리시
4. **Visualize 3D**로 Open3D 뷰어 열기
5. **Preprocessing** 체크박스로 SOR/Voxel/Normals 토글

---

## 11. 의존성 및 설치

### 11.1 requirements.txt

```
# Core
numpy>=1.20.0
scipy>=1.7.0
opencv-python>=4.5.0
filterpy>=1.4.5
pyyaml>=5.4.0
pandas>=1.3.0

# Deep Learning (PyTorch)
torch>=1.10.0
torchvision>=0.11.0

# YOLO Detection
ultralytics>=8.0.0

# 3D Processing
trimesh>=3.9.0
open3d>=0.13.0
scikit-image>=0.18.0

# GUI
PyQt5>=5.15.0

# RealSense
pyrealsense2>=2.50.0

# Development
pytest>=6.0.0
pytest-cov>=2.0.0
```

### 11.2 setup.py 정의 패키지

```python
setup(
    name='nimg_v3',
    version='3.0.0',
    description='FoundationPose-based 6DoF Pose Estimation System',
    install_requires=[
        'numpy>=1.20.0', 'scipy>=1.7.0', 'opencv-python>=4.5.0',
        'filterpy>=1.4.5', 'pyyaml>=5.4.0', 'pandas>=1.3.0'
    ],
    extras_require={
        'full': ['torch>=1.10.0', 'trimesh>=3.9.0',
                 'scikit-image>=0.18.0', 'ultralytics>=8.0.0'],
        'dev':  ['pytest>=6.0.0', 'pytest-cov>=2.0.0'],
    },
    entry_points={
        'console_scripts': [
            'nimg_v3_train=scripts.train_neural_field:main',
            'nimg_v3_measure=scripts.run_measurement:main',
        ],
    },
)
```

### 11.3 설치 방법

```bash
# 기본 설치
pip install -e .

# 전체 기능
pip install -e .[full]

# 개발 환경
pip install -e .[dev]

# RealSense 시스템 패키지 (Ubuntu)
sudo apt install librealsense2-dev python3-pyrealsense2

# ROS2 (옵션)
sudo apt install ros-foxy-rclpy ros-foxy-sensor-msgs
```

### 11.4 외부 의존성

#### NVIDIA FoundationPose

`/root/fursys_imgprosessing_ws/src/FoundationPose` 경로에 설치 필요:
- 공식 저장소: https://github.com/NVlabs/FoundationPose
- Docker 이미지: `wenbowen123/foundationpose`
- nvdiffrast 빌드 필요 (`build_all.sh` 실행)
- 가중치:
  - `weights/no_diffusion/2024-01-11-20-02-45/model_best.pth` (Scorer)
  - `weights/no_diffusion/2023-10-28-18-33-37/model_best.pth` (Refiner)

#### nimg_v2 (선택)

`detection/yolo_detector.py`를 동적으로 임포트하여 YOLO 탐지에 사용. 없으면 폴백 더미 탐지 사용.

---

## 12. 향후 개선 사항

| # | 항목 | 설명 |
|---|------|------|
| 1 | **FoundationPose 실제 통합** | 현재 더미 구현 → NVIDIA 공식 모델 연동 강화 |
| 2 | **Neural Field 실제 학습** | 현재 시뮬레이션 → BundleSDF 등 실제 학습 |
| 3 | **TensorRT 최적화** | 추론 속도 향상 |
| 4 | **다중 객체 추적** | 복수 도장물 동시 추적 (스트리밍에서는 부분 구현) |
| 5 | **utils/tracking 모듈 확장** | 현재 비어있음 |
| 6 | **EKF/UKF 도입** | 비선형 쿼터니언 동역학 정확 표현 |

---

## 13. 참고 자료

- **FoundationPose Paper** (Section 4.1, NeRF 비교)
- **Zhou et al. (CVPR 2019)** - "On the Continuity of Rotation Representations in Neural Networks" (6D 연속 표현)
- **euler_vs_quaternion_rotation_analysis.md** - 회전 표현 분석 (내부 문서)
- **nimg_v3_foundationpose_comprehensive_design_guide.md** - 종합 설계 가이드 (내부 문서)
- **NIMG_V3_STRUCTURE_ANALYSIS.md** - 폴더 구조 상세 분석 (`docs/`)
- **pointcloud_interface_guide.md** - 포인트클라우드 GUI 가이드 (`docs/`)

---

**문서 끝**
