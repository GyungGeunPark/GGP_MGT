# nimg_v3 폴더 구조 상세 분석

**작성일**: 2024-12-31
**작성자**: FurSys AI Team
**버전**: 3.0.0

---

## 목차
1. [전체 구조 개요](#1-전체-구조-개요)
2. [models 폴더 상세](#2-models-폴더-상세)
3. [reference_images 폴더 상세](#3-reference_images-폴더-상세)
4. [nimg_v3 코어 모듈](#4-nimg_v3-코어-모듈)
5. [scripts 폴더 상세](#5-scripts-폴더-상세)
6. [tests 폴더 상세](#6-tests-폴더-상세)
7. [설정 및 의존성 파일](#7-설정-및-의존성-파일)

---

## 1. 전체 구조 개요

```
nimg_v3/
├── models/                    # 학습된 모델 저장소
│   ├── foundationpose/        # FoundationPose 사전학습 가중치
│   ├── neural_fields/         # 학습된 Neural Object Field
│   └── yolo/                  # 파인튜닝된 YOLO 모델
├── reference_images/          # 참조 이미지 (사용 안함 - 외부 경로 사용)
├── nimg_v3/                   # 코어 Python 패키지
│   ├── config/                # 시스템 설정
│   ├── detection/             # 객체 탐지 (YOLO)
│   ├── input/                 # 데이터 입력/로드
│   ├── measurement/           # 측정 및 변환 모듈
│   ├── output/                # 결과 출력
│   ├── pose/                  # 자세 추정 (FoundationPose)
│   ├── tracking/              # 객체 추적
│   ├── utils/                 # 유틸리티
│   └── main.py                # 통합 측정 시스템
├── scripts/                   # 실행 스크립트
├── tests/                     # 단위 테스트
├── test_result/               # 테스트 결과 저장
├── requirements.txt           # 의존성 패키지
└── setup.py                   # 패키지 설치 스크립트
```

---

## 2. models 폴더 상세

### 2.1 폴더 구조
```
models/
├── foundationpose/
├── neural_fields/
└── yolo/
    └── class187_image85286_v12x_250epochs.pt  # 114MB, 파인튜닝된 YOLO 모델
```

### 2.2 foundationpose 폴더 (비어있음)
**역할**: FoundationPose 사전학습 모델 가중치를 저장하는 디렉토리

**들어가야 하는 파일**:
```
foundationpose/
├── scorer.pth                 # 자세 점수 평가 네트워크
├── refiner.pth               # 자세 정제 네트워크
├── feature_extractor.pth     # 특징 추출 네트워크
└── config.yaml               # 모델 설정 파일
```

**다운로드 방법**:
```bash
# NVIDIA FoundationPose 공식 저장소에서 다운로드
# https://github.com/NVlabs/FoundationPose
# 또는 Docker 이미지 사용: wenbowen123/foundationpose
```

### 2.3 neural_fields 폴더 (비어있음)
**역할**: `train_neural_field.py` 스크립트로 학습된 Neural Object Field를 저장

**들어가야 하는 파일** (학습 후 자동 생성):
```
neural_fields/painting_object/     # 객체 이름으로 폴더 생성
├── config.json               # Neural Field 학습 설정
├── metadata.json             # 학습 메타데이터 (이미지 수, 시간 등)
├── weights.pkl               # 학습된 네트워크 가중치
└── mesh.obj                  # 추출된 3D 메시 (선택적)
```

**학습 명령**:
```bash
python scripts/train_neural_field.py \
    --ref_dir /root/fursys_img_251229/extraction \
    --output_dir models/neural_fields/painting_object \
    --num_iterations 1000
```

### 2.4 yolo 폴더
**역할**: 도장 공정 객체 탐지용 파인튜닝된 YOLO 모델

**현재 파일**:
| 파일명 | 크기 | 설명 |
|--------|------|------|
| `class187_image85286_v12x_250epochs.pt` | 114MB | YOLOv12x 모델, 187 클래스, 85,286 이미지로 250 에포크 학습 |

다운 링크 : https://magenta.dooray.com/project/pages/4221286911063439348 참고

**모델 사양**:
- 아키텍처: YOLOv12x (Ultralytics)
- 클래스 수: 187개 (도장물 종류)
- 학습 이미지: 85,286장
- 학습 에포크: 250
- 신뢰도 임계값 (기본): 0.5

---

## 3. reference_images 폴더 상세

### 3.1 폴더 상태
**현재 상태**: 비어있음 (외부 경로 `/root/fursys_img_251229/extraction` 사용)

### 3.2 외부 참조 이미지 구조
Neural Object Field (Model-Free) 학습을 위한 참조 이미지는 별도 경로에 저장되어 있습니다:

```
/root/fursys_img_251229/extraction/
├── 20251229_093820_front/         # 정면 (0°)
│   ├── rgb/                       # RGB 이미지
│   ├── depth/                     # Depth 이미지 (PNG)
│   └── depth_csv/                 # Depth 이미지 (CSV, 미터 단위)
├── 20251229_094115_45/            # 45°
├── 20251229_094410_90/            # 90°
├── 20251229_094733_135/           # 135°
├── 20251229_094948_180/           # 180°
├── 20251229_095222_225/           # 225°
├── 20251229_095322_270/           # 270°
├── 20251229_150727_315/           # 315°
├── 20251229_151239_topfront/      # 위에서 정면
├── 20251229_151338_top90/         # 위에서 90°
├── 20251229_151622_top270/        # 위에서 270°
├── 20251229_152005_bottomfront/   # 아래에서 정면
├── 20251229_152111_bottom90/      # 아래에서 90°
├── 20251229_152156_bottom270/     # 아래에서 270°
└── 20251229_154835_test/          # 테스트 데이터 (제외)
```

### 3.3 참조 이미지 요구사항
**최소 요구 이미지 수**: 4장 (권장: 8-16장)

**각도 분포**:
- 수평 360° 회전: 8개 각도 (0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°)
- 상단 뷰: 3개 각도 (topfront, top90, top270)
- 하단 뷰: 3개 각도 (bottomfront, bottom90, bottom270)

**이미지 형식**:
- RGB: PNG 또는 JPEG (640x480 권장)
- Depth: PNG (16-bit) 또는 CSV (미터 단위)

### 3.4 reference_images 폴더 사용 시 구조
만약 `reference_images` 폴더를 직접 사용한다면:

```
reference_images/
├── front/                     # 정면 뷰
│   ├── rgb/
│   │   └── image_001.png
│   ├── depth/
│   │   └── depth_001.png
│   └── mask/ (선택적)
│       └── mask_001.png
├── 45/                        # 45° 뷰
├── 90/                        # 90° 뷰
└── ...
```

---

## 4. nimg_v3 코어 모듈

### 4.1 전체 구조
```
nimg_v3/
├── __init__.py               # 패키지 초기화
├── main.py                   # 통합 측정 시스템
├── config/                   # 설정 관리
├── detection/                # 객체 탐지
├── input/                    # 데이터 입력
├── measurement/              # 측정/변환
├── output/                   # 결과 출력
├── pose/                     # 자세 추정
├── tracking/                 # 추적
└── utils/                    # 유틸리티
```

### 4.2 config 모듈
**파일**: `config/system_config.py`

**역할**: 전체 시스템 설정 통합 관리

**주요 클래스**:
```python
@dataclass
class CameraConfig:           # 카메라 파라미터 (RealSense D455)
    fx: float = 383.883       # 초점 거리 X
    fy: float = 383.883       # 초점 거리 Y
    cx: float = 320.499       # 주점 X
    cy: float = 237.913       # 주점 Y
    width: int = 640
    height: int = 480

@dataclass
class DetectionConfig:        # YOLO 탐지 설정
    model_path: str
    conf_threshold: float = 0.5
    iou_threshold: float = 0.45

@dataclass
class PoseEstimationConfig:   # FoundationPose 설정
    model_dir: str
    mode: str = "model_free"
    neural_field_dir: str

@dataclass
class KalmanFilterConfig:     # 칼만 필터 설정
    mode: str = "quaternion"
    process_noise_pos: float = 0.01
    measurement_noise_pos: float = 0.005

@dataclass
class SystemConfig:           # 전체 시스템 설정
    camera: CameraConfig
    detection: DetectionConfig
    pose_estimation: PoseEstimationConfig
    kalman_filter: KalmanFilterConfig
    output: OutputConfig
```

### 4.3 input 모듈
**파일**: `input/data_loader.py`

**역할**: RGB, Depth, IMU 데이터 통합 로드

**주요 클래스**:
```python
@dataclass
class FrameData:              # 단일 프레임 데이터
    frame_idx: int
    rgb: np.ndarray          # [H, W, 3]
    depth: np.ndarray        # [H, W] 미터 단위
    timestamp: float
    accel: Optional[np.ndarray]  # 가속도계 데이터
    gyro: Optional[np.ndarray]   # 자이로스코프 데이터

class DataLoader:             # 데이터 로더
    def __init__(self, data_dir, depth_scale=0.001)
    def load_frame(self, idx) -> FrameData
    def __iter__() -> Iterator[FrameData]
```

**지원 폴더 구조**:
1. 서브폴더 구조: `rgb/`, `depth/`
2. 플랫 구조: `color_*`, `depth_*`

### 4.4 measurement 모듈

#### 4.4.1 pose_converter.py
**역할**: 6DoF 자세 변환 (오일러/쿼터니언/회전행렬)

**설계 원칙** (euler_vs_quaternion_rotation_analysis.md 기반):
- 내부 처리: 쿼터니언 (수치 안정성)
- 외부 인터페이스: 오일러 (직관적)
- ROS 통신: 쿼터니언 (표준)

**주요 클래스 및 함수**:
```python
@dataclass
class EulerAngles:            # 오일러 각도 (도)
    roll: float               # X축 회전
    pitch: float              # Y축 회전
    yaw: float                # Z축 회전

@dataclass
class Quaternion:             # 쿼터니언 (x, y, z, w)
    x, y, z, w: float

class PoseConverter:          # 변환기
    def pose_matrix_to_components(pose_4x4) -> PoseComponents
    def quaternion_to_euler(quat) -> EulerAngles
    def euler_to_quaternion(euler) -> Quaternion
    @staticmethod
    def slerp(q0, q1, t) -> Quaternion  # 구면 선형 보간

# 6D 연속 표현 (신경망 학습용)
def rotation_matrix_to_6d(R) -> np.ndarray
def sixd_to_rotation_matrix(sixd) -> np.ndarray

# 각도 변화 계산
def compute_angle_change(prev_pose, curr_pose, use_quaternion=True)
```

#### 4.4.2 pose_kalman_filter.py
**역할**: 6DoF 자세 Kalman Filter (속도 추정)

**지원 모드**:
| 모드 | 상태 차원 | 상태 벡터 |
|------|----------|-----------|
| EULER | 12 | [x,y,z, vx,vy,vz, roll,pitch,yaw, wx,wy,wz] |
| QUATERNION | 13 | [x,y,z, vx,vy,vz, qx,qy,qz,qw, wx,wy,wz] |

**주요 클래스**:
```python
class FilterMode(Enum):
    EULER = "euler"
    QUATERNION = "quaternion"

@dataclass
class PoseKalmanState:
    position: np.ndarray      # [x, y, z] 미터
    velocity: np.ndarray      # [vx, vy, vz] m/s
    orientation_euler: EulerAngles
    orientation_quat: Quaternion
    angular_velocity: np.ndarray  # [wx, wy, wz] deg/s
    speed: float              # 선속도 크기 m/s

class PoseKalmanFilter:
    def __init__(dt=1/30.0, mode=FilterMode.QUATERNION)
    def initialize(position, orientation)
    def predict() -> PoseKalmanState
    def update(position, orientation) -> PoseKalmanState
    def predict_and_update(...) -> PoseKalmanState
    def update_adaptive_noise(depth_distance)  # 거리 기반 노이즈 조정
```

### 4.5 pose 모듈

#### 4.5.1 foundationpose_estimator.py
**역할**: FoundationPose 기반 6DoF 자세 추정/추적

**지원 모드**:
```python
class PoseMode(Enum):
    MODEL_BASED = "model_based"  # CAD 모델 사용
    MODEL_FREE = "model_free"    # 참조 이미지 + Neural Field

class TrackingState(Enum):
    INITIALIZING = "initializing"
    TRACKING = "tracking"
    LOST = "lost"
```

**주요 클래스**:
```python
@dataclass
class PoseResult:
    pose_matrix: np.ndarray      # 4x4 변환 행렬
    translation: np.ndarray      # [tx, ty, tz]
    rotation_matrix: np.ndarray  # 3x3
    confidence: float            # 신뢰도 (0-1)
    tracking_state: TrackingState
    processing_time_ms: float

class FoundationPoseEstimator:
    def __init__(model_dir, mode, neural_field_dir, device)
    def estimate(rgb, depth, mask, intrinsics) -> PoseResult
    def track(rgb, depth, intrinsics) -> PoseResult
    def process(rgb, depth, mask, intrinsics, force_estimate) -> PoseResult
```

#### 4.5.2 neural_object_field.py
**역할**: Model-Free 방식의 핵심 - 참조 이미지로부터 3D 객체 표현 학습

**구조**:
```
Neural Object Field
├── 기하학 함수 Ω: x → s (SDF)
│   └── 입력: 3D 점 x, 출력: 부호 있는 거리 값 s
└── 외관 함수 Φ: (f, n, d) → c
    └── 입력: 기하학 특징 f, 법선 n, 시선 방향 d
    └── 출력: 색상 c
```

**주요 클래스**:
```python
@dataclass
class NeuralFieldConfig:
    num_iterations: int = 1000
    batch_size: int = 1024
    learning_rate: float = 1e-3
    hidden_dim: int = 128
    num_layers: int = 4
    marching_cubes_resolution: int = 128

class NeuralObjectField:
    def train(reference_images, camera_poses, verbose)
    def extract_mesh(resolution) -> trimesh.Trimesh
    def render(camera_pose, intrinsics, image_size) -> (rgb, depth)
    def save(save_dir)
    def load(load_dir)
```

#### 4.5.3 reference_image_loader.py
**역할**: 참조 이미지 로드 및 관리

**주요 클래스**:
```python
@dataclass
class ReferenceImage:
    rgb: np.ndarray           # [H, W, 3]
    depth: np.ndarray         # [H, W] 미터 단위
    mask: Optional[np.ndarray]
    view_angle: str           # "0", "45", "90", "topfront" 등
    folder_name: str

@dataclass
class ReferenceImageSet:
    images: List[ReferenceImage]
    object_name: str
    camera_intrinsics: Dict

    def get_rgb_stack() -> np.ndarray    # [N, H, W, 3]
    def get_depth_stack() -> np.ndarray  # [N, H, W]

class ReferenceImageLoader:
    def __init__(base_dir, exclude_patterns=['_test'])
    def load_all_views(images_per_view=1) -> ReferenceImageSet
    def load_specific_views(view_angles) -> ReferenceImageSet
    def generate_masks_from_depth(ref_set) -> ReferenceImageSet
```

### 4.6 output 모듈
**파일**: `output/result_exporter.py`

**역할**: 측정 결과를 CSV, JSON으로 내보내기

**주요 클래스**:
```python
class ResultExporter:
    def __init__(output_dir, prefix="nimg_v3", format="csv")
    def add_result(result)
    def save(filename) -> str
    def get_summary() -> Dict
```

### 4.7 main.py - 통합 측정 시스템
**역할**: YOLO + FoundationPose + Kalman Filter 통합 파이프라인

```python
@dataclass
class MeasurementResult:
    # 탐지 정보
    detection_bbox: tuple
    detection_confidence: float
    class_id: int
    class_name: str

    # 자세 정보
    pose_matrix: np.ndarray
    translation: np.ndarray
    euler_angles: EulerAngles
    quaternion: Quaternion

    # Kalman Filter 상태
    filtered_position: np.ndarray
    velocity: np.ndarray
    speed: float
    angular_velocity: np.ndarray

    # 기준 대비 변화량
    position_change: np.ndarray
    angle_change: np.ndarray

class IntegratedMeasurementSystem:
    def __init__(yolo_model_path, foundationpose_model_dir, ...)
    def process_frame(rgb, depth, timestamp) -> MeasurementResult
    def process_sequence(frames) -> List[MeasurementResult]
    def reset()

    @classmethod
    def from_config(config) -> IntegratedMeasurementSystem
```

---

## 5. scripts 폴더 상세

### 5.1 전체 구조
```
scripts/
├── train_neural_field.py                  # Neural Field 학습
├── run_measurement.py                     # 측정 실행
├── test_pipeline.py                       # 기본 파이프라인 테스트
├── test_pipeline_extended.py              # 확장 파이프라인 테스트
├── test_velocity_angle_measurement.py     # 속도/각도 측정 테스트
└── test_absolute_relative_measurement.py  # 절대/상대 측정 테스트
```

### 5.2 train_neural_field.py
**역할**: 참조 이미지로부터 Neural Object Field 학습

**사용법**:
```bash
python scripts/train_neural_field.py \
    --ref_dir /root/fursys_img_251229/extraction \
    --output_dir models/neural_fields/painting_object \
    --num_iterations 1000 \
    --device cuda:0 \
    --generate_masks  # 선택: Depth 기반 마스크 자동 생성
```

**처리 과정**:
1. 참조 이미지 로드 (ReferenceImageLoader)
2. 마스크 생성 (선택적)
3. Neural Field 학습
4. 메시 추출
5. 결과 저장

### 5.3 run_measurement.py
**역할**: 오프라인 데이터에서 6DoF 자세 측정 실행

**사용법**:
```bash
python scripts/run_measurement.py \
    --data_dir /path/to/data \
    --output_dir output \
    --config config.yaml \
    --visualize \
    --max_frames 100
```

### 5.4 test_pipeline.py
**역할**: YOLO 탐지 기본 테스트

**출력**:
- `test_result/detections/`: 탐지 결과
- `test_result/visualizations/`: 시각화 이미지
- `test_result/test_results.json`: 결과 JSON

### 5.5 test_absolute_relative_measurement.py
**역할**: 기준 프레임 대비 절대/상대 위치 및 각도 측정

**출력**:
- `test_result/absolute_relative_measurement/`
  - 위치 변화 플롯
  - 각도 변화 플롯
  - 속도 플롯
  - CSV 데이터
  - JSON 결과
  - 텍스트 요약

---

## 6. tests 폴더 상세

### 6.1 전체 구조
```
tests/
├── __init__.py
├── test_pose_converter.py           # 자세 변환 테스트
├── test_pose_kalman_filter.py       # 칼만 필터 테스트
└── test_reference_image_loader.py   # 참조 이미지 로더 테스트
```

### 6.2 test_pose_converter.py
**테스트 항목**:
- `TestEulerAngles`: 오일러 각도 생성, 배열 변환, 정규화, 뺄셈
- `TestQuaternion`: 단위 쿼터니언, 정규화, 켤레, 곱셈
- `TestPoseConverter`: 항등 자세, 이동, 회전, 왕복 변환
- `TestAngleChange`: 각도 변화 계산
- `Test6DRepresentation`: 6D 연속 표현 왕복 변환
- `TestGimbalLock`: 짐벌 락 감지

### 6.3 test_pose_kalman_filter.py
**테스트 항목**:
- `TestPoseKalmanFilterEuler`: 오일러 모드 초기화, 예측, 업데이트
- `TestPoseKalmanFilterQuaternion`: 쿼터니언 모드 테스트
- `TestKalmanStateOutput`: 상태 출력 형식
- `TestAdaptiveNoise`: 거리 기반 노이즈 조정
- `TestMultipleFrames`: 연속 프레임 추적
- `TestFilterReset`: 필터 리셋

### 6.4 test_reference_image_loader.py
**테스트 항목**:
- 로더 초기화
- 뷰 폴더 탐색
- 모든 뷰 로드
- 참조 이미지 속성 (RGB, Depth, 마스크)
- RGB/Depth 스택 생성
- 폴더명에서 각도 추출
- 제외 패턴 (`_test` 폴더 제외)

### 6.5 테스트 실행
```bash
# 전체 테스트 실행
cd /root/fursys_imgprosessing_ws/src/nimg_v3
pytest tests/ -v

# 특정 테스트 파일 실행
pytest tests/test_pose_converter.py -v

# 커버리지 포함
pytest tests/ -v --cov=nimg_v3
```

**현재 테스트 결과**: 48개 테스트 모두 통과 (PASSED)

---

## 7. 설정 및 의존성 파일

### 7.1 requirements.txt
```
# Core
numpy>=1.20.0
scipy>=1.7.0
opencv-python>=4.5.0
filterpy>=1.4.5      # Kalman Filter
pyyaml>=5.4.0
pandas>=1.3.0

# Deep Learning
torch>=1.10.0
torchvision>=0.11.0

# YOLO
ultralytics>=8.0.0

# 3D Processing
trimesh>=3.9.0       # 메시 처리
open3d>=0.13.0       # 포인트 클라우드
scikit-image>=0.18.0 # Marching Cubes

# Development
pytest>=6.0.0
pytest-cov>=2.0.0
```

### 7.2 setup.py
```python
setup(
    name='nimg_v3',
    version='3.0.0',
    description='FoundationPose-based 6DoF Pose Estimation System',
    packages=find_packages(),
    python_requires='>=3.8',
    install_requires=[...],
    extras_require={
        'full': [...],      # 전체 기능
        'dev': [...],       # 개발용
    },
    entry_points={
        'console_scripts': [
            'nimg_v3_train=scripts.train_neural_field:main',
            'nimg_v3_measure=scripts.run_measurement:main',
        ],
    },
)
```

### 7.3 설치 방법
```bash
# 기본 설치
pip install -e .

# 전체 기능 설치
pip install -e .[full]

# 개발 환경 설치
pip install -e .[dev]
```

---

## 8. 데이터 흐름 요약

```
입력 데이터                    처리 파이프라인                      출력
─────────────────────────────────────────────────────────────────────────
RGB 이미지    ─┬─→  YOLO 탐지  ─→  바운딩 박스, 마스크
Depth 이미지  ─┤
              └─→  FoundationPose  ─→  6DoF 자세 (4x4 행렬)
                        ↓
              PoseConverter  ─→  오일러/쿼터니언/회전행렬
                        ↓
              Kalman Filter  ─→  필터링된 위치, 속도, 각속도
                        ↓
              기준 프레임 비교  ─→  절대/상대 변화량
                        ↓
              ResultExporter  ─→  CSV/JSON 결과 파일
```

---

## 9. 주요 사용 시나리오

### 9.1 시나리오 1: Model-Free 방식으로 새 객체 설정
```bash
# 1. 참조 이미지 촬영 (8-16개 각도)
# 2. Neural Field 학습
python scripts/train_neural_field.py \
    --ref_dir /path/to/reference_images \
    --output_dir models/neural_fields/new_object

# 3. 측정 실행
python scripts/run_measurement.py \
    --data_dir /path/to/test_data \
    --output_dir results
```

### 9.2 시나리오 2: 기존 모델로 테스트
```bash
# Depth 기반 절대/상대 측정 테스트
python scripts/test_absolute_relative_measurement.py
```

### 9.3 시나리오 3: Python API 사용
```python
from nimg_v3.main import IntegratedMeasurementSystem
from nimg_v3.config.system_config import SystemConfig

# 시스템 초기화
config = SystemConfig()
system = IntegratedMeasurementSystem.from_config(config)

# 프레임 처리
result = system.process_frame(rgb, depth)

# 결과 확인
print(f"Position: {result.translation}")
print(f"Yaw: {result.euler_angles.yaw:.2f}°")
print(f"Speed: {result.speed:.3f} m/s")
```

---

## 10. 향후 개선 사항

1. **FoundationPose 실제 통합**: 현재 더미 구현 → NVIDIA 공식 모델 연동
2. **실시간 카메라 지원**: RealSense 실시간 스트림 처리
3. **ROS2 통합**: geometry_msgs/Pose 퍼블리시
4. **GPU 최적화**: TensorRT 변환으로 추론 가속
5. **다중 객체 추적**: 복수 도장물 동시 추적

---

**문서 끝**
