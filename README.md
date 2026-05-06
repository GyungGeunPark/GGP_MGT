# nimg_v3 — FoundationPose 기반 6DoF 자세 추정 시스템

> **버전**: 3.0.0 / v5 통합 파이프라인 (2026-04-20)
> **작성자**: FurSys AI Team
> **목적**: Fursys 산업용 도장(painting) 품질 검사를 위한 실시간 6DoF 자세 추정 + 상대 yaw 신호(-2~+2) 분류 + ROS2 퍼블리시

---

## 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [v5 시스템 아키텍처](#2-v5-시스템-아키텍처)
3. [폴더 구조](#3-폴더-구조)
4. [환경 설정 및 설치](#4-환경-설정-및-설치)
5. [핵심 모듈 상세](#5-핵심-모듈-상세)
6. [실행 스크립트](#6-실행-스크립트)
7. [설정 (Config)](#7-설정-config)
8. [데이터 흐름](#8-데이터-흐름)
9. [ROS2 토픽](#9-ros2-토픽)
10. [테스트](#10-테스트)
11. [디자인 패턴](#11-디자인-패턴)
12. [v1→v5 핵심 교훈](#12-v1v5-핵심-교훈)

---

## 1. 프로젝트 개요

`nimg_v3`는 Intel RealSense D455 깊이 카메라로 컨베이어 위 트레이의 6자유도(6DoF) 자세를 추정하고, 기준 자세 대비 상대 yaw 각도를 이산 신호(`-2, -1, 0, +1, +2`)로 분류해 ROS2 토픽(`p_s`)으로 퍼블리시하는 시스템입니다.

### 주요 특징

| 특징 | 설명 |
|------|------|
| **실시간 6DoF 추정** | NVIDIA FoundationPose GPU 추론 (~30 ms) |
| **Zero-Shot 일반화** | CAD 메쉬 + 텍스처 없이 flat-gray 패딩으로 동작 |
| **멀티 객체 트래킹** | per-instance FP++ 팩토리, IoU + center-dist 재연결 |
| **하이브리드 회전 표현** | 내부 쿼터니언, 외부 오일러, ROS는 쿼터니언 |
| **그레이스풀 디그레이드** | SAM3.1 → SAM2.1 → YOLO-seg → Depth-bbox 자동 폴백 |
| **이산 신호 분류** | yaw 임계값 기반 -2/-1/0/+1/+2/none (CW=+) |
| **Blackwell GPU 지원** | RTX 5080 (sm_120) — env-local CUDA 12.8 으로 빌드 |

### 세대별 위치

| 세대 | 경로 | 상태 |
|------|------|------|
| nimg (v1) | `nimg/` | YOLOv5 + ORB + SQL DB. legacy |
| nimg_v2 | `src/nimg_v2/` | PCA + 9-state CA Kalman. legacy 알고리즘 모듈 |
| **nimg_v3 (현재)** | `src/nimg_v3/` | NOCTIS + FoundationPose + 13-state Quaternion KF |

---

## 2. v5 시스템 아키텍처

[src/research/260420/](../research/260420/) 설계 문서 기반 4-layer 구조.

```
┌────────────────────────────────────────────────────────────────────┐
│  Input Adapter (PairedVideoSource | RealSenseSource)               │
│  └─→ Frame {rgb, depth_m, K, timestamp = frame_idx/fps}            │
└─────────────────────────────┬──────────────────────────────────────┘
                              │
┌─────────────────────────────▼──────────────────────────────────────┐
│  Layer A — FastROI                              (~2 ms)            │
│  • depth 평면 제거 + 전경 bbox 후보                                  │
└─────────────────────────────┬──────────────────────────────────────┘
                              │
┌─────────────────────────────▼──────────────────────────────────────┐
│  Layer B — NOCTISRecognizer                     (~22 ms)           │
│  • SAM3.1 / SAM2.1 / YOLO-seg → 마스크 제안                         │
│  • DINOv3-L / DINOv2-L / HSV-hist → CLS 임베딩                      │
│  • TemplateDB (N=84, D=1024) cosine 매칭 + cyclic threshold        │
│  • IoU + center-dist 기반 prior track 재연결                        │
└─────────────────────────────┬──────────────────────────────────────┘
                              │
┌─────────────────────────────▼──────────────────────────────────────┐
│  Layer C — FoundationPose++                     (~30 ms)           │
│  • per-instance FoundationPosePlusPlus 팩토리                       │
│    ├─ RealFPEstimator (NVlabs FoundationPose, GPU)                 │
│    │   └─ mesh 텍스처 없으면 32×32 flat-gray 패딩                    │
│    ├─ IoUTracker2D (CSRT + fresh bbox 매 프레임 reset)              │
│    ├─ PoseHypothesisKF (13-state quaternion pre-filter)            │
│    └─ LostDetector (tracker_fail_frames=5 grace)                   │
└─────────────────────────────┬──────────────────────────────────────┘
                              │
┌─────────────────────────────▼──────────────────────────────────────┐
│  Layer D — Measurement                          (<1 ms)            │
│  • BaselineLoader: reference_config.yaml 의 0° 이미지로 T_ref 사전계산│
│  • PCA 부호 모호성 2단 해결 (CW=+)                                   │
│  • ShapeBasedSignalGenerator: yaw 임계값 → -2..+2, 99=none          │
└─────────────────────────────┬──────────────────────────────────────┘
                              │
┌─────────────────────────────▼──────────────────────────────────────┐
│  ROS2 Publisher: 'p_s' 토픽                                         │
│  {signal, uid, conf, relative_yaw, position, v, ω}                 │
└────────────────────────────────────────────────────────────────────┘
```

### 통합 파이프라인 클래스

`Nimg3Pipeline.step(frame)` — streaming + batch 공통 사용 ([nimg_v3/pipeline.py](nimg_v3/pipeline.py))

```python
@dataclass
class FrameResult:
    frame: Frame
    tracks: List[dict]           # uid, class_id, hyp, ry, signal, v, ω, bbox, mask
    current_objs: List[RecognizedObject]
    rois: List[BBox]
    viz: Optional[np.ndarray]    # 시각화 이미지 + status panel
    latency_ms: dict             # {A, B, C_mean, D_mean}
    reinit_events: int
```

---

## 3. 폴더 구조

```
src/nimg_v3/
├── README.md                        # 이 문서
├── setup.py                         # 패키지 설치 스크립트
├── docs/
│   ├── NIMG_V3_STRUCTURE_ANALYSIS.md
│   └── pointcloud_interface_guide.md
├── nimg_v3/                         # 코어 Python 패키지
│   ├── __init__.py
│   ├── main.py                      # IntegratedMeasurementSystem (오프라인)
│   ├── pipeline.py                  # Nimg3Pipeline (v5 통합)
│   ├── common/
│   │   └── data_types.py            # Frame, BBox, RecognizedObject, PoseHypothesis, MeasurementResult
│   ├── config/
│   │   ├── system_config.py         # 모든 dataclass 설정
│   │   └── default_config.yaml      # 기본 YAML 설정
│   ├── input/
│   │   ├── adapters.py              # PairedVideoSource, RealSenseSource (InputAdapter)
│   │   ├── data_loader.py           # 폴더 기반 RGB/Depth/IMU 로더
│   │   ├── depth_io.py              # 16-bit depth PNG/NPZ 입출력
│   │   └── realsense_filters.py     # spatial+temporal+hole_filling
│   ├── recognition/                 # Layer B
│   │   ├── fast_roi.py              # depth 평면 제거 (Layer A)
│   │   ├── sam_segmenter.py         # SAM3.1/2.1 + YOLO-seg + DepthBox 폴백
│   │   ├── dinov3_encoder.py        # DINOv3 + DINOv2 + HSV-hist 폴백
│   │   ├── template_db.py           # 템플릿 DB (N×D, cosine search)
│   │   ├── template_builder.py      # icosphere 42뷰 렌더 + DINO 인코딩
│   │   └── noctis_pipeline.py       # NOCTISRecognizer 통합
│   ├── tracker/                     # Layer C
│   │   ├── fp_estimator.py          # RealFPEstimator + FallbackFPEstimator (factory)
│   │   ├── fp_plus_plus.py          # FoundationPosePlusPlus (FP++)
│   │   ├── tracker_2d.py            # IoUTracker2D (CSRT)
│   │   ├── pose_hypothesis_kf.py    # 13-state Quaternion pre-filter KF
│   │   ├── lost_detector.py         # 추적 실패 감지 + grace period
│   │   ├── hierarchical_refine.py   # n_hyp 다가설 refine (선택)
│   │   └── baseline_loader.py       # reference_config.yaml → T_ref 사전계산
│   ├── pose/                        # 레거시 + 측정 보조
│   │   ├── foundationpose_estimator.py  # 통합 래퍼 (estimate/track/process 자동 전환)
│   │   ├── foundationpose_wrapper.py    # NVIDIA FP 실제 래퍼
│   │   ├── real_foundationpose.py       # FP 대안 구현
│   │   ├── neural_object_field.py       # placeholder (메시 생성은 generate_mesh.py)
│   │   └── reference_image_loader.py    # 14방향 뷰, depth 마스크 자동 생성
│   ├── measurement/                 # Layer D
│   │   ├── pose_converter.py        # Euler↔Quaternion↔RotationMatrix↔6D, SLERP
│   │   └── pose_kalman_filter.py    # 12-state Euler / 13-state Quaternion KF
│   ├── pointcloud/                  # PointCloud 캡처/퍼블리시 (PyQt5 GUI)
│   │   ├── camera_wrapper.py        # RealSenseCameraWrapper
│   │   ├── capture_manager.py       # SOR + voxel + normal estimation
│   │   ├── visualizer.py            # Open3D 3D 뷰어
│   │   ├── ros2_publisher.py        # ROS2PointCloudPublisher (Singleton)
│   │   └── gui.py                   # PointCloudInterfaceGUI (PyQt5)
│   ├── detection/                   # nimg_v2 YOLODetector 임포트
│   ├── tracking/                    # 빈 모듈 (FP++ 내부 위임)
│   ├── utils/                       # 빈 모듈
│   └── output/
│       └── result_exporter.py       # CSV/JSON 내보내기
├── scripts/
│   ├── streaming/
│   │   ├── realsense_6dof_stream.py        # 실시간 D455 (--backend v5/legacy)
│   │   ├── realsense_6dof_stream_video.py  # 비디오 파일 재생 (오프라인)
│   │   └── pointcloud_interface.py         # PyQt5 PointCloud GUI
│   ├── eval/
│   │   ├── video_benchmark.py       # /video/ 폴더 6쌍 벤치마크
│   │   ├── make_report.py           # 차트 + REPORT.md + 시계열 플롯
│   │   ├── run_ablation.py          # v6 토글별 ablation
│   │   └── make_ablation_report.py
│   ├── mesh_tools/
│   │   ├── clean_mesh.py            # 메시 정제
│   │   └── bake_texture.py          # 텍스처 베이크
│   ├── build_templates.py           # 템플릿 DB 빌드 (icosphere 42뷰)
│   ├── generate_mesh.py             # TSDF Fusion / Poisson 메시 생성
│   ├── train_neural_field.py        # Neural Field 학습 (legacy)
│   ├── run_measurement.py           # 오프라인 측정
│   ├── test_pipeline.py             # YOLO 탐지 테스트
│   ├── test_pipeline_extended.py    # 전체 파이프라인 테스트
│   ├── test_velocity_angle_measurement.py
│   └── test_absolute_relative_measurement.py
├── tests/                           # 48개 단위 테스트
│   ├── test_pose_converter.py
│   ├── test_pose_kalman_filter.py
│   └── test_reference_image_loader.py
└── models/
    ├── foundationpose/              # FoundationPose 가중치 (scorer/refiner)
    ├── neural_fields/
    │   ├── housing_M/reference_images/  # baseline_angles + reference_config.yaml
    │   ├── Wiring_tray/reference_images/
    │   └── template_db.npz          # N=84, D=1024 (DINOv2-L)
    └── yolo/
        ├── class187_image85286_v12x_250epochs.pt  # 187클래스 범용
        ├── Yolo_v11_seg_m_top.pt    # tray_top 세그멘테이션
        └── yolo26_2class_seg_best_260324.pt        # 2클래스 fallback
```

---

## 4. 환경 설정 및 설치

### 환경 매트릭스 (2026-04-20 기준)

| env | Python | torch | CUDA | pytorch3d | nvdiffrast | sm_120(RTX 5080) | FoundationPose | 용도 |
|-----|--------|-------|------|-----------|------------|------------------|----------------|------|
| **fp (기본)** | 3.11 | 2.7.0+cu128 | env-local 12.8 | 0.7.8 | 0.4.0 | OK | OK | v5 메인 |
| yolo26 | 3.12 | 2.12-dev+cu128 | host 12.1 | - | - | OK | - | YOLO 학습 |
| my | 3.8 | 2.1.0+cu121 | host 12.1 | 0.7.6 | 0.3.1 | - | kernel mismatch | 과거 FP env |

> **GPU**: NVIDIA GeForce RTX 5080 (Blackwell, sm_120, 15 GB)
> **호스트 nvcc 12.1 미지원 → fp env 내부에 CUDA 12.8 toolkit 별도 설치**

### 중요 환경 변수

```bash
export CUDA_HOME=/opt/conda/envs/fp
export PATH=/opt/conda/envs/fp/bin:$PATH    # env-local nvcc 우선
export TORCH_CUDA_ARCH_LIST="8.0;8.6;9.0;12.0+PTX"
export FORCE_CUDA=1
```

### fp env 신규 머신 재현

```bash
# 1. env + torch 2.7+cu128 (Blackwell sm_120 지원)
conda create -n fp python=3.11 -y
conda run -n fp pip install torch==2.7.0 torchvision \
    --index-url https://download.pytorch.org/whl/cu128

# 2. env-local CUDA 12.8 toolkit (호스트 12.1 우회)
conda install -n fp -c nvidia/label/cuda-12.8.0 -y \
    cuda-nvcc cuda-cudart cuda-libraries-dev cuda-nvtx

# 3. pytorch3d + nvdiffrast 소스 빌드
conda run -n fp bash -c '
  export CUDA_HOME=/opt/conda/envs/fp
  export PATH=/opt/conda/envs/fp/bin:$PATH
  export TORCH_CUDA_ARCH_LIST="8.0;8.6;9.0;12.0+PTX"
  export FORCE_CUDA=1
  pip install fvcore iopath
  pip install --no-build-isolation \
      git+https://github.com/facebookresearch/pytorch3d.git@stable
  pip install --no-build-isolation git+https://github.com/NVlabs/nvdiffrast.git
'

# 4. 프로젝트 의존성
conda run -n fp pip install \
    trimesh pyrender pymeshlab faiss-cpu transformers \
    open3d scikit-image imageio imageio-ffmpeg ultralytics filterpy \
    warp-lang joblib transformations kornia h5py ruamel.yaml \
    timm einops accelerate pyrealsense2

# 5. FoundationPose mycpp (py3.11 재빌드)
cd src/FoundationPose/mycpp && mkdir -p build && cd build
conda run -n fp bash -c '
  cmake .. -DCMAKE_PREFIX_PATH=$(python -c "import torch; print(torch.utils.cmake_prefix_path)")
  make -j4
'

# 6. nimg_v3 editable install
cd /root/rvc_scan_ws/src/nimg_v3
conda run -n fp pip install -e .
```

### 의존성 (setup.py)

| 카테고리 | 패키지 |
|----------|--------|
| **Core** | numpy, scipy, opencv-python, filterpy, pyyaml, pandas |
| **AI/ML (`[full]`)** | torch, torchvision, ultralytics (YOLO v11/v12), trimesh, scikit-image |
| **3D** | open3d, trimesh, pyrender, pymeshlab |
| **GUI** | PyQt5 |
| **Hardware** | pyrealsense2 (Intel RealSense D455) |
| **ROS2** | rclpy, sensor_msgs, std_msgs |
| **GPU 가속** | cupy (선택), nvdiffrast (FP 렌더, 선택) |
| **Dev (`[dev]`)** | pytest, pytest-cov |

---

## 5. 핵심 모듈 상세

### 5.1 `common/data_types.py` — 핵심 데이터 타입

```python
@dataclass
class BBox:
    x1: int; y1: int; x2: int; y2: int; conf: float = 1.0
    @property cx, cy, w, h, area
    def iou(other) -> float

@dataclass
class Frame:
    rgb: np.ndarray              # (H, W, 3) uint8 BGR
    depth: np.ndarray            # (H, W)    float32 meter
    K: np.ndarray                # (3, 3)
    timestamp: float             # frame_idx / fps (비디오 시간)
    frame_idx: int

@dataclass
class RecognizedObject:
    object_uid: int
    class_id: int
    bbox: BBox
    mask: Optional[np.ndarray]   # (H, W) bool
    ref_view_idx: int = -1       # DINOv3 top-1 뷰 인덱스 (coarse pose 힌트)
    match_score: float           # cosine 유사도
    source: str                  # "noctis" | "cnos" | "yolo" | "depth_bbox"

@dataclass
class PoseHypothesis:
    T: np.ndarray                # (4, 4) 카메라 좌표계
    score: float
    source: str                  # "init" | "track" | "reinit"
    timestamp: float
    velocity: Optional[np.ndarray]          # (3,) m/s
    angular_velocity: Optional[np.ndarray]  # (3,) rad/s

@dataclass
class MeasurementResult:
    frame_idx: int; timestamp: float
    class_id: int; object_uid: int
    T: np.ndarray; score: float
    relative_T: np.ndarray
    relative_yaw_deg, relative_pitch_deg, relative_roll_deg: float
    signal: int                  # -2..+2, 99 = 'none'
    linear_velocity_ms: float
    angular_velocity_rads: float
    latency_A_ms, B_ms, C_ms, D_ms: float
    source: str
```

### 5.2 `input/adapters.py` — 통일 입력

| 클래스 | 용도 | 핵심 동작 |
|--------|------|-----------|
| `InputAdapter` (ABC) | 공통 인터페이스 | `start()`, `read() → Frame`, `stop()`, `get_K()` |
| `PairedVideoSource` | RGB+Depth MP4 쌍 | `_idx / fps` 타임스탬프, BGR depth → 0-5m 선형 매핑, 선택적 bilateral denoise |
| `RealSenseSource` | D455 실시간 | aligned RGB+Depth, librealsense spatial+temporal+hole_filling 후처리 |
| `discover_video_pairs()` | `*_rgb_*.mp4` ↔ `*_depth_*.mp4` 자동 페어링 | tag 추출 |

**핵심 교훈**: `timestamp = frame_idx / fps` (wall-clock 사용 시 KF dt 가 잘못되어 속도/각도 0 으로 고정됨)

### 5.3 `recognition/` — Layer B

#### 5.3.1 `FastROI` (Layer A)

depth 히스토그램 peak → 전경 마스크 → connected component → bbox 후보 (~2 ms).

#### 5.3.2 `SAM2Segmenter` / `YoloSegSegmenter` / `DepthBoxSegmenter`

**자동 폴백 체인**: SAM 3.1 → SAM 2.1 → YOLO-seg → Depth-bbox
- SAM 가중치 미존재 시 `RuntimeError` 발생 → 폴백
- YOLO-seg 가 직접 class 를 주면 NOCTIS 매칭 결과보다 우선

#### 5.3.3 `DinoV3Encoder` / `HsvHistEncoder`

**자동 폴백 체인**: DINOv3-vitl16 → DINOv2-L → HSV 8×8×8 (512-dim) histogram
- transformers 최신 버전은 negative-stride numpy 거부 → 연속 배열 복사
- L2-normalize 후 cosine search

#### 5.3.4 `NOCTISRecognizer`

```
SAM proposals → crop (224×224) → DINO encode → TemplateDB.search (top-1)
   → cyclic threshold (매칭 없으면 τ-=0.05 재시도)
   → IoU NMS (nms_iou=0.5)
   → List[RecognizedObject]
```

### 5.4 `tracker/` — Layer C

#### 5.4.1 `RealFPEstimator` / `FallbackFPEstimator`

**Real (NVlabs FoundationPose)**:
- `register(rgb, depth, K, mask, iter_n=8)` → (T, score)
- `track_one(rgb, depth, T_prev, omega, dt)` → (T, score)
- 메시 텍스처 없으면 32×32 flat-gray 패딩 (XY-planar UV)

**Fallback (PCA + depth centroid)**:
- 실 FP 빌드 실패 시 자동 활성화
- 검증용으로 동작은 하나 정확도 ↓
- `depth threshold > 0.01` (0.15 → 0.01 변경 — MP4 인코딩된 0~0.28m depth 처리)

#### 5.4.2 `FoundationPosePlusPlus`

```python
class FoundationPosePlusPlus:
    def on_recognized(frame, obj) -> PoseHypothesis        # 초기 등록
    def track(frame, prev, fresh_mask, fresh_bbox)
        -> Tuple[PoseHypothesis, reinit_needed]
```

**구성 요소**:
1. **fresh_bbox > 2D tracker > prev_T projection** 우선순위로 bbox 결정
2. **Robust depth sample** (5×5 patch median, 0.15-3.5m 유효 범위)
3. **PoseHypothesisKF** 13-state quaternion pre-filter — KF 예측으로 흔들림 억제
4. **HierarchicalRefiner** (n_hyp=1 또는 2개 다가설 refine)
5. **SLERP 25%** orientation smoothing — fresh_mask 있을 때 translation 신뢰, 방위 heavy smooth
6. **Try/except for singular matrix** — bbox edge case 시 prev_T 유지 + score×0.8
7. **LostDetector** — 5 프레임 grace period 적용 (잦은 reinit 방지)

#### 5.4.3 `BaselineLoader`

`reference_config.yaml` 의 `baseline_angles[0]` 또는 `baseline_image` 또는 `signal_mapping[0]` 키에서 0° 이미지를 찾아 FP 통과 → `T_ref` 사전 계산. 모든 상대 yaw 계산의 기준점.

### 5.5 `measurement/` — Layer D

#### 5.5.1 `pose_converter.py`

```python
class RotationOrder(Enum):
    XYZ ('xyz')   # Roll-Pitch-Yaw 표준
    ZYX ('zyx')
    INTRINSIC_XYZ ('XYZ')
    ...

@dataclass class EulerAngles: roll, pitch, yaw  # degrees
@dataclass class Quaternion: x, y, z, w
@dataclass class PoseComponents: translation, euler, quaternion, rotation_matrix

class PoseConverter:
    pose_matrix_to_components(T_4x4) -> PoseComponents
    quaternion_to_euler(q) -> EulerAngles
    euler_to_quaternion(e) -> Quaternion
    @staticmethod slerp(q0, q1, t) -> Quaternion          # 구면 선형 보간
    detect_gimbal_lock(pitch_deg, threshold=85.0) -> bool

# 6D continuous representation (Zhou et al. CVPR 2019)
def rotation_matrix_to_6d(R) -> ndarray
def sixd_to_rotation_matrix(sixd) -> ndarray
def compute_angle_change(prev_pose, curr_pose, use_quaternion=True)
```

#### 5.5.2 `pose_kalman_filter.py`

| 모드 | 상태 차원 | 상태 벡터 |
|------|-----------|-----------|
| `EULER` | 12 | `[x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]` |
| `QUATERNION` | 13 | `[x, y, z, vx, vy, vz, qx, qy, qz, qw, wx, wy, wz]` |

**적응형 노이즈**: `update_adaptive_noise(depth_distance)` 거리 비례로 measurement noise 조정 (D455 노이즈 모델)
**연속성 보장**: 쿼터니언 부호 정렬, Euler wrap-around 방지

### 5.6 `pose/` (레거시 통합 래퍼)

#### `FoundationPoseEstimator`

```python
class TrackingState(Enum): INITIALIZING, TRACKING, LOST
class PoseMode(Enum): MODEL_BASED, MODEL_FREE

@dataclass class PoseResult:
    pose_matrix: ndarray (4,4)
    translation: ndarray
    rotation_matrix: ndarray (3,3)
    confidence: float
    tracking_state: TrackingState
    processing_time_ms: float

class FoundationPoseEstimator:
    estimate(rgb, depth, mask, intrinsics) -> PoseResult   # 초기화/재초기화
    track(rgb, depth, intrinsics) -> PoseResult            # 추적 (mask 불필요)
    process(...)  # 자동 전환 + LOST 시 재초기화
```

상태 머신: `INITIALIZING → TRACKING → LOST → 복구(estimate)`

### 5.7 `pointcloud/` — D455 PointCloud GUI

| 클래스 | 역할 |
|--------|------|
| `RealSenseCameraWrapper` | depth 필터 체인, aligned RGB+Depth 캡처 |
| `PointCloudCaptureManager` | SOR (Statistical Outlier Removal) → voxel downsample → normal estimation → `.ply` 저장 |
| `PointCloudVisualizer` | Open3D 3D 뷰어 |
| `ROS2PointCloudPublisher` | PointCloud2 `pc_point` 토픽 (Singleton) |
| `PointCloudInterfaceGUI` | PyQt5 GUI: 카메라 프리뷰, 캡처, 퍼블리시, 시각화 |

---

## 6. 실행 스크립트

### 6.1 프로덕션 실시간 스트리밍 (가장 중요)

`--backend {v5, legacy}` 옵션 지원. 기본은 `v5` (NOCTIS + 실 FoundationPose + 멀티트래킹).

```bash
# 실시간 RealSense D455 (fp env, v5 기본)
python src/nimg_v3/scripts/streaming/realsense_6dof_stream.py
python src/nimg_v3/scripts/streaming/realsense_6dof_stream.py --backend legacy
python src/nimg_v3/scripts/streaming/realsense_6dof_stream.py --no-ros --no-roi
python src/nimg_v3/scripts/streaming/realsense_6dof_stream.py --save output.mp4

# 비디오 파일 재생 (오프라인 검증)
python src/nimg_v3/scripts/streaming/realsense_6dof_stream_video.py \
    --rgb-video video/output_video_20260323_155751_test4_rgb_30fps.mp4 \
    --depth-video video/output_video_20260323_155751_test4_depth_30fps.mp4 \
    --no-loop --no-display --save test_result/v5_test4.mp4

# PointCloud 캡처/퍼블리시 GUI
python src/nimg_v3/scripts/streaming/pointcloud_interface.py
python src/nimg_v3/scripts/streaming/pointcloud_interface.py --save-dir /path/to/data --no-ros2
```

**키보드 컨트롤** (스트리밍 윈도우):
- `q` / `ESC`: 종료
- `s`: 현재 프레임 이미지 저장
- `r`: baseline 리셋
- `p`: 일시정지/재개
- `f`: 풀스크린 토글

### 6.2 v5 파이프라인 배치 (오프라인 검증)

```bash
# 템플릿 DB 빌드 (icosphere 42뷰 × 2 클래스 = 84개) — 1회만
python src/nimg_v3/scripts/build_templates.py --encoder dinov2-large

# /video/ 6쌍 벤치마크
python src/nimg_v3/scripts/eval/video_benchmark.py --pairs all \
    --max-frames 400 --stride 3 --viz-every 3

# 리포트 생성 (차트 + REPORT.md + 시계열 플롯)
python src/nimg_v3/scripts/eval/make_report.py
# → 결과: test_result/260420/

# v6 ablation 실행
python src/nimg_v3/scripts/eval/run_ablation.py
python src/nimg_v3/scripts/eval/make_ablation_report.py
```

### 6.3 메시 생성 (FoundationPose Model-Free)

```bash
# housing_M 메시 생성 (TSDF Fusion, 기본)
python src/nimg_v3/scripts/generate_mesh.py \
    --data_dir src/nimg_v3/models/neural_fields/housing_M/reference_images

# 수평 뷰만 사용 (top/bottom 제외, 더 안정적)
python src/nimg_v3/scripts/generate_mesh.py \
    --data_dir <path> --horizontal_only

# Poisson Surface Reconstruction
python src/nimg_v3/scripts/generate_mesh.py \
    --data_dir <path> --method pointcloud

# 포인트 클라우드 같이 저장
python src/nimg_v3/scripts/generate_mesh.py \
    --data_dir <path> --save_pointcloud
```

### 6.4 오프라인 측정 / 학습

```bash
# Neural Object Field 학습 (legacy, placeholder)
python src/nimg_v3/scripts/train_neural_field.py --ref_dir <path> --output_dir <path>

# 오프라인 측정
python src/nimg_v3/scripts/run_measurement.py \
    --data_dir <path> --output_dir <path> --visualize

# 단계별 테스트 스크립트
python src/nimg_v3/scripts/test_pipeline.py                       # YOLO 탐지
python src/nimg_v3/scripts/test_pipeline_extended.py              # 전체
python src/nimg_v3/scripts/test_velocity_angle_measurement.py     # 속도/각도
python src/nimg_v3/scripts/test_absolute_relative_measurement.py  # 절대/상대
```

### 6.5 Python API 사용 예시

```python
# v5 통합 파이프라인 (권장)
from nimg_v3.pipeline import Nimg3Pipeline
from nimg_v3.input import PairedVideoSource
from nimg_v3.config.system_config import RecognitionConfig, TrackerConfig

source = PairedVideoSource("rgb.mp4", "depth.mp4")
source.start()
pipeline = Nimg3Pipeline(
    template_db_path="models/neural_fields/template_db.npz",
    reco_cfg=RecognitionConfig(),
    tracker_cfg=TrackerConfig(),
    class_mesh_paths={0: "models/neural_fields/housing_M/mesh.obj",
                      1: "models/neural_fields/Wiring_tray/mesh.obj"},
)
while True:
    frame = source.read()
    if frame is None: break
    result = pipeline.step(frame, draw=True)
    for tr in result.tracks:
        print(f"uid={tr['uid']} signal={tr['signal']} yaw={tr['ry']:.1f}°")

# 레거시 오프라인 시스템
from nimg_v3.main import IntegratedMeasurementSystem
from nimg_v3.config.system_config import SystemConfig
config = SystemConfig()
system = IntegratedMeasurementSystem.from_config(config)
result = system.process_frame(rgb, depth)
print(f"Position: {result.translation}")
print(f"Yaw: {result.euler_angles.yaw:.2f}°")
print(f"Speed: {result.speed:.3f} m/s")
```

---

## 7. 설정 (Config)

### 7.1 SystemConfig 통합 dataclass ([config/system_config.py](nimg_v3/config/system_config.py))

```python
@dataclass class SystemConfig:
    camera: CameraConfig                # D455 fx/fy/cx/cy, fps, depth_scale
    detection: DetectionConfig          # YOLO conf/iou/img_size
    pose_estimation: PoseEstimationConfig    # FP model_dir, mode, mesh, neural_field
    kalman_filter: KalmanFilterConfig   # Euler/Quaternion 모드, process/measure noise
    recognition: RecognitionConfig      # NOCTIS sam_version, encoder, template_db_root
    tracker: TrackerConfig              # FP++ backend, tracker_2d, KF, FP weights
    optim: V6OptimizationFlags          # v6 토글 (raw_depth, mesh, MEKF, hierarchical, ...)
    output: OutputConfig                # CSV/JSON, visualize, log_level
    device: str = "cuda:0"
    reference_frame_idx: int = 0
```

### 7.2 RecognitionConfig 주요 필드

| 필드 | 기본값 | 설명 |
|------|--------|------|
| `backend` | `"noctis"` | `noctis`/`cnos`/`yolo`/`depth_bbox`/`disabled` |
| `sam_version` | `"sam3.1"` | `sam3.1`/`sam2.1`/`disabled` |
| `encoder` | `"dinov3-vitl16"` | `dinov3-vitl16`/`dinov3-vitb16`/`dinov2-large`/`hsvhist` |
| `template_db_root` | `"src/nimg_v3/models/neural_fields"` | 템플릿 DB 루트 |
| `match_threshold` | 0.35 | DINO cosine 임계값 |
| `cyclic_threshold_step` | 0.05 | NOCTIS 전용 |
| `nms_iou` | 0.5 | bbox NMS |
| `roi` | None | `(x1, y1, x2, y2)` |
| `yolo_fallback_path` | `models/yolo/yolo26_2class_seg_best_260324.pt` | Tier-1 YOLO fallback |

### 7.3 TrackerConfig 주요 필드

| 필드 | 기본값 | 설명 |
|------|--------|------|
| `backend` | `"fp_plus_plus"` | `fp_plus_plus`/`fp`/`pca` |
| `tracker_2d` | `"iou"` | `dam4sam`/`him2sam`/`samurai`/`cutie`/`ostrack`/`iou` |
| `track_refine_iter` | 3 | FP refine 반복 |
| `est_refine_iter` | 8 | FP register 반복 |
| `kf_measurement_noise_scale` | 0.05 | pre-filter KF |
| `lost_score_threshold` | 0.4 | LostDetector |
| `lost_score_frames` | 3 | grace period |
| `periodic_reinit_frames` | 150 | 주기적 재초기화 |
| `fp_weights_root` | `models/foundationpose` | scorer/refiner 디렉토리 |

### 7.4 V6OptimizationFlags (Top-5 최적화 토글)

| Rank | 필드 | 기본값 | 설명 |
|------|------|--------|------|
| 1 | `use_raw_depth` | False | 16-bit PNG/NPZ depth |
| 1 | `depth_denoise_bilateral` | False | bilateral filter |
| 1 | `use_realsense_filters` | True | librealsense 후처리 |
| 2 | `use_textured_mesh` | False | `*_textured.obj` 우선 |
| 2 | `use_symmetry_tfs` | False | reference_config 의 symmetry 적용 |
| 3 | `use_mekf` | False | MEKF (placeholder) |
| 4 | `use_hierarchical_refine` | False | n_hyp 다가설 |
| 4 | `hierarchical_n_hyp` | 2 | |
| 5 | `track_refine_iter_v6` | 5 | 3→5 |
| 5 | `render_resolution` | 160 | FP 렌더 해상도 |

### 7.5 프로덕션 스트리밍 핵심 상수 (`realsense_6dof_stream.py`)

```python
CONFIDENCE_THRESHOLD = 0.88   # YOLO confidence
IOU_THRESHOLD = 0.15
MAX_LOST_FRAMES = 60
ROI = (225, 110) ~ (345, 220)
NONE_BOUNDARY = yaw < -36° OR yaw > +46°
```

### 7.6 신호 임계값 (CW=+ convention)

```python
def classify_signal(yaw_deg, thresholds):
    t = {"s2": 25.0, "s1": 12.0, "s0": 6.0,
         "none_lo": -36.0, "none_hi": 46.0}
    if yaw_deg < -36 or yaw_deg > 46:    return 99   # none
    if yaw_deg <= -25:                   return -2
    if yaw_deg <= -12 or yaw_deg < -6:   return -1
    if yaw_deg < 6:                      return 0
    if yaw_deg < 12 or yaw_deg < 25:     return 1
    return 2
```

---

## 8. 데이터 흐름

### 8.1 v5 파이프라인 (Streaming + Batch 공통)

```
RealSense D455 / MP4 쌍
  └─→ InputAdapter.read() → Frame{rgb, depth_m, K, ts=idx/fps}
        │
        ▼
   Layer A: FastROI.process(frame)
        │ ROI bbox 후보 (~2 ms)
        ▼
   Layer B: NOCTISRecognizer.recognize(frame, prior_bboxes)
        │ SAM → DINO → TemplateDB → cyclic threshold → NMS
        │ List[RecognizedObject]                          (~22 ms)
        ▼
   IoU + center-dist 재연결 (active tracks ↔ current_objs)
        │ assigned: Dict[uid, RecognizedObject]
        │ unused: List[RecognizedObject] → 신규 트랙 생성
        ▼
   Layer C: per-instance FoundationPosePlusPlus.track(frame, prev_hyp,
                                                       fresh_mask, fresh_bbox)
        │ FP register/track_one + 2D tracker + pre-KF + LostDetector  (~30 ms)
        │ PoseHypothesis{T, score, velocity, ω}
        ▼
   Layer D: Measurement
        │ T_ref = BaselineLoader 로 사전 계산
        │ relative_ypr_deg(T, T_ref) → (rr, rp, ry)
        │ classify_signal(ry) → -2..+2 / 99
        ▼
   ROS2.publish_signal('p_s', JSON)
   FrameResult{frame, tracks, current_objs, viz, latency_ms}
```

### 8.2 레거시 오프라인 (`main.py`)

```
RGB+Depth → YOLO 탐지 → FoundationPose 6DoF (4×4 행렬)
  → PoseConverter (Euler/Quaternion/6D)
  → Kalman Filter (12/13-state)
  → 기준 프레임 대비 변화량 (position_change, angle_change)
  → ResultExporter (CSV/JSON)
```

---

## 9. ROS2 토픽

| 토픽 | 메시지 타입 | 페이로드 |
|------|------------|---------|
| `p_s` | `std_msgs/String` (JSON) | `{signal, object_id, confidence, relative_yaw, position}` |
| `pc_point` | `sensor_msgs/PointCloud2` | XYZRGB, frame_id=`camera_link` |

`ROS2SignalPublisher` 는 0.2초 타임아웃 confirmation manager + 빈도 기반 투표를 거쳐 stable signal 만 퍼블리시.

---

## 10. 테스트

### 실행

```bash
# 전체 (48개)
pytest src/nimg_v3/tests/ -v

# 커버리지
pytest src/nimg_v3/tests/ -v --cov=nimg_v3

# 단일 파일
pytest src/nimg_v3/tests/test_pose_converter.py -v

# 단일 테스트
pytest src/nimg_v3/tests/test_pose_converter.py::TestPoseConverter::test_euler_to_rotation_matrix -v
```

### 테스트 파일

| 파일 | 테스트 항목 |
|------|------------|
| `test_pose_converter.py` | EulerAngles 기본/정규화, Quaternion 단위/켤레/곱, PoseConverter 왕복 변환, AngleChange, 6D representation, GimbalLock 감지 |
| `test_pose_kalman_filter.py` | Euler 모드 / Quaternion 모드 / 적응형 노이즈 / 연속 프레임 / 필터 리셋 |
| `test_reference_image_loader.py` | 14방향 뷰 폴더 탐색, RGB/Depth 스택, 각도 파싱, 제외 패턴 |

**현재 상태**: 48 / 48 PASSED

---

## 11. 디자인 패턴

| 패턴 | 적용 위치 | 목적 |
|------|----------|------|
| **Singleton** | `ROS2PointCloudPublisher` | 단일 ROS2 노드 보장 |
| **Factory** | `build_fp_estimator()`, `build_segmenter()`, `build_encoder()`, `build_tracker_2d()`, `IntegratedMeasurementSystem.from_config()` | 백엔드 자동 선택 |
| **Adapter** | `InputAdapter` (PairedVideoSource / RealSenseSource) | 동일 `Frame` 인터페이스 |
| **State Machine** | `TrackingState` (INITIALIZING → TRACKING → LOST → 복구) | 자동 재초기화 |
| **Strategy** | `RecognitionConfig.backend` ("noctis"/"cnos"/"yolo"/"depth_bbox") | 인식 전략 교체 |
| **Graceful Degradation** | SAM3.1 → SAM2.1 → YOLO-seg → DepthBox / DINOv3 → DINOv2 → HSV-hist / Real FP → Fallback FP / CuPy GPU → NumPy CPU | 환경에 따라 동작 보장 |
| **Lazy Initialization** | `IntegratedMeasurementSystem._init_detector/pose_estimator` | 무거운 모델 지연 로딩 |

---

## 12. v1→v5 핵심 교훈

[test_result/260420/REPORT.md](../../test_result/260420/REPORT.md) 에 전체 diff + diagnostics 보존.

### 버전별 변화

| ver | 초점 | 결과 |
|-----|------|------|
| **v1** | 초기 모듈 + 비디오 벤치마크 | 엔드투엔드 파이프라인 기동, Layer-C 2-5ms (PCA), viz.mp4 생성 실패 |
| **v2** | 매 프레임 YOLO-seg refresh + IoU 재연결 + viz | mask 가시화 + imageio-ffmpeg 로 viz.mp4 교체 |
| **v3** | depth threshold 0.15→0.01, score 공식, `ts=idx/fps`, LostDetector grace | uid 134→3, velocity 현실적 수렴 |
| **v4** | per-instance FP++ (멀티트래킹), BaselineLoader, PCA 부호 모호성 2단 해결, CW=+ | 평균 tracks/frame 1.5-2.3, yaw 180° 플립 제거 |
| **v5** | fp env 구축 (torch 2.7+cu128 + env-local CUDA 12.8 + pytorch3d/nvdiffrast 소스 빌드) → **실 FoundationPose GPU 추론**, mesh flat-gray 패딩, singular-matrix 예외 | 6 비디오 전부 `fp_estimator: foundationpose`, Layer-C 29-32ms mean / 31-51ms p95 |

### 재발 방지 체크리스트

| 문제 | 원인 | 해결 |
|------|------|------|
| 트래킹 끊김 (uid 134개/400 프레임) | `FallbackFPEstimator` depth `>0.15m` 가 MP4 인코딩 0~0.28m depth 거의 필터링 → score=0.1 → reinit 루프 | threshold `>0.01`, score 공식을 mask 픽셀 기반으로 재작성 |
| 속도/각도 0 으로 고정 | `timestamp=time.time()` (wall-clock) + 잦은 reinit 으로 KF reset | `timestamp = frame_idx / fps` (비디오 시간) + LostDetector grace period 5 프레임 |
| 세그멘테이션 미표시 | viz 에 mask overlay 없음 | `draw_mask_overlay` (alpha=0.42 + contour) + `draw_bbox_label` + `draw_signal_badge` |
| 멀티트래킹 불가 | class_id 당 shared FP++, init loop `break` | per-instance FP++ 팩토리 + break 제거 + 중복 bbox IoU>0.4 체크 |
| housing_M baseline 아님 | runtime 첫 프레임의 T 를 baseline 사용 | `BaselineLoader` 로 `reference_config.yaml` / `0deg*.png` 에서 T_ref 사전 계산 |
| yaw ±180° 플립 | PCA 고유벡터 ±부호 모호 | (1) principal[0]≥0 정규화, (2) 이전 프레임 내적 양수 유지, (3) normal 동일 처리, (4) wrap (-180,180] |
| `nvcc fatal: compute_120 unsupported` | 호스트 CUDA 12.1 이 Blackwell sm_120 미지원 | fp env 내부에 `conda install -c nvidia/label/cuda-12.8.0` 로 env-local CUDA 12.8 |
| `pytorch3d` pip wheel 없음 | torch 2.7+cu128 용 prebuilt 없음 | env-local nvcc + `TORCH_CUDA_ARCH_LIST="...12.0+PTX"` 로 소스 빌드 |
| FP `AttributeError: 'NoneType' has no 'convert'` | 메쉬에 texture 이미지 없음 | `_load_textured_mesh()` 가 flat-gray 32×32 + XY-planar UV 패딩 |
| FP `linalg.inv: singular matrix` | bbox edge case | `fp_plus_plus.track()` try/except → prev_T 유지 + score×0.8 |

---

## 부록 A: 하드웨어 사양

| 항목 | 사양 |
|------|------|
| **카메라** | Intel RealSense D455 (640×480, 30 fps) |
| **기본 K** | fx=383.883, fy=383.883, cx=320.499, cy=237.913 |
| **보정 파일** | `nimg/Calibration_result_d455.yaml` (1280×720용) |
| **타겟 배포** | NVIDIA Jetson Orin Nano Super |
| **현재 GPU** | NVIDIA GeForce RTX 5080 (Blackwell, sm_120, 15 GB) |

## 부록 B: YOLO 데이터셋

Roboflow 프로젝트 `sl-cjsxb/project-wdpim`. 단일 클래스 세그멘테이션.

| 버전 | 경로 | 클래스명 | 비고 |
|------|------|---------|------|
| v1~v5 | `src/project.v{1,3,5}i.yolov11/` | `objects` | 초기 |
| v6~v10 | `src/project.v{6..10}i.yolov11/` | `tray_top` | 클래스명 변경 |
| combined | `combined_dataset/` | `tray_top` | v7+v8+v9 병합 |

## 부록 C: 리서치 문서

[src/research/](../research/) — 날짜별 리서치 결과:

| 디렉토리 | 주제 |
|---------|------|
| `251008/` | Depth 기반 속도/각도 측정 원리 |
| `251204/` | D455 + Jetson Orin Nano 하드웨어 분석, AI 모델 선정 |
| `251205/` | 구현 설계, ORB 부적합 결론 |
| `251218/` | RAFT, DenseFusion, KalmanNet, Depth-Anything-v3 |
| `251224/` | FoundationPose 논문 한국어 번역 |
| `251229/` | YOLO 도메인 시프트 분석 |
| `251231/` | Euler vs Quaternion, FoundationPose 설계, YOLO Roboflow |
| `260106/` | FoundationPose Model-Free 마스크 요구사항 |
| `260212/` | PointCloud 인터페이스 설계 |
| `260327/` | 3D 스캔 메쉬 + SAM + FP++ 실시간 추적 |
| `260420/` | **v5 아키텍처 통합 설계** (SAM3.1 + DINOv3 + NOCTIS + FP++ + DAM4SAM) |

---

## 부록 D: 모델 파일

### YOLO

| 파일 | 크기 | 용도 |
|------|------|------|
| `models/yolo/class187_image85286_v12x_250epochs.pt` | 114 MB | 187 클래스 범용 (YOLOv12x) |
| `models/yolo/Yolo_v11_seg_m_top.pt` | - | tray_top 세그멘테이션 (프로덕션 스트리밍) |
| `models/yolo/yolo26_2class_seg_best_260324.pt` | - | NOCTIS 의 Tier-1 YOLO fallback |

### FoundationPose

| 파일 | 일자 | 용도 |
|------|------|------|
| `models/foundationpose/scorer/` | 2024-01-11 | Pose scoring network |
| `models/foundationpose/refiner/` | 2023-10-28 | Pose refinement network |

### Template DB

| 파일 | 사양 |
|------|------|
| `models/neural_fields/template_db.npz` | N=84 (42뷰 × 2클래스), D=1024 (DINOv2-L) |

---

**문서 끝.** v5 통합 파이프라인 (2026-04-20) 기준. 다음 변경 시 [test_result/260420/REPORT.md](../../test_result/260420/REPORT.md) 와 [CLAUDE.md](../../CLAUDE.md) 도 함께 업데이트.
