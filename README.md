# 듀얼 라이다 기반 로봇 도장 경로 생성 시스템 - 코드 분석 문서

## 📋 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [전체 아키텍처](#2-전체-아키텍처)
3. [실행 파이프라인](#3-실행-파이프라인)
4. [파일별 상세 분석](#4-파일별-상세-분석)
   - [4.1 자동화 실행기 (auto.py / auto_1.py)](#41-자동화-실행기-autopy--auto_1py)
   - [4.2 라이다 캘리브레이션 (cali.py)](#42-라이다-캘리브레이션-calipy)
   - [4.3 듀얼 라이다 정합 (view.py / view_1.py)](#43-듀얼-라이다-정합-viewpy--view_1py)
   - [4.4 다운샘플링 (downsample.py / downsample_1.py)](#44-다운샘플링-downsamplepy--downsample_1py)
   - [4.5 ICP 등록 파이프라인 (final.py)](#45-icp-등록-파이프라인-finalpy)
   - [4.6 노이즈 제거 (ran.py)](#46-노이즈-제거-ranpy)
   - [4.7 경로 생성 (path1.py / path2.py)](#47-경로-생성-path1py--path2py)
   - [4.8 경로 변환/시각화 (paview.py 시리즈)](#48-경로-변환시각화-paviewpy-시리즈)
   - [4.9 사용자 클릭 기반 경로 (first.py)](#49-사용자-클릭-기반-경로-firstpy)
   - [4.10 모서리 점 추출 (path_edge_display.py)](#410-모서리-점-추출-path_edge_displaypy)
   - [4.11 라이다 노드 (lidar_node.cpp)](#411-라이다-노드-lidar_nodecpp)
   - [4.12 GUI 메인 (mainwindow.py)](#412-gui-메인-mainwindowpy)
5. [데이터 파일 구조](#5-데이터-파일-구조)
6. [의존 라이브러리](#6-의존-라이브러리)
7. [실행 방법](#7-실행-방법)

---

## 1. 프로젝트 개요

### 🎯 목적
이 프로젝트는 **듀얼 라이다(Dual LiDAR)** 센서를 활용해 3D 객체(예: 선체 블록)를 스캔한 뒤, 그 결과를 바탕으로 **로봇 도장(painting) 경로**를 자동 생성하는 시스템입니다.

### 🏭 적용 분야
- **마젠타로보틱스(Magenta Robotics)** 의 산업용 로봇 도장 자동화
- 선박/조선 산업의 블록 표면 도장
- 대형 곡면 객체의 로봇 자동 가공

### ⚙️ 핵심 기술
| 기술 | 사용 라이브러리 | 용도 |
|------|----------------|------|
| **포인트클라우드 처리** | Open3D, PCL | 라이다 데이터 입출력, 변환 |
| **메시 처리** | PyVista | STL 모델 시각화/레이캐스팅 |
| **ICP 정합** | CloudComPy + C++ ICP | 스캔 데이터를 모델에 정렬 |
| **수치 계산** | NumPy, SciPy | 행렬 연산, 보간, 필터링 |
| **로봇 통신** | ROS2 (rclcpp) | 라이다 데이터 수집 |
| **GUI** | PyQt5 | 사용자 인터페이스 |

---

## 2. 전체 아키텍처

```
┌──────────────────────────────────────────────────────────────┐
│                     하드웨어 계층 (Hardware)                   │
│  ┌─────────────┐  ┌─────────────┐  ┌──────────────────────┐  │
│  │ Livox MID360│  │ Livox MID360│  │  로봇 (UR/산업용)      │  │
│  │  Lidar 1    │  │  Lidar 2    │  │                      │  │
│  └──────┬──────┘  └──────┬──────┘  └──────────┬───────────┘  │
└─────────┼────────────────┼────────────────────┼──────────────┘
          │ ROS2 토픽       │ ROS2 토픽          │ TCP/IP
          ▼                ▼                    ▼
┌──────────────────────────────────────────────────────────────┐
│                  데이터 수집 계층 (Collection)                  │
│   ┌──────────────────────┐    ┌──────────────────────────┐   │
│   │ lidar_node.cpp       │    │ lidar_collector          │   │
│   │ (단일 라이다 누적/저장)│    │ (듀얼 라이다 동시 수집)   │   │
│   └──────────────────────┘    └──────────────────────────┘   │
└──────────────────────────┬───────────────────────────────────┘
                           ▼ .pcd 파일 (raw)
┌──────────────────────────────────────────────────────────────┐
│                  전처리 계층 (Preprocessing)                    │
│  cali.py → 4점 SVD 캘리브레이션 → calib_dual.txt              │
│  view.py → 두 라이다 병합 + 크롭 → cropped_world.pcd          │
│  downsample.py → 복셀 다운샘플링 → cropped_world_downsampled  │
│  ran.py → 노이즈 제거 (SOR/ROR) + 평면 추출                   │
└──────────────────────────┬───────────────────────────────────┘
                           ▼
┌──────────────────────────────────────────────────────────────┐
│              ICP 정합 계층 (Registration - final.py)            │
│  [1단계] C++ ICP (Initial alignment)                          │
│  [2단계] CloudComPy ICP (Refined alignment)                   │
│  → T_ms_overall.txt (4×4 변환 행렬)                           │
└──────────────────────────┬───────────────────────────────────┘
                           ▼
┌──────────────────────────────────────────────────────────────┐
│              경로 생성 계층 (Path Generation)                   │
│  path1.py: 반구 레이캐스팅 + 다중 라인 도장 경로                │
│  path2.py: PCA 기반 정규화 + 곡면 추적 경로                    │
│  first.py: 사용자 클릭 기반 시작점                              │
└──────────────────────────┬───────────────────────────────────┘
                           ▼ robot_path.json (월드 좌표 m)
┌──────────────────────────────────────────────────────────────┐
│            경로 변환 계층 (Path Transformation)                 │
│  paview.py / paview1_headless.py:                             │
│  - T_ms_overall.txt 적용                                      │
│  - m → mm 단위 변환                                           │
│  → iktarget/robot_path_mm_*.json                              │
└──────────────────────────┬───────────────────────────────────┘
                           ▼
                    [로봇 IK / 컨트롤러]
```

---

## 3. 실행 파이프라인

### 🚀 자동 파이프라인 (auto.py)
```
view.py → downsample.py → final.py → paview.py
```

| 단계 | 스크립트 | 입력 | 출력 | 타임아웃 |
|------|---------|------|------|----------|
| 1 | `view.py` | 라이다 1, 2 PCD + calib_dual.txt | merged_world.pcd, cropped_world.pcd | 무제한 |
| 2 | `downsample.py` | cropped_world.pcd | cropped_world_downsampled.pcd | 5분 |
| 3 | `final.py` | downsampled PCD + STL | T_ms_overall.txt | 1시간 |
| 4 | `paview.py` | robot_path.json + T_ms_overall.txt | iktarget/*.json (mm 단위) | 무제한 |

---

## 4. 파일별 상세 분석

### 4.1 자동화 실행기 (auto.py / auto_1.py)

#### 📁 파일 정보
- **경로**: [auto.py](auto.py), [auto_1.py](auto_1.py)
- **크기**: 약 6.9KB
- **차이점**: `auto_1.py`는 SCRIPT_DIR 기본값이 `/root/sl`로, `auto.py`는 `/home/magenta/sl`로 다름

#### 🎯 역할
듀얼 라이다 처리 파이프라인의 **전체 실행을 자동화**하는 마스터 스크립트

#### 🔑 핵심 데이터 클래스
```python
@dataclass
class StepResult:
    script: str                         # 실행한 스크립트명
    success: bool                       # 성공 여부
    returncode: int                     # 종료 코드
    elapsed: float                      # 실행 시간(초)
    error_msg: Optional[str] = None     # 오류 메시지
```

#### ⚙️ 주요 함수

| 함수명 | 설명 |
|--------|------|
| `get_python_executable()` | 사용할 Python 인터프리터 결정 (`PYTHON_EXECUTABLE` 또는 `sys.executable`) |
| `get_script_path(script_name)` | `SCRIPT_DIR` 기준으로 스크립트 절대경로 생성 |
| `run_script(script_name, timeout)` | `subprocess.run`으로 단일 스크립트 실행, 타임아웃·예외 처리 |
| `print_summary(results)` | 실행 결과(시간, 성공/실패) 요약 출력 |
| `run_pipeline()` | 전체 4단계 파이프라인 순차 실행 (실패 시 STOP_ON_FAILURE 적용) |

#### 💡 특징
- 각 스크립트마다 **개별 타임아웃** 적용 가능 (TIMEOUTS 딕셔너리)
- 실패 시 `STOP_ON_FAILURE=True` 로 즉시 중단
- KeyboardInterrupt(Ctrl+C) 처리: 종료코드 130으로 종료

---

### 4.2 라이다 캘리브레이션 (cali.py)

#### 📁 파일 정보
- **경로**: [cali.py](cali.py)
- **크기**: 약 10.6KB

#### 🎯 역할
듀얼 라이다의 **각각의 좌표계를 로봇 월드 좌표계로 정합**하는 4점 캘리브레이션 도구

#### 🧮 알고리즘
**SVD 기반 Rigid Transform**:
1. 사용자가 **GUI에서 4개 대응점을 클릭** (Shift+좌클릭)
2. SVD로 회전 R, 평행이동 t 계산
3. Reflection 방지 (det(R) < 0 일 때 마지막 행 부호 반전)
4. RMS 오차 계산

#### 🔑 핵심 설정값
```python
TARGET_POINTS = np.array([
    [0.578, -0.100, 0.284],  # P1 (월드 좌표, m)
    [0.843, -0.087, 0.284],  # P2
    [0.856, -0.581, 0.284],  # P3
    [0.590, -0.581, 0.284],  # P4
], dtype=np.float64)

LIDAR_INPUT_UNIT_SCALE = 0.001  # mm → m 변환
```

#### ⚙️ 주요 함수

| 함수명 | 설명 |
|--------|------|
| `load_pointcloud(path, unit_scale)` | PLY 로드 + 단위 변환 (mm → m) |
| `pick_4_points(pcd, point_size, lidar_name)` | Open3D `VisualizerWithEditing`으로 4점 선택 |
| `compute_T_svd(source_pts, target_pts)` | SVD로 4×4 변환 행렬 계산 |
| `calc_errors(source_pts, target_pts, T)` | 변환 후 각 점의 잔차 + RMS |
| `save_dual_txt(...)` | Lidar1/Lidar2 변환 행렬을 `calib_dual.txt`에 저장 |
| `calibrate_single_lidar(...)` | 단일 라이다 전체 흐름 (로드→선택→SVD→오차) |
| `visualize_dual_result(result1, result2)` | 두 라이다 동시 시각화 (빨강/파랑 = 원본, 초록/시안 = 정합 후) |

#### 📤 출력
[`calib_dual.txt`](calib_dual.txt) - 두 라이다의 4×4 변환 행렬 + 메타 정보
```python
transformation_lidar1 = np.array([
    [-0.506652, -0.769103, -0.389595, 1.562448],
    [ 0.658939, -0.636846,  0.400282, -1.593019],
    [-0.555971, -0.053915,  0.829452,  1.112797],
    [ 0.000000,  0.000000,  0.000000,  1.000000],
])
# Lidar1 RMS Error: 0.005612 m
```

---

### 4.3 듀얼 라이다 정합 (view.py / view_1.py)

#### 📁 파일 정보
- **경로**: [view.py](view.py)
- **크기**: 약 19.5KB

#### 🎯 역할
1. `calib_dual.txt`의 변환 행렬로 두 라이다를 **로봇 월드 좌표계**로 변환
2. 두 포인트클라우드 **병합 + AxisAlignedBoundingBox로 크롭**
3. 결과 저장 (`merged_world.pcd`, `cropped_world.pcd`)

#### 🔑 핵심 설정
```python
LIDAR1_FILE = "~/sl_per_ws/pcd_file/left"   # 디렉토리도 가능 → 최신 .pcd 자동 선택
LIDAR2_FILE = "~/sl_per_ws/pcd_file/right"
CALIB_FILE  = "calib_dual.txt"
LIDAR_INPUT_UNIT_SCALE = 0.001              # mm → m 변환
CROP_BOUNDS = (0.3, 1.0, -1.15, 0.45, 0.35, 1)  # X/Y/Z min·max (m)
ENABLE_VISUALIZATION = False                # 서버 환경
```

#### 🏛️ 주요 클래스: `DualLidarMerge`

| 메서드 | 설명 |
|--------|------|
| `__init__(lidar1, lidar2, calib, point_size, unit_scale)` | 경로/스케일 초기화 |
| `load_calib_file()` | 정규식으로 `calib_dual.txt` 파싱 → `transformation_lidar1/2` |
| `load_pointclouds()` | `time.sleep(3)`(파일 저장 대기) 후 PCD 로드 + mm→m 변환 |
| `transform_to_world()` | T 적용 → 색상 부여 (빨강/파랑) → 병합 |
| `set_crop_bounds(...)` / `crop_pointcloud()` | AABB 크롭 |
| `visualize_world(pcd, title)` | 좌표축 + 그리드 + 포인트클라우드 시각화 |
| `save_results(merged, cropped, config)` | 결과 PCD/`crop_config.txt` 저장 |
| `run(crop_bounds, ...)` | 전체 파이프라인 (1.캘리브 로드 → 2.PCD 로드 → 3.월드 변환 → 4.시각화 → 5.크롭 → 6.저장) |

#### 🔍 보조 함수

| 함수 | 용도 |
|------|------|
| `get_latest_pcd_file(path)` | 디렉토리면 가장 최근 mtime의 `.pcd` 자동 선택 |
| `create_axes_lines(size, origin)` | RGB 좌표축 LineSet 생성 |
| `create_axes_with_labels(size, origin)` | 좌표축 + 끝점 색상 구체 |
| `create_grid(size, divisions, plane, height)` | XY 평면 그리드 |

---

### 4.4 다운샘플링 (downsample.py / downsample_1.py)

#### 📁 파일 정보
- **경로**: [downsample.py](downsample.py), [downsample_1.py](downsample_1.py)
- **크기**: 약 0.8KB / 1.0KB

#### 🎯 역할
크롭된 포인트클라우드를 **복셀 다운샘플링** (메모리/계산량 감소)

#### 🔑 설정
```python
INPUT_PATH = "cropped_world.pcd"
VOXEL_SIZE = 0.001  # 1mm 복셀
```

#### 차이점
- `downsample.py`: PCD만 저장
- `downsample_1.py`: PCD **+ PLY** 둘 다 저장 (CloudComPy 호환용)

```python
# downsample_1.py 추가 부분
PLY_PATH = str(Path(OUTPUT_PATH).with_suffix('.ply'))
o3d.io.write_point_cloud(PLY_PATH, pcd)
print(f"[저장] {PLY_PATH} (CloudComPy용)")
```

---

### 4.5 ICP 등록 파이프라인 (final.py)

#### 📁 파일 정보
- **경로**: [final.py](final.py)
- **크기**: 약 12.1KB

#### 🎯 역할
**2단계 ICP 정합**으로 스캔 데이터(World)를 모델 STL에 정렬:
1. **C++ ICP** (빠른 초기 정합) → `transformation_matrix_inverse.txt`
2. **CloudComPy ICP** (정밀 정합) → `T_ms_overall.txt`

#### 🔑 고정 경로 설정
```python
CPP_INPUT_PCD       = ~/sl_per_ws/gene1.pcd                          # 모델
CPP_GLOBAL_PCD      = ~/sl_per_ws/cropped_world_downsampled.pcd      # 스캔
CLOUDCOMPY_WORKDIR  = ~/sl_per_ws/cloudcompy
CLOUDCOMPY_SCRIPT   = ~/sl_per_ws/scripts/test1.py
CLOUDCOMPY_MODEL_STL = ~/sl_per_ws/gene1.stl
PRE_XFORM_PATH      = .../transformation_matrix_inverse.txt          # 1단계 결과
OUTPUT_DIR          = ~/sl_per_ws/global/out_icp_only
```

#### 🔑 주요 데이터 클래스
```python
@dataclass
class ProcResult:
    returncode: int
    elapsed: float
    log_path: Optional[str] = None
    killed_by_timeout: bool = False
```

#### ⚙️ 주요 함수

| 함수 | 역할 |
|------|------|
| `convert_pcd_to_ply(pcd_path)` | PCD→PLY 변환 (CloudComPy는 PCD 로드 실패) |
| `is_display_error(log_path)` | 로그에서 X/Qt/GLX 오류 감지 (DISPLAY_ERR_PATTERNS 사용) |
| `_run_stream(cmd, cwd, timeout, log_path)` | subprocess `Popen` + 스레드로 stdout 펌프 + 타임아웃 시 SIGTERM/SIGKILL |
| `run_cpp_icp(timeout_sec)` | `bash run_cpp_icp.sh INPUT GLOBAL` 실행 |
| `run_cloudcompy_icp(timeout_sec)` | `bash run_cloudcompy_icp.sh --workdir ... --python test1.py -- --model-stl ...` |
| `run_full_registration()` | C++ → CloudComPy 순서, 디스플레이 오류는 무시 |
| `print_configuration()` | 현재 고정 경로 설정 출력 |

#### 🚨 디스플레이 오류 패턴 (서버 환경 대응)
```python
DISPLAY_ERR_PATTERNS = [
    r"vtkXOpenGLRenderWindow.*bad X server connection",
    r"QXcbConnection:\s*could not connect to display",
    r"DISPLAY\s*=\s*\.?",
    r"GLX.*failed",
    r"Xlib:.*extension.*missing",
    r"xcb:.*ERROR",
    r"Could not initialize offscreen",
    r"qt\.qpa\.xcb.*could not connect to display",
    r"QSocketNotifier.*can only be used with threads",
    r"Could not load the Qt platform plugin",
    r"no Qt platform plugin could be initialized",
]
```

#### 💡 명령행 옵션
```bash
python final.py --help     # 도움말
python final.py --config   # 현재 경로 설정 출력
python final.py            # 전체 파이프라인 실행
```

---

### 4.6 노이즈 제거 (ran.py)

#### 📁 파일 정보
- **경로**: [ran.py](ran.py)
- **크기**: 약 4.7KB

#### 🎯 역할
**원시 PLY 파일의 노이즈/이상치 제거 + 평면 추출**

#### 🔑 설정 (mm 단위)
```python
CONFIG = {
    "input_path": "2.ply",
    "voxel_size": 10,                   # 10mm 다운샘플
    "method": "sor",                    # sor / ror / both / None
    "sor_nb_neighbors": 20,
    "sor_std_ratio": 2.0,
    "ror_nb_points": 16,
    "ror_radius": 50,                   # 50mm
    "plane_distance_threshold": 10,     # 10mm
    "plane_ransac_n": 3,
    "plane_num_iterations": 1000,
    "visualize": True,
}
```

#### ⚙️ 주요 함수

| 함수 | Open3D API | 설명 |
|------|-----------|------|
| `load_ply(filepath)` | `read_point_cloud` | PLY 로드 |
| `downsample(pcd, voxel_size)` | `voxel_down_sample` | 복셀 다운샘플링 |
| `remove_statistical_outliers(pcd, nb_neighbors, std_ratio)` | `remove_statistical_outlier` | SOR (이웃 수 + 표준편차 기반) |
| `remove_radius_outliers(pcd, nb_points, radius)` | `remove_radius_outlier` | ROR (반경 내 점 수 기반) |
| `extract_plane(pcd, distance_threshold, ransac_n, num_iterations)` | `segment_plane` | RANSAC으로 평면 모델 추출 |

#### 📤 출력
- `{input_stem}_denoised.ply` (예: `2_denoised.ply`)

---

### 4.7 경로 생성 (path1.py / path2.py)

#### 📁 파일 정보
- **path1.py**: 약 21.9KB - 반구 레이캐스팅 + 다중 라인 도장 경로
- **path2.py**: 약 30.6KB - **개선판** (PCA 정규화 + 곡면 추적 + 자동 시작점)

#### 🎯 path1.py: `PaintPathGenerator` 클래스

##### 🏛️ 주요 메서드

| 메서드 | 설명 |
|--------|------|
| `__init__(stl_path)` | STL 로드 + 바운딩박스 |
| `generate_multi_direction_path(...)` | **반구 레이캐스팅** - X 방향 라인×반원 angle (theta = 0~π) |
| `find_sharp_turns(angle_threshold)` | 같은 라인에서 갑작스럽게 꺾이는 점 검출 |
| `find_sharp_turns_with_chain(...)` | 연쇄 이상점까지 검출 (편차 기반) |
| `correct_outliers_by_column(...)` | 같은 열의 가까운 정상 점으로 보정 |
| `smooth_x_rotation_outliers(...)` | X축 회전 이상치 윈도우 평균으로 스무딩 |
| `get_6dof_path(invert_normal)` | 6-DOF (3 위치 + 3 법선) 경로 |
| `get_path_by_lines(invert_normal)` | 라인별로 분리 |
| `save_path_by_lines(output_path, format)` | JSON/CSV 저장 |
| `visualize(...)` | PyVista 다중 요소 시각화 |

##### 🔑 알고리즘 핵심
1. STL 바운딩박스에서 반지름 = `max(size_y, size_z) / 2 * RADIUS_SCALE`
2. X축으로 일정 간격(`LINE_SPACING`) 라인 생성
3. 각 X에서 theta(0~π) 각도로 **반구 표면** 위 점 → 메시 중심으로 레이 발사
4. `mesh.ray_trace()`로 히트 → 면 법선 추출 + Y rotation limit 클리핑
5. 미스된 점은 인접 라인에서 복사 (X만 현재값으로 변경)
6. 이상점 검출/보정 + X축 회전 스무딩

#### 🎯 path2.py: 개선판 v4.1

##### 추가된 기능
1. **자동 시작점 추출** (`extract_edge_points_with_normals`)
   - X 최소 모서리 점 자동 검출 (반구 360° 레이캐스팅)
   - 시작점 오프셋 (lift + X+ 이동)
2. **PCA 기반 로봇 포지션 정규화** (`normalize_robot_positions_to_line`)
   - 로봇 타겟 포지션을 PCA 주성분 라인에 투영
   - 표면 접촉점은 그대로 (이미 일직선)
3. **곡면 따라가기 경로 생성** (`generate_path_along_surface`)
   - 법선 ⊥ + X+ 방향으로 이동
   - 히트 실패 시 방향 이동 + 이전 법선 복사
   - 경로 각도 급변 시 이전 방향으로 예측 이동
4. **JSON 출력 형식 변경**
   - `[robot_x, robot_y, robot_z, surface_x, surface_y, surface_z]` (6차원)
   - 로봇 위치 ↔ 표면 접촉점 연결선 시각화

##### ⚙️ 주요 함수
| 함수 | 역할 |
|------|------|
| `get_perpendicular_xplus_direction(normal)` | 법선 ⊥ + X+ 방향 단위 벡터 |
| `check_normal_angle_change(new_n, prev_n, threshold)` | 법선 각도 변화 검출 |
| `check_path_direction_change(prev_pt, curr_pt, new_pt, threshold)` | 경로 방향 변화 검출 |
| `normalize_robot_positions_to_line(robot_pos, surface_pts)` | PCA 정규화 |
| `extract_edge_points_with_normals(stl_path, ...)` | X 최소 모서리 점 자동 추출 |
| `generate_path_along_surface(mesh, start_pt, start_n, ...)` | 곡면 따라 6-DOF 경로 |
| `convert_to_robot_pose(points, normals, offset)` | 표면점 → 로봇 위치 변환 |
| `transpose_and_save_json(robot_pos, surf_pts, points, output)` | 라인을 열로 전치하여 JSON 저장 |
| `visualize_result(...)` | 정규화 전/후 비교 시각화 |

#### 📤 출력 (path2.py)
[`robot_path.json`](robot_path.json):
```json
{
  "description": "각 점: [robot_x, robot_y, robot_z, surface_x, surface_y, surface_z]",
  "num_lines": <int>,
  "total_points": <int>,
  "lines": [[[robot_xyz, surface_xyz], ...], ...]
}
```

---

### 4.8 경로 변환/시각화 (paview.py 시리즈)

| 파일 | 역할 |
|------|------|
| [paview.py](paview.py) | **표준 버전** - PyVista 시각화 + JSON 변환 (path2의 6-DOF 형식) |
| [paview1.py](paview1.py) | **구 버전** - path1의 6-DOF 형식 (point + direction) |
| [paview1_headless.py](paview1_headless.py) | **헤드리스 버전** - 시각화 없이 JSON 변환만 |

#### 🎯 공통 역할
1. `robot_path.json` (m 단위) 로드
2. `T_ms_overall.txt` (4×4) 로드
3. **변환 적용**:
   - 위치: `p' = R @ p + t` → `output_scale` 곱
   - 법선/방향: `n' = R @ n` (회전만)
4. **mm 단위로 저장** (output_scale = 1000.0)
5. 타임스탬프 파일명: `robot_path_mm_YYMMDD_HHMMSS_ffffff.json` (마이크로초 포함)
6. 출력 폴더: `./iktarget/`

#### 🔑 paview.py vs paview1.py 차이
- **paview.py** (path2.py 결과 호환):
  - 입력 형식: `[robot_x, robot_y, robot_z, surface_x, surface_y, surface_z]`
  - 두 위치 벡터 모두 변환 (`R @ robot_pos + t`, `R @ surface_pt + t`)
- **paview1.py** (path1.py 결과 호환):
  - 입력 형식: `[x, y, z, nx, ny, nz]`
  - 위치는 변환 + scale, 방향은 회전만

#### 💡 명령행 옵션 (paview.py)
```bash
python paview.py \
  --json robot_path.json \
  --transform ./global/out_icp_only/T_ms_overall.txt \
  --output ./iktarget/output.json \
  --scale 1000.0
```

---

### 4.9 사용자 클릭 기반 경로 (first.py)

#### 📁 파일 정보
- **경로**: [first.py](first.py)
- **크기**: 약 23.4KB

#### 🎯 역할
**path2.py의 이전 버전** - 자동 시작점 대신 **사용자가 Open3D 창에서 점을 클릭**하여 시작점 지정

#### 🔑 핵심 알고리즘
1. STL을 포인트클라우드로 샘플링 (`sample_points_uniformly`)
2. KDTree로 법선 추정
3. 사용자가 Shift+클릭으로 점 선택 (`VisualizerWithEditing`)
4. 클릭점을 추정 법선 방향으로 띄움 → 역방향 레이캐스팅 → 정확한 메시 삼각형 법선 획득
5. 곡면 따라 경로 생성 (path2와 동일한 `generate_path_along_surface`)

#### ⚙️ 핵심 함수
- `pick_points_open3d(stl_path, ...)` - 점 클릭 + 메시 법선 정확화
- `generate_path_along_surface(mesh, start_pt, start_n, ...)` - 곡면 따라가기
- `check_normal_angle_change()`, `check_path_direction_change()` - 이상치 검출

---

### 4.10 모서리 점 추출 (path_edge_display.py)

#### 📁 파일 정보
- **경로**: [path_edge_display.py](path_edge_display.py)
- **크기**: 약 5.2KB

#### 🎯 역할
**디버그/시각화 전용** - STL 메시에서 X 최소 모서리 점 추출 후 표시
- Open3D 레이캐스팅 (빠름) + PyVista 시각화

#### 🔑 알고리즘
1. STL 로드 → 바운딩박스
2. X 방향 + 반구 360° 레이캐스팅 (theta = 0~2π)
3. 각 theta별로 X 최소값을 가진 히트점 선택 → 메시의 좌측 가장자리 추출
4. PyVista로 메시(투명) + 레이 시작점(회색) + X 최소점(빨강) 시각화

#### 🔑 파라미터
```python
STL_FILE = "gene1.stl"
X_EXTEND = 0.2
LINE_SPACING = 0.0001       # 0.1mm 매우 촘촘
POINTS_PER_LINE = 180       # 360° / 2°
RADIUS_SCALE = 1.2
```

---

### 4.11 라이다 노드 (lidar_node.cpp)

#### 📁 파일 정보
- **경로**: [lidar_node.cpp](lidar_node.cpp)
- **크기**: 약 7.4KB
- **언어**: C++ (ROS2)

#### 🎯 역할
ROS2 노드로 **Livox 라이다 데이터를 누적·저장하고 LaserScan으로 변환** 발행

#### 🔑 커스텀 PointType
```cpp
struct PointXYZRTL {
    PCL_ADD_POINT4D;            // x, y, z, padding
    float reflectivity;
    std::uint8_t tag;
    std::uint8_t line;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
```
PCL에 등록: `POINT_CLOUD_REGISTER_POINT_STRUCT`

#### 🏛️ 클래스: `LidarNode` (rclcpp::Node 상속)
| 멤버 | 설명 |
|------|------|
| `sub_` | `/livox/lidar` 토픽 구독 |
| `scan_pub_` | `perc/pcl` 토픽으로 LaserScan 발행 |
| `accumulated_cloud_` | 5초간 누적 포인트클라우드 |
| `accumulate_duration_=5` | 누적 주기 (초) |
| `delete_threshold_=20` | 오래된 PCD 파일 자동 삭제 (초) |

#### ⚙️ 처리 흐름
1. **콜백** (`lidarCallback`):
   - `pcl::fromROSMsg`로 PCD 변환
   - `accumulated_cloud_`에 추가
2. **5초 경과 시**:
   - 360° angle bin (1°)으로 LaserScan 생성 (`atan2(y, x)`, `sqrt(x²+y²)`)
   - `pcl::io::savePCDFileBinary`로 파일 저장 (`~/sam_ws/storage/lidar_img1/lidar_cloud_YYYYMMDD_HHMMSS_mmm.pcd`)
   - 20초 이상 된 PCD 파일 자동 삭제 (`removeOldPcdFiles`)
   - `scan_pub_->publish(scan_msg)`

---

### 4.12 GUI 메인 (mainwindow.py)

#### 📁 파일 정보
- **경로**: [mainwindow.py](mainwindow.py)
- **크기**: 약 64.9KB (대형)
- **추정**: PyQt5 기반 GUI 메인 윈도우 (전체 시스템 제어 인터페이스)

---

## 5. 데이터 파일 구조

### 📂 입력 데이터
| 파일 | 설명 |
|------|------|
| `1.ply`, `2.ply` | 라이다 1, 2의 원시 스캔 (mm 단위) |
| `1_denoised.ply`, `2_denoised.ply` | ran.py로 노이즈 제거된 PLY |
| `gene1.pcd` | 모델 포인트클라우드 (m 단위) |
| `gene1.stl` | 모델 STL 메시 |

### 📂 중간 산출물
| 파일 | 생성자 | 설명 |
|------|--------|------|
| `calib_dual.txt` | cali.py | 두 라이다의 4×4 변환 행렬 (m 단위) |
| `merged_world.pcd` | view.py | 두 라이다 병합 결과 (월드 좌표) |
| `cropped_world.pcd` | view.py | 크롭 결과 |
| `crop_config.txt` | view.py | 크롭 범위 (X/Y/Z min·max, m) |
| `cropped_world_downsampled.pcd` | downsample.py | 다운샘플 (1mm 복셀) |
| `cropped_world_downsampled.ply` | downsample_1.py | CloudComPy용 PLY |
| `transformation_matrix*.txt` | C++ ICP | 1단계 ICP 변환 |
| `T_ms_overall.txt` | CloudComPy ICP | 최종 누적 변환 |
| `robot_path.json` | path1/2.py | 도장 경로 (m 단위) |

### 📂 최종 출력
| 파일 | 생성자 | 설명 |
|------|--------|------|
| `iktarget/robot_path_mm_*.json` | paview*.py | 변환된 경로 (mm 단위, 로봇 IK 입력용) |

### 📂 데이터 형식 예시 (`robot_path.json`)
```json
{
  "description": "각 점: [robot_x, robot_y, robot_z, surface_x, surface_y, surface_z]",
  "num_lines": 4,
  "total_points": 68,
  "lines": [
    [
      [257.71, -355.26, 1008.12, 0.787, 0.023, -0.616],  // 점 1
      [252.48, -295.32, 1003.44, 0.806, 0.025, -0.592]    // 점 2
      // ...
    ]
    // ...
  ],
  "transform_applied": [[...], [...], [...], [...]],  // 4x4 행렬
  "output_unit_scale": 1000.0
}
```

---

## 6. 의존 라이브러리

### Python
```
open3d              # 포인트클라우드/메시 처리, 레이캐스팅
pyvista             # STL 시각화, 레이트레이싱
numpy               # 행렬/벡터 연산
scipy               # 보간(splprep), 필터(uniform_filter1d, gaussian_filter1d)
matplotlib          # (추정) 보조 시각화
PyQt5               # GUI (mainwindow.py)
dataclasses         # 표준 라이브러리
pathlib             # 표준 라이브러리
```

### C++ / ROS2
```
rclcpp                              # ROS2 C++ 클라이언트
sensor_msgs                          # PointCloud2, LaserScan
pcl_conversions, pcl                 # PCL ROS 변환
opencv2                              # OpenCV
livox_ros_driver2 / livox_ros2_driver # Livox SDK
```

### 외부 도구
```
CloudComPy311       # CloudCompare Python 바인딩 (cloudcompy/ 폴더)
ICP C++ 바이너리    # icp_registration_portable/bin/
```

---

## 7. 실행 방법

### 🎬 1. 캘리브레이션 (최초 1회)
```bash
# 두 라이다에서 각각 4점을 클릭 (Shift+좌클릭)
python cali.py
# → calib_dual.txt 생성
```

### 🎬 2. 자동 파이프라인 (반복 실행)
```bash
python auto.py
# 또는 단계별 수동 실행:
python view.py        # 듀얼 라이다 정합 + 크롭
python downsample.py  # 다운샘플링
python final.py       # ICP 정합 (C++ + CloudComPy)
python paview.py      # 경로 변환 (mm 단위 JSON 저장)
```

### 🎬 3. 경로 재생성만 (모델 변경 시)
```bash
python path2.py       # 자동 시작점으로 도장 경로 생성
# 또는
python first.py       # 사용자 클릭으로 시작점 지정
python paview.py      # T_ms_overall.txt 적용 + mm 변환
```

### 🎬 4. 디버그/시각화
```bash
python ran.py                   # 노이즈 제거 + 시각화
python path_edge_display.py     # X 최소 모서리 점 시각화
python paview1.py               # 변환 결과 시각화 (시각화 활성화 필요)
```

### 🛠️ 5. 라이다 데이터 수집 (ROS2 환경)
```bash
ros2 run lidar_collector dual_lidar_node  # 듀얼 라이다 노드
# 또는 단일 라이다 (lidar_node.cpp 빌드 후)
ros2 run <pkg> lidar_node
```

---

## 📌 부록: 좌표계와 단위

### 좌표계
- **라이다 좌표계**: 각 라이다의 자체 좌표계 (mm 단위로 저장됨)
- **월드 좌표계**: 로봇 베이스 기준 (m 단위, calib_dual.txt에 변환 행렬)
- **모델 좌표계**: STL/gene1.pcd의 좌표계 (m 단위)

### 단위 변환
| 데이터 | 입력 단위 | 처리 단위 | 출력 단위 |
|--------|----------|----------|----------|
| 라이다 PCD | mm | m (×0.001) | m |
| 모델 STL | m | m | m |
| robot_path.json | m | m | m |
| iktarget/*_mm.json | m → ×1000 | mm | mm (로봇 IK 입력) |

### ICP 변환 방향
- `transformation_matrix.txt` : Model → Scan
- `transformation_matrix_inverse.txt` : Scan → Model (사용)
- `T_ms_overall.txt` : C++ + CloudComPy 누적 변환 (model space → scan space의 역변환)

---

**작성일**: 2026-05-06
**작성자**: 코드 자동 분석 (Claude Code)
**프로젝트**: 마젠타로보틱스 듀얼 라이다 도장 자동화
