# Calibration 패키지 코드 분석

---

## 목차

1. [전체 개요](#1-전체-개요)
2. [디렉토리 구조](#2-디렉토리-구조)
3. [패키지 메타 파일](#3-패키지-메타-파일)
4. [calibration/ 루트 모듈](#4-calibration-루트-모듈)
5. [common/ — 공통 유틸리티](#5-common--공통-유틸리티)
6. [stage1/ — 단일 유닛 캘리브레이션](#6-stage1--단일-유닛-캘리브레이션)
7. [stage2/ — 다중 유닛 Inter-system 캘리브레이션](#7-stage2--다중-유닛-inter-system-캘리브레이션)
8. [tools/ — 보조 도구](#8-tools--보조-도구)
9. [test/ — 단위 테스트](#9-test--단위-테스트)
10. [데이터 흐름 요약](#10-데이터-흐름-요약)
11. [주요 수식 / 좌표 변환 약속](#11-주요-수식--좌표-변환-약속)

---

## 1. 전체 개요

본 패키지는 **이중 보드(Dual-Board) + 2단계(Two-Stage)** 방식으로 다중 카메라/LiDAR 시스템을 캘리브레이션합니다.

### 1.1 단계별 역할

| 단계 | 보드 | 대상 | 산출물 |
| --- | --- | --- | --- |
| **Stage 1** | Board A (r2) — ChArUco 8×8 + 콘 4개 | 단일 유닛 (per-unit) | 카메라 내부 파라미터 K, D + LiDAR↔카메라 외부 변환 `T_lidar_to_cam` |
| **Stage 2** | Board B (r3) — ChArUco 6×6 + 콘 4개 | 4+1 유닛 동시 관측 | 유닛 간 외부 변환 `T_unit_to_world` (월드 프레임 기준) |

### 1.2 파이프라인 흐름

```
[Stage 1] (각 유닛별 독립)
  Step 1a: 내부 캘리브레이션 (cv2.calibrateCamera + ChArUco)
  Step 1b: LiDAR-카메라 초기 외부 변환 (Rigid-3D 정합)
  Step 1c: Joint BA (scipy least_squares, TRF + Huber)
       │
       ▼  → intrinsic.yaml + lidar_to_cam.yaml

[Stage 2] (전체 유닛 동기 캡처)
  Step 2a: 동기 다중 유닛 캡처
  Step 2b: 월드 프레임 유닛 기준 초기 T_unit_to_world (SE(3) median)
  Step 2c: Multi-unit Joint BA (Stage 1 결과를 prior로 사용)
       │
       ▼  → unit_extrinsics.yaml
```

### 1.3 검출/관측 잔차

BA에서 사용하는 잔차 항목:

- **E_reproj_corner** — ChArUco 코너 재투영 오차 (2D)
- **E_reproj_apex** — 콘 정점(apex) 재투영 오차 (2D, optional)
- **E_apex3D** — LiDAR 검출 콘 apex와 보드 좌표계 기대값의 3D 거리 오차
- **E_prior_lc** (Stage 2 only) — Stage 1 결과를 가우시안 prior로 사용

### 1.4 주요 의존성

| 항목 | 필수 여부 | 용도 |
| --- | --- | --- |
| `numpy` | 필수 | 수치 연산 전반 |
| `scipy` | 필수 | least_squares (BA), cKDTree (DBSCAN) |
| `pyyaml` | 필수 | 설정/결과 YAML |
| `opencv-python` | 필수 | ChArUco 검출, PnP, 재투영 |
| `flask` | 선택 | `/calib` REST API + 미니 UI |
| `open3d` | 선택 | PCD 로딩 (없으면 자체 파서 fallback) |
| `matplotlib` | 선택 | 잔차 히스토그램 시각화 |

---

## 2. 디렉토리 구조

```
calibration/
├── package.xml              # ROS2 package manifest
├── setup.py                 # ament_python setup + console scripts
├── resource/
│   └── calibration          # ament resource marker (빈 파일)
├── launch/                  # (현재 비어 있음)
├── test/
│   └── test_basic.py        # pytest 단위 테스트
└── calibration/             # Python 패키지 루트
    ├── __init__.py
    ├── bridge.py            # mainwindow.iface 브리지
    ├── session_manager.py   # 세션 디렉토리/캡처 관리
    ├── web_bp.py            # Flask blueprint (/calib)
    ├── common/              # 공통 모듈
    │   ├── __init__.py
    │   ├── io_yaml.py
    │   ├── transforms.py
    │   ├── board_definition.py
    │   ├── charuco_detect.py
    │   ├── cone_ransac.py
    │   ├── cone_detect_lidar.py
    │   ├── cone_detect_image.py
    │   ├── validation.py
    │   └── visualization.py
    ├── stage1/              # 단일 유닛 캘리브레이션
    │   ├── __init__.py
    │   ├── step1a_intrinsic.py
    │   ├── step1b_lidar_cam_init.py
    │   ├── joint_ba_scipy.py
    │   └── stage1_runner.py
    ├── stage2/              # 다중 유닛 캘리브레이션
    │   ├── __init__.py
    │   ├── step2b_init.py
    │   ├── joint_ba_scipy.py
    │   └── stage2_runner.py
    └── tools/               # 보조 도구
        ├── __init__.py
        ├── _board_generator.py
        ├── generate_board_a_r2.py
        ├── generate_board_b_r3.py
        ├── verify_install.py
        ├── synth_smoke_test.py
        └── synth_stage1_test.py
```

---

## 3. 패키지 메타 파일

### 3.1 [`package.xml`](package.xml)

ROS2 패키지 매니페스트(`format=3`).

| 키 | 값 | 의미 |
| --- | --- | --- |
| `name` | `calibration` | 패키지 이름 |
| `version` | `0.1.0` | |
| `description` | "Dual-board two-stage (camera + LiDAR) calibration pipeline for 4+1 gantry units." | |
| `depend` | `rclpy`, `std_msgs`, `std_srvs`, `sensor_msgs`, `sensor_msgs_py` | ROS2 빌드/런타임 의존 |
| `exec_depend` | `sensor_cam_main` | UI/카메라 매니저 패키지 |
| `build_type` | `ament_python` | |

### 3.2 [`setup.py`](setup.py)

`ament_python` 빌드 진입점. 콘솔 스크립트 5종 등록:

| 콘솔 명 | 진입 함수 | 설명 |
| --- | --- | --- |
| `calib_stage1` | `calibration.stage1.stage1_runner:main_cli` | Stage 1 CLI |
| `calib_stage2` | `calibration.stage2.stage2_runner:main_cli` | Stage 2 CLI |
| `calib_generate_board_a` | `calibration.tools.generate_board_a_r2:main` | Board A 인쇄용 PNG |
| `calib_generate_board_b` | `calibration.tools.generate_board_b_r3:main` | Board B 인쇄용 PNG |
| `calib_verify_install` | `calibration.tools.verify_install:main` | 설치 점검 |

---

## 4. calibration/ 루트 모듈

### 4.1 [`__init__.py`](calibration/__init__.py)

패키지 docstring + `__version__ = '0.1.0'`.

### 4.2 [`bridge.py`](calibration/bridge.py) — Mainwindow 브리지

`sensor_cam_main.mainwindow`의 전역 `iface = CameraManager()` 객체에 안전하게 접근하는 어댑터 모듈.

- **`get_iface()`**: lazy import. 임포트 실패 시 명확한 RuntimeError를 던져, ROS2/Flask가 없는 CI에서도 다른 모듈은 단독 테스트 가능.
- **`set_pcl_target_override(map)`**: `iface._pcl_target_override`에 `{unit: 파일 개수}` 매핑을 설정. 한 파일당 5초 적분 기준이므로 `N → N×5초` 적분.
- **`clear_pcl_target_override()`**: 위 오버라이드 초기화.
- **`wait_pcl_complete(unit, timeout_s=90, poll_interval_s=0.5)`**: `iface.get_pcl_status(unit)`을 폴링하다가 `is_collecting=False` & `last_saved_pcd_path` 존재 시 절대 경로 반환. 타임아웃 시 `TimeoutError`.

### 4.3 [`session_manager.py`](calibration/session_manager.py) — 세션/캡처 관리

세션 디렉토리 레이아웃과 동기 캡처 로직.

#### 4.3.1 `PoseMeta` (dataclass)

| 필드 | 타입 | 기본값 | 설명 |
| --- | --- | --- | --- |
| `stage` | Literal['stage1', 'stage2'] | — | 단계 |
| `step` | Literal['intrinsic', 'lidar_cam', 'interstage'] | 'interstage' | 캡처 단계 |
| `pose_index` | int | 0 | 포즈 번호 |
| `tilt_deg` | float | 0.0 | 보드 틸트 |
| `distance_m` | Optional[float] | None | 보드 거리 |
| `stand_position_m` | Optional[tuple] | None | 스탠드 위치 |
| `note` | str | '' | 메모 |

#### 4.3.2 `new_session_id(prefix='')`

`YYYYMMDD_HHMMSS_<prefix>` 형식 ID 반환. (`_` 구분자, prefix 없으면 타임스탬프만)

#### 4.3.3 `CalibrationSession` 클래스

생성자 인자: `stage`, `session_id`, `board_yaml`, `units`, `out_root='/root/sam_ws/calibration_data'`.

**디렉토리 규약**:

- **Stage 1**: `<root>/stage1/<unit>/<session>/<step>/poseNNN/`
  - `step == 'intrinsic'`: poseNNN 디렉토리 없이 평면 디렉토리에 `img_poseNNN.png`
- **Stage 2**: `<root>/stage2/<session>/poseNNN/` — 모든 유닛 한 디렉토리 공유

생성 시 `board_yaml`이 존재하면 `sha256_file()`로 해시 계산해 `_board_sha256`에 보관 → meta.json 무결성 추적.

**캡처 메서드**:

- **`capture_stage1_intrinsic(unit, pose) → str`**: `iface.img_save(unit)` 호출 후 `cam.png`를 `pose_dir/img_pose{NNN}.png`로 이동.
- **`capture_stage1_lidar_cam(unit, pose, lidar_settle_s=2.0) → {'image', 'lidar'}`**:
  1. `iface.pc_save_start(unit)` 시작
  2. `lidar_settle_s` 대기
  3. `iface.img_save(unit)` 즉시 캡처
  4. `bridge.wait_pcl_complete(unit, 90s)` 대기
  5. `cam.png` / `lidar.pcd` 로 이동
  6. `_write_meta()`로 `meta.json` 기록
- **`capture_stage2_pose(pose, lidar_settle_s=2.0, pcl_target_override=3)`**: 다중 유닛 동기 캡처.
  1. `bridge.set_pcl_target_override({u: N for all units})` (3 → 15초 적분)
  2. 모든 유닛 LiDAR 시작 → settle
  3. 모든 유닛 이미지 연속 저장
  4. 모든 유닛 LiDAR 완료 대기
  5. `cam_<unit>.png` / `lidar_<unit>.pcd`로 이동, meta.json 기록

#### 4.3.4 `_write_meta()`

각 포즈 디렉토리에 `meta.json`을 생성:

```json
{
  "session_id": "...",
  "stage": "stage1|stage2",
  "pose_index": 1,
  "tilt_deg": 0.0,
  "distance_m": null,
  "stand_position_m": null,
  "board_yaml": "/path/to/board.yaml",
  "board_params_sha256": "abc123...",
  "timestamp_utc": "YYYY-MM-DDTHH:MM:SSZ",
  "capture": { "<unit>": {"image": "...", "lidar": "...", "image_sha256": "..."} },
  "notes": ""
}
```

### 4.4 [`web_bp.py`](calibration/web_bp.py) — Flask Blueprint

`/calib` 접두사로 mainwindow의 Flask 앱에 등록되는 blueprint.

```python
from calibration.web_bp import calib_bp
app.register_blueprint(calib_bp, url_prefix='/calib')
```

**중요**: Flask가 미설치된 환경에서도 안전하게 import 가능 — `Blueprint`가 `None`이면 `calib_bp = None`.

#### 4.4.1 엔드포인트 요약

| Method | Path | 기능 |
| --- | --- | --- |
| GET | `/ping` | 서비스 헬스 체크 (`{"ok": true, "service": "calibration", "version": "0.1.0"}`) |
| GET | `/sessions` | 모든 세션 상태 dict 반환 |
| GET | `/session/<sid>/status` | 단일 세션 상태 |
| POST | `/stage1/create` | Stage 1 세션 생성 (`unit`, `board_yaml?`, `session_id?`) |
| POST | `/stage2/create` | Stage 2 세션 생성 (`units`, `board_yaml?`, `session_id?`) |
| POST | `/capture_pose` | 단일 포즈 캡처 (`session_id`, `step?`, `pose_index?`, `tilt_deg?`, `distance_m?`, `note?`) |
| POST | `/stage1/run_ba` | 백그라운드 스레드에서 Stage 1 BA 실행 |
| POST | `/stage2/run_ba` | 백그라운드 스레드에서 Stage 2 BA 실행 |
| GET | `/ui` | 임베디드 미니 HTML/JS UI |

#### 4.4.2 동시성

- 모듈 전역 `_sessions: Dict[str, Dict]` (메타데이터), `_live: Dict[str, CalibrationSession]` (활성 세션 객체).
- `_lock = threading.Lock()`으로 보호.
- BA는 daemon thread에서 실행되며 진행률 콜백으로 상태 갱신:
  - `running` → `done`/`failed`/`error`

#### 4.4.3 미니 UI

`_CALIB_UI_HTML` 문자열에 포함된 단일 HTML+JS:
- Stage 1 카드: 유닛 선택 → Create Session / Capture Intrinsic / Capture LiDAR-Cam / Run BA
- Stage 2 카드: 유닛 콤마 입력 → Create Session / Capture (틸트 0/15/30) / Run BA
- 2초마다 `/calib/sessions` polling

---

## 5. common/ — 공통 유틸리티

### 5.1 [`io_yaml.py`](calibration/common/io_yaml.py) — YAML/SHA256

| 함수 | 설명 |
| --- | --- |
| `load_yaml(path) → dict` | PyYAML safe_load. PyYAML 미설치 시 ImportError. |
| `save_yaml(path, data)` | safe_dump (`sort_keys=False`, `allow_unicode=True`, `indent=2`). 부모 디렉토리 자동 생성. |
| `sha256_file(path) → hex str` | 64KB 청크 단위 SHA-256 해시. |

### 5.2 [`transforms.py`](calibration/common/transforms.py) — SE(3)/so(3) 수학

순수 numpy(+ optional scipy) 구현.

| 함수 | 입출력 | 설명 |
| --- | --- | --- |
| `skew(v)` | (3,) → (3,3) | 외적 행렬 [v]× |
| `rodrigues(rvec)` | (3,) → (3,3) | so(3) → SO(3). theta<1e-12면 I 반환 |
| `rotation_matrix_to_rvec(R)` | (3,3) → (3,) | 역 Rodrigues. theta≈π 대칭 케이스 별도 처리 |
| `se3_from_rvec_tvec(rvec, tvec)` | → (4,4) | 4×4 동차 행렬 구성 |
| `se3_to_rvec_tvec(T)` | (4,4) → (rvec, tvec) | |
| `se3_inverse(T)` | (4,4) → (4,4) | (R, t) → (Rᵀ, -Rᵀt) |
| `se3_compose(*Ts)` | n개 → (4,4) | 좌→우 행렬 곱 |
| `transform_points(T, pts)` | (4,4),(N,3) → (N,3) | `pts @ R.T + t` |
| `rigid_transform_3d(src, dst)` | (N,3),(N,3) → (4,4) | Umeyama (no scale). reflection 보정. |
| `se3_log(T)` | (4,4) → (6,) | (omega, v) tangent 표현. theta→0 한계 처리 |
| `se3_median(Ts)` | List[(4,4)] → (4,4) | 측지 L1 중간값. 가중치 = 1/거리, 10회 반복 |
| `angle_between(v1, v2)` | → float (rad) | 안전한 acos (clip [-1,1]) |
| `rvec_from_matrix(R)` / `matrix_from_rvec(rvec)` | 별칭 | |

> **수치 안전장치**: 모든 변환에서 노름·tan(θ/2)에 clamp/검사 적용. SVD 기반 Umeyama는 `det(R)<0` 시 마지막 V 행 부호 반전.

### 5.3 [`board_definition.py`](calibration/common/board_definition.py) — 보드 정의 로더

ChArUco 보드 + 콘 4개 형상을 YAML에서 로드.

#### 5.3.1 dataclasses

- **`ConeDef`**: id, base_center_m(3,), apex_m(3,), diameter_m, height_m, half_angle_rad, measured(bool)
- **`CharucoParams`**: squares_x/y, square_length_m, marker_length_m, aruco_dict_name, marker_id_range, n_inner_corners, min_corners_for_valid
- **`BoardDef`**: name, outer_size_m(tuple), plane_thickness_m, charuco, cones[], detection(dict), aruco_detector(dict), yaml_path, yaml_sha256

#### 5.3.2 메서드

| 메서드 | 반환 | 설명 |
| --- | --- | --- |
| `BoardDef.load(path)` | BoardDef | YAML 파싱 + half_angle_deg→rad 자동 변환 + sha256 |
| `get_cv_aruco_dict()` | cv2.aruco.Dictionary | YAML name → `cv2.aruco.<NAME>` 룩업 |
| `get_cv_charuco_board()` | cv2.aruco.CharucoBoard | OpenCV 4.5/4.6/4.7+ 다중 API 호환 (3-fallback) |
| `apex_3d_board()` | (4,3) | id 정렬된 콘 apex 좌표 |
| `base_center_3d_board()` | (4,3) | id 정렬된 콘 base 좌표 |
| `expected_apex_distances_m()` | (4,4) | 콘 apex 간 기대 거리 행렬 |

### 5.4 [`charuco_detect.py`](calibration/common/charuco_detect.py) — ChArUco 검출

OpenCV 4.7+ `CharucoDetector` API와 4.6 legacy API 양쪽 지원.

#### 5.4.1 `ChArUcoDetection` dataclass

| 필드 | 타입 | 설명 |
| --- | --- | --- |
| `corners_2d` | (N,2) float64 | 픽셀 좌표 |
| `corners_3d_board` | (N,3) float64 | 보드 프레임 (z=0) |
| `ids` | (N,) int32 | charuco 코너 id |
| `marker_corners` | list | 원본 ArUco 마커 4점 |
| `marker_ids` | (M,) | ArUco 마커 id |

`count` property: ids 길이.

#### 5.4.2 검출 흐름

`detect_charuco(image, board_def)`:
1. BGR → GRAY 자동 변환
2. `_build_detector_params(board_def.aruco_detector)`:
   - adaptiveThreshold 윈도우 min/max/step
   - minMarkerPerimeterRate
   - cornerRefinementMethod (`SUBPIX`/`CONTOUR`/`APRILTAG`/`NONE`)
   - cornerRefinementWinSize / MaxIterations
3. **Path A** (OpenCV 4.7+): `cv2.aruco.CharucoDetector(board, ch_params, params).detectBoard(gray)` 호출
4. **Path B** (OpenCV 4.6): `detectMarkers` → `interpolateCornersCharuco`
5. 검출 코너 수가 `min_corners_for_valid` 미만이면 `None`
6. **3D 매칭**: `cv_board.matchImagePoints(corners, ids)` 우선 시도 → 실패 시 squares grid에서 직접 계산 fallback
7. `ChArUcoDetection` 반환

#### 5.4.3 PnP

`charuco_pnp(detection, K, D)`:
- `cv2.solvePnP(SOLVEPNP_ITERATIVE)` → `T_cam_to_board` (4×4)
- count<4 또는 실패 시 None

### 5.5 [`cone_ransac.py`](calibration/common/cone_ransac.py) — 콘 RANSAC 피팅

축(`axis`)과 반각(`half_angle`) prior가 있는 단순화 모델 — apex 위치 (3D)만 추정.

#### 5.5.1 콘 잔차 함수

점 P, apex A, 단위 축 u, 반각 α일 때:

```
v          = P - A
proj_axis  = v · u
perp       = v - proj_axis · u
d_perp     = ||perp||
잔차       = d_perp - |proj_axis| · tan(α)
```

#### 5.5.2 Gauss-Newton apex refinement

`_refine_apex(pts, axis, half_angle, apex_init, max_iters=50, tol=1e-8)`:
- `dr/d(apex) = -perp_unit + sign(proj_axis) · tan(α) · u` (perp_unit·u=0 이용)
- 정규방정식 `H Δ = -g`, `Δ` 갱신, ‖Δ‖<tol이면 종료

#### 5.5.3 RANSAC 본체

```python
fit_cone_ransac(pts, axis_prior, half_angle_rad, tol_m,
                min_inliers_frac=0.6, n_iters=50,
                apex_init_mode='centroid_along_axis',
                apex_offset_from_centroid_m=0.05, seed=42)
```

루프:
1. `centroid ± axis × offset` (혹은 random_point)으로 초기 apex 후보
2. GN refine
3. 잔차로 inlier 결정 → 비율이 `min_inliers_frac` 미만이면 스킵
4. inlier로 한 번 더 refine → RMS 계산
5. (n_inliers, RMS)로 best 후보 갱신

루프 종료 후 best가 없으면 RANSAC 없는 직접 refine fallback (>=4 inlier일 때).

반환: `ConeFitResult(apex, axis, half_angle_rad, inliers, rms_m, n_inliers)`.

### 5.6 [`cone_detect_lidar.py`](calibration/common/cone_detect_lidar.py) — LiDAR 콘 검출

#### 5.6.1 PCD 로딩

`load_pcd(path)`:
1. open3d 시도 → `np.asarray(pcd.points)` 비어 있지 않으면 반환
2. fallback `_read_pcd_simple()` — ASCII / binary(float32 xyz) PCD 파서. binary_compressed는 미지원.

#### 5.6.2 평면 RANSAC

`ransac_plane(pts, tol_m, n_iters=2000, rng_seed=42)`:
- 3점 샘플 → 정규화된 normal `n`, `d=-n·p1`
- inlier 가장 많은 평면 채택
- inlier 셋에 대해 공분산 행렬의 최소 고유벡터로 refine
- 반환: `(plane_abcd, inlier_mask)`

#### 5.6.3 DBSCAN

`_dbscan(pts, eps, min_samples)`:
- scipy `cKDTree`로 이웃 탐색
- 표준 DBSCAN: 시드 → BFS 확장
- 노이즈 = -1, 군집 0..K-1 반환

#### 5.6.4 통합 검출

`detect_cones_lidar(pcd_path, board_def) → (4,3) | None`:
1. PCD 로딩 (<100점이면 None)
2. RANSAC 평면 (`detection.plane_tol_m`, `detection.plane_iters`)
3. **법선 방향 자동 보정**: `median(signed) > 0`이면 normal/d 부호 반전 (보드 위쪽 향하도록)
4. 평면 위로 `height_margin_frac × cone_h` 이상인 후보 점 추출
5. DBSCAN (`detection.dbscan_eps_m`, `detection.min_cluster_pts`)
6. 군집 수 <4면 None, ≥4면 큰 4개만 채택
7. 각 군집에 `fit_cone_ransac()` 적용 → apex 4개 수집
8. **순서 매칭**: `_order_apexes_by_geometry()`로 보드 정의 순서와 정렬

#### 5.6.5 apex 순서 정렬

`_order_apexes_by_geometry(apexes_lidar, board_def)`:
- 보드 expected `apex_3d_board`와의 pairwise 거리 행렬 비교
- 4! = 24개 순열 brute force → distance-matrix Frobenius 거리가 최소인 순열 채택

### 5.7 [`cone_detect_image.py`](calibration/common/cone_detect_image.py) — 이미지 콘 apex 검출

전략: `T_cam_board`로 apex를 이미지에 투영해 시드 → ROI 내 탐색.

| 함수 | 설명 |
| --- | --- |
| `project_points(pts_board, T_cam_board, K, D) → (N,2)` | `cv2.projectPoints`로 보드 점 투영 |
| `_se3_to_rvec_tvec(T)` | 내부 헬퍼 |
| `detect_apex_in_image(image, seed_xy, roi_size_px=60, threshold='otsu')` | 단일 apex 검출 |
| `detect_cone_apexes_in_image(image, board_def, T_cam_board, K, D, roi_size_px=60)` | 4개 apex 일괄 검출 |

`detect_apex_in_image` 단계:
1. Gray 변환
2. seed 중심 ROI (`(±r) × (±r)`, 경계 클립)
3. Otsu 이진화 (혹은 임계값 직접)
4. `findContours(RETR_EXTERNAL)` → 최대 면적 컨투어
5. `convexHull` → 컨투어 중심에서 가장 먼 hull 정점이 apex 후보
6. `cv2.cornerSubPix(WIN=5, CRIT=EPS+ITER, max=50)` 서브픽셀 보정
7. ROI 좌상단 오프셋 더해 절대 좌표 반환

### 5.8 [`validation.py`](calibration/common/validation.py) — 검증 헬퍼

| 함수 | 설명 |
| --- | --- |
| `reprojection_rms(obj, img, rvec, tvec, K, D) → float` | 픽셀 RMS |
| `per_corner_max_error(...)` | 코너별 최대 오차 |
| `inter_apex_distance_error(got, expected) → float` | apex 거리 행렬 비교 (max abs) |
| `plane_rms(pts, abcd) → float` | 점들의 평면까지의 거리 RMS |
| `compare_criteria(value_dict, pass_dict) → dict` | 키별 임계값 비교. `pass_dict[k] = (op, thr)`. `_all_pass` 합산 |

지원 비교 연산자: `<`, `<=`, `>`. 누락된 값은 `pass=False`.

### 5.9 [`visualization.py`](calibration/common/visualization.py) — 시각화

| 함수 | 설명 |
| --- | --- |
| `overlay_charuco(image, detection, out_path)` | 검출 코너에 녹색 점 + id 라벨 |
| `project_lidar_onto_image(image, pcd_pts_cam, K, D, out_path, max_distance_m=25)` | LiDAR 포인트를 카메라 좌표로 가정하고 깊이 컬러맵 (가까움=빨강, 멀음=파랑) |
| `residual_histogram(residuals, out_path, title, xlabel, bins=50)` | matplotlib (Agg 백엔드) 히스토그램 |

> matplotlib import 실패 시 `False` 반환하고 동작 스킵 — optional 의존성 처리.

---

## 6. stage1/ — 단일 유닛 캘리브레이션

### 6.1 [`step1a_intrinsic.py`](calibration/stage1/step1a_intrinsic.py) — 내부 파라미터

#### 6.1.1 함수 요약

- **`collect_intrinsic_images(session_dir)`**: `<session>/intrinsic/*.{png,jpg,jpeg}` (대소문자) 정렬 리스트.
- **`calibrate_intrinsic(image_paths, board_def, image_size, flags=0) → dict`**: 핵심 함수.
- **`get_opencv_flags(names) → int`**: 문자열 리스트 → bit-mask. 예: `['CALIB_RATIONAL_MODEL', 'CALIB_FIX_K4']`

#### 6.1.2 `calibrate_intrinsic` 흐름

1. 이미지 순회 — `cv2.imread(GRAYSCALE)` → `detect_charuco`
2. 검출 코너수 < `min_corners_for_valid`인 이미지는 제외
3. 유효 이미지 6장 미만이면 RuntimeError
4. `cv2.calibrateCamera(all_obj, all_img, image_size, None, None, flags=flags)`
5. **per-image 진단**:
   - `per_image_rms` (각 이미지의 재투영 RMS)
   - `max_per_corner_px` (모든 코너 중 최대 오차)
6. 반환 dict: `K, D, rms_px, per_image_rms, max_per_corner_px, n_images, n_corners_per_image, used_paths, rvecs, tvecs, image_size`

### 6.2 [`step1b_lidar_cam_init.py`](calibration/stage1/step1b_lidar_cam_init.py) — LiDAR↔카메라 초기 변환

#### 6.2.1 흐름

각 `lidar_cam/pose*` 디렉토리에서:
1. `cam.png` + `lidar.pcd` 존재 확인
2. ChArUco 검출 → PnP → `T_cam_board`
3. (옵션) 이미지 콘 apex 4점
4. LiDAR 콘 apex 4점 (실패 시 해당 포즈 스킵)
5. `T_lidar_to_board = rigid_transform_3d(apex_3d_lidar → apex_3d_board)`
6. `T_lidar_to_cam = T_cam_board @ inverse(T_lidar_to_board)`

> ⚠️ **표기 약속**: `T_cam_board`는 *board → cam* 좌표 변환을 수행 (점을 곱했을 때 카메라 좌표가 나옴). 이는 OpenCV solvePnP가 출력하는 (rvec, tvec) 의미와 일치.

#### 6.2.2 함수

- **`collect_lidar_cam_poses(session_dir)`**: `<session>/lidar_cam/pose*` 디렉토리 리스트.
- **`init_lidar_cam_observations(pose_dirs, board_def, K, D) → List[dict]`**: 위 흐름으로 관측 dict 생성. 각 dict 키: `pose_dir, T_cam_board, T_lidar_board, T_lidar_cam, corners_2d, corners_3d_board, ids, apex_2d, apex_3d_lidar, img_path, pcd_path`.
- **`median_lidar_cam(observations) → (4,4) | None`**: `se3_median([T_lidar_cam])` — 강건 초기값.

### 6.3 [`joint_ba_scipy.py`](calibration/stage1/joint_ba_scipy.py) — Stage 1 Joint BA

#### 6.3.1 변수 레이아웃

`x` = `[T_lidar_to_cam(6) | T_cam_to_board₀(6) | … | T_cam_to_boardₙ₋₁(6)]`

총 차원 = 6 + 6N (단일 유닛 N개 포즈)

> K, D는 Stage 1a 결과로 고정. 본격 Joint BA(intrinsic+extrinsic 동시 최적화)는 의도적으로 범위에서 제외.

#### 6.3.2 잔차

| 항목 | 가중치 (default) | 의미 |
| --- | --- | --- |
| `r_corner` | 1.0 | ChArUco 코너 재투영 (2D, 픽셀) |
| `r_apex` | 3.0 | 콘 apex 재투영 (2D, optional) |
| `r_apex3d` | 15.0 | LiDAR apex → 보드 좌표계 변환 후 expected와의 3D 오차 |

3D apex 변환 식:
```
T_lidar_to_board = inverse(T_cam_to_board) @ T_lidar_to_cam
apex_in_board    = T_lidar_to_board · apex_lidar
r_apex3d         = apex_in_board - apex_3d_board (expected)
```

> 코드 내부 표기 주의: `T_cb`는 `T_board_to_cam`을 의미하며 `transform_points(T_cb, board_pt) = cam_pt` 식으로 사용.

#### 6.3.3 솔버 호출

`scipy.optimize.least_squares`:
- method: `trf` (Trust Region Reflective)
- max_nfev: `max_iters × max(10, len(x0))`
- xtol/ftol: 1e-10 기본
- robust loss: `huber` (k=1.0 default)

#### 6.3.4 반환 `Stage1BAResult`

- `T_lidar_to_cam` (4,4)
- `T_cam_to_board` (List[(4,4)])
- `residuals_corner_px` (concat 1-D)
- `residuals_apex_px` (concat 1-D | None)
- `residuals_apex_3d_m` (concat 1-D)
- `optimiser_info`: cost, n_evals, status, message, n_variables, n_residuals

### 6.4 [`stage1_runner.py`](calibration/stage1/stage1_runner.py) — Stage 1 오케스트레이터

#### 6.4.1 `run_stage1(unit, session_id, ...)`

기본 경로:
- data: `/root/sam_ws/calibration_data/stage1`
- results: `/root/sam_ws/calibration_results/stage1`
- board: `/root/sam_ws/config/board_a_r2_params.yaml`
- config: `/root/sam_ws/config/stage1_config.yaml`
- image_size: `(4000, 4000)`

진행률 콜백 `progress_cb(progress, message)`로 5%~100% 갱신.

##### 단계별

1. **Step 1a (5–30%)**: 내부 파라미터
   - 이미지 ≥10장 강제 (없으면 RuntimeError)
   - cfg `intrinsic.opencv_flags`로 마스크 구성
   - `intrinsic.yaml` 출력
   - PASS 조건: `rms_px < pass_rms (0.15)` & `max_per_corner_px < pass_per_corner (0.5)`

2. **Step 1b (35–50%)**: LiDAR-카메라 초기
   - 포즈 ≥5개 필요, 유효 관측 ≥3개 필요
   - `T_lc_init = se3_median(...)`

3. **Step 1c (60–90%)**: Joint BA

4. **검증 + 저장 (90–100%)**:
   - `lidar_to_cam.yaml`: rotation_matrix, translation_m, rvec, residuals, ba_info, board info
   - `validation_report.md` 생성: 조건별 PASS/FAIL + 수치
   - 종합 PASS = 모든 임계값 통과
     - `intrinsic_rms_px`
     - `per_corner_max_px`
     - `apex_3d_mm` (cfg `lidar_cam.pass_apex_3d_mm`, 기본 1.5mm)

#### 6.4.2 출력 YAML 예시 (`intrinsic.yaml`)

```yaml
camera_matrix: { fx, fy, cx, cy }
distortion: { model: RATIONAL|KANNALA, coeffs: [k1, k2, p1, p2, k3, ...] }
image_size: [W, H]
residuals: { reprojection_rms_px, reprojection_max_px, n_images }
calibration_timestamp: "...Z"
session_id: "..."
board: "board_a_r2"
```

> distortion 모델 추정: D 길이 >4면 RATIONAL(8/12/14 계수), 4 이하면 KANNALA로 라벨링.

#### 6.4.3 CLI (`main_cli`)

```bash
calib_stage1 --unit Gantry_Global1 \
             --session-id 20250506_120000 \
             [--data-root .] [--results-root .] \
             [--board-yaml ...] [--config ...] \
             [--image-width 4000] [--image-height 4000]
```

종료 코드: 0=PASS, 1=ERROR, 2=FAIL.

---

## 7. stage2/ — 다중 유닛 Inter-system 캘리브레이션

전략 **C**: Stage 1 결과(`T_lidar_to_cam`)를 가우시안 prior로 사용하면서 모든 유닛의 `T_unit_to_world`를 동시 최적화.

### 7.1 [`step2b_init.py`](calibration/stage2/step2b_init.py) — 초기 외부 변환

#### 7.1.1 함수

- **`collect_stage2_poses(session_dir)`**: `<session>/pose*` 디렉토리 정렬 리스트.
- **`detect_all_units(pose_dir, units, board_def, intrinsics) → Dict[u, dict] | None`**:
  모든 유닛에 대해 `cam_<u>.png` + `lidar_<u>.pcd` 검출. **하나라도 실패하면 None** (해당 포즈는 BA에서 제외).
  유닛별 dict: `T_cam_board, corners_2d, corners_3d_board, ids, apex_2d, apex_3d_lidar, img_path, pcd_path`.

#### 7.1.2 초기 유닛 외부 변환 계산

`compute_initial_unit_extrinsics(per_pose_obs, world_frame_unit, units) → Dict[u, (4,4)]`:

월드 프레임 유닛 `w`를 기준으로:

```
T_unit_u_to_world = inverse(T_world_cam_to_board) @ T_unit_cam_to_board
```

각 포즈에서 위 식으로 샘플 수집 → `se3_median(samples)`로 강건 추정.

> 의미: 같은 보드를 두 유닛이 보고 있으므로, 보드를 매개로 두 카메라 간의 상대 변환 추출 가능.

### 7.2 [`joint_ba_scipy.py`](calibration/stage2/joint_ba_scipy.py) — Stage 2 Joint BA

#### 7.2.1 변수 레이아웃

`x` = (월드 유닛 제외 N-1 유닛의 `T_u_to_world(6)`)
   ‖ (모든 N 유닛의 `T_lc(6)`)
   ‖ (P 포즈 각각의 `T_board_to_world(6)`)

총 차원 = `6(N-1) + 6N + 6P = 12N + 6P - 6`

월드 프레임 유닛 = `np.eye(4)` 고정.

#### 7.2.2 잔차

각 (포즈 p, 유닛 u)에 대해:

```
T_world_to_cam_u  = inverse(T_u_to_world)
T_board_to_cam_u  = T_world_to_cam_u @ T_board_to_world
```

| 잔차 | 가중치 default | 단위 |
| --- | --- | --- |
| corner reprojection | `weights.reproj_corner = 1.0` | px |
| apex reprojection (optional) | `weights.reproj_apex = 3.0` | px |
| apex 3D (LiDAR→board) | `weights.apex_3d = 10.0` | m |
| **prior on T_lc** (per-unit, vs Stage 1) | `weights.prior_lc = 1.0` | rad/m |

prior는 SE(3) log 잔차에 표준편차 정규화:
```
delta      = se3_log(inverse(T_lc_prior) @ T_lc)
r_prior[0:3] = delta[0:3] / σ_rot
r_prior[3:6] = delta[3:6] / σ_trans
```
- `prior.sigma_rot_deg` (default 0.1°), `prior.sigma_trans_m` (default 5mm).

#### 7.2.3 솔버

`least_squares`(method=`trf`, robust loss `huber`, k=1.0):
- max_iters default 500 (Stage 1보다 큼)
- xtol/ftol 1e-12

#### 7.2.4 공분산 근사

`_approx_sigma_from_jac(result, ...)`:
- result.jac가 있으면 `J^T J`의 의사역행렬 대각으로 분산 추정
- `T_u_to_world` 6 파라미터에 대해 σ_rot(deg), σ_trans(mm)
- 실패해도 0벡터 반환 (안전)

#### 7.2.5 반환 `Stage2BAResult`

- `T_unit_to_world: Dict[u, (4,4)]`
- `T_lidar_to_cam: Dict[u, (4,4)]`
- `T_board: List[(4,4)]` — 각 포즈의 보드→월드
- `residuals_corner_px`, `residuals_apex_px`, `residuals_apex_3d_m`
- `optimiser_info`
- `sigma_rot_deg: Dict[u, (3,)]`
- `sigma_trans_mm: Dict[u, (3,)]`

### 7.3 [`stage2_runner.py`](calibration/stage2/stage2_runner.py) — Stage 2 오케스트레이터

#### 7.3.1 `_load_stage1(unit, results_root, session_id=None)`

- session_id 없으면 mtime 최대 = 가장 최근 세션 자동 채택
- `intrinsic.yaml` + `lidar_to_cam.yaml` 로드 → `K`, `D`, `T_lidar_to_cam`(rotation_matrix+translation 결합), `image_size`

#### 7.3.2 `run_stage2(units, session_id, ...)`

기본 경로:
- data: `/root/sam_ws/calibration_data/stage2`
- results: `/root/sam_ws/calibration_results/stage2`
- stage1_results_root: `/root/sam_ws/calibration_results/stage1`
- board: `/root/sam_ws/config/board_b_r3_params.yaml`
- config: `/root/sam_ws/config/stage2_config.yaml`
- unit_mapping: `/root/sam_ws/config/unit_mapping.yaml`

##### 단계 (진행률)

1. **(0–10%) Stage 1 결과 로드**: 각 유닛의 K, D, T_lc + session_id 보관
2. **(10–40%) 검출**: pose ≥6개, 유효 관측 pose ≥6개 강제
3. **(50%) 초기값**: `T_u_to_world`(world unit=I, 나머지는 SE(3) median), `T_lc_init`(Stage 1), `T_board_init`(world unit의 `T_cam_board`)
4. **(50–90%) Joint BA**
5. **(90–100%) 검증 + 저장**

#### 7.3.3 PASS 조건 (cfg `pass_criteria`)

| 키 | 비교 | 기본 임계 |
| --- | --- | --- |
| `reproj_rms_px` | `corner_rms < x` | 0.8 |
| `apex_3d_mm` | `apex3d_rms_mm < x×10` | 2.0 → 실효 20mm |
| `sigma_rot_deg` | `max σ_rot < x` | 0.01 |
| `sigma_trans_mm` | `max σ_trans < x` | 5.0 |

> ⚠️ apex_3d_mm 비교에서 `pass_cfg.inter_apex_mm × 10` 곱이 적용됨 — 코드 그대로 명시. 의도 의심 여지.

#### 7.3.4 출력

`unit_extrinsics.yaml`:
```yaml
session_id: ...
world_frame_unit: ...
units:
  Gantry_Global1:
    T_cam_to_world: { rotation_matrix: [[...]], translation_m: [...], rvec: [...] }
    T_lidar_to_cam: { rotation_matrix, translation_m }
    covariance: { sigma_rot_deg, sigma_trans_mm }
  ...
stage1_session_refs: { ...: { session_id, T_lidar_to_cam } }
strategy: C
residuals: { corner_reprojection_rms_px, apex_reprojection_rms_px, apex_3d_rms_mm }
ba_info: { cost, n_evals, status, message, n_variables, n_residuals }
pass_details: { ... }
pass: true|false
calibration_timestamp_utc: ...Z
```

`validation_report.md`: 조건별 PASS/FAIL + 핵심 수치 요약.

#### 7.3.5 CLI (`main_cli`)

```bash
calib_stage2 --units Gantry_Global1,Gantry_Global2,Gantry_Global3,Gantry_Global4 \
             --session-id 20250506_120000 \
             [--data-root ...] [--results-root ...] \
             [--stage1-results-root ...] [--board-yaml ...] \
             [--config ...] [--unit-mapping ...]
```

종료 코드: 0=PASS, 1=ERROR(traceback 출력), 2=FAIL.

---

## 8. tools/ — 보조 도구

### 8.1 [`_board_generator.py`](calibration/tools/_board_generator.py) — 인쇄용 ChArUco 이미지 생성

`generate_charuco_image(board_yaml, out_path, dpi=300, border_mm=None)`:
1. `BoardDef.load(board_yaml)`로 메타 로드
2. `outer_size_m × dpi/25.4` → 캔버스 픽셀 크기 계산
3. ChArUco 패턴 픽셀 영역 = `squares_x × square_length × dpmm`
4. OpenCV 다중 API 호환 (`generateImage` → `draw` fallback)
5. 흰 캔버스 중앙에 패턴 합성
6. **콘 발자국**: 각 콘의 `base_center_m` 위치에 `diameter` 원 + 십자선(연한 회색)
7. PNG 저장

> 콘 자체는 물리 부착물이라 인쇄물에는 위치 가이드만 표시.

### 8.2 [`generate_board_a_r2.py`](calibration/tools/generate_board_a_r2.py)

CLI 래퍼.
```
calib_generate_board_a [--yaml /root/sam_ws/config/board_a_r2_params.yaml]
                       [--out  /root/sam_ws/calibration_results/board_a_r2_print.png]
                       [--dpi  300]
```

### 8.3 [`generate_board_b_r3.py`](calibration/tools/generate_board_b_r3.py)

```
calib_generate_board_b [--yaml /root/sam_ws/config/board_b_r3_params.yaml]
                       [--out  /root/sam_ws/calibration_results/board_b_r3_print.png]
                       [--dpi  150]
```

> Board B는 더 큰 보드라 default DPI가 150으로 낮음.

### 8.4 [`verify_install.py`](calibration/tools/verify_install.py) — 설치/설정 점검

`calib_verify_install`:

1. **Dependency check**: numpy/scipy/yaml/cv2(필수) + flask/open3d/matplotlib(선택). 버전 출력.
2. **Config check**: 5개 경로 존재 확인
   - `/root/sam_ws/config/board_a_r2_params.yaml`
   - `/root/sam_ws/config/board_b_r3_params.yaml`
   - `/root/sam_ws/config/stage1_config.yaml`
   - `/root/sam_ws/config/stage2_config.yaml`
   - `/root/sam_ws/config/unit_mapping.yaml`
3. Board A/B는 실제 파싱 시도 → `charuco squares_x×y, n cones` 출력
4. 둘 다 통과시 exit 0, 아니면 1.

### 8.5 [`synth_smoke_test.py`](calibration/tools/synth_smoke_test.py) — 합성 스모크 테스트

빠른 sanity 검사 (CI / 첫 동작 확인용).

#### 8.5.1 `quick_charuco_detection_check()`

1. `_board_generator`로 Board A를 100 DPI로 렌더 → 임시 파일
2. 그 이미지를 다시 `detect_charuco`로 검출
3. `count >= min_corners_for_valid` 통과 여부 + count 반환

#### 8.5.2 `quick_cone_ransac_check()`

1. ground-truth apex `(0.5, 0.3, 0.04)`, axis +Z, half=36.87°
2. 콘 표면에 120점 샘플 + Gaussian σ=3mm 노이즈
3. `fit_cone_ransac` 적용
4. `‖fit.apex - apex_true‖ < 0.02m` 검사

`main()`: 두 검사 모두 PASS 시 0 종료, 실패 시 1.

### 8.6 [`synth_stage1_test.py`](calibration/tools/synth_stage1_test.py) — Stage 1 BA 종단 합성 검증

이미지/PCD 렌더링 단계 우회. **GT 외부 변환이 알려진 합성 관측을 직접 BA에 주입**해 수렴성을 검증.

#### 8.6.1 시나리오

- 카메라 내부: `f=900`, 1200×1200, D=0
- GT `T_lidar_to_cam`: rvec=(0.02,-0.015,0.01), tvec=(0.10,-0.02,0.04) [m]
- 8개 포즈 생성 (보드 거리 2.5–3.5m, 회전 ±0.3rad, 평행이동 ±0.15m)
- 각 포즈:
  - `corners_3d_board` (analytical 8×8 grid)
  - `corners_2d` = 투영 + N(0, 0.15px)
  - `apex_2d` = 투영 + N(0, 0.2px)
  - `apex_3d_lidar` = `T_board_to_lidar @ apex_board` + N(0, 2mm)
  - `T_cam_board` 노이즈: rvec N(0, 0.005rad), tvec N(0, 0.003m)
- 초기 `T_lc_init` = GT + N(0, 0.01)

#### 8.6.2 검증

BA 후:
- `rot_err` = arccos(0.5(tr(R_rec R_gt^T)-1)) [deg]
- `trans_err_mm`
- `corner_rms_px`, `apex3d_rms_mm`

PASS: `rot_err < 0.5°` & `trans_err_mm < 30mm`.

`_board_charuco_corners(board_def)` 헬퍼: 분석적 8×8 내부 코너 그리드 (z=0).

---

## 9. test/ — 단위 테스트

### 9.1 [`test_basic.py`](test/test_basic.py)

`pytest`로 ROS2/mainwindow 없이 실행 가능한 단위 테스트.

| 테스트 함수 | 검증 대상 |
| --- | --- |
| `test_board_a_load` | Board A YAML — name/squares_x/y=8/cones=4/apex z=0.040 |
| `test_board_b_load` | Board B YAML — name/squares=6×6/cones=4 |
| `test_transforms_rodrigues` | 20회 랜덤 rvec → R → rvec → R 일관성 (atol 1e-10) |
| `test_rigid_transform_3d` | Umeyama 정답 복원 (atol 1e-8) |
| `test_cone_ransac_synthetic` | 200점 합성 콘에서 apex 오차 < 1cm |
| `test_validation_thresholds` | `compare_criteria` 기본 동작 |
| `test_board_image_generation_tmp` | `generate_charuco_image` (DPI=50) — 파일 생성 + 1KB 이상 |

> 테스트는 Board YAML이 `/root/sam_ws/config/`에 실제 존재해야 통과.

---

## 10. 데이터 흐름 요약

### 10.1 Stage 1 (단일 유닛)

```
                ┌──────────────────────────────────────────────┐
                │  /calib/stage1/create  (web_bp)              │
                │  → CalibrationSession(stage1, unit, ...)     │
                └──────────────┬───────────────────────────────┘
                               ▼
   ┌─────────────────── 캡처 (10+ intrinsic, 5+ lidar_cam) ───────┐
   │  /calib/capture_pose                                          │
   │  ├── capture_stage1_intrinsic  → cam.png                       │
   │  └── capture_stage1_lidar_cam  → cam.png + lidar.pcd + meta    │
   └────────────────────────┬─────────────────────────────────────┘
                            ▼
   ┌────── /calib/stage1/run_ba (백그라운드) ──────────────┐
   │  Step 1a: calibrate_intrinsic → K, D                  │
   │  Step 1b: ChArUco PnP + LiDAR cone detect → T_lc_init │
   │  Step 1c: Joint BA (scipy TRF + Huber)                │
   └────────┬──────────────────────────────────────────────┘
            ▼
   intrinsic.yaml + lidar_to_cam.yaml + validation_report.md
```

### 10.2 Stage 2 (전체 유닛)

```
   ┌── /calib/stage2/create ── CalibrationSession(stage2, units, ...) ──┐
   ▼
   ┌── /calib/capture_pose (6+ poses) ─────────────────────────────────┐
   │  capture_stage2_pose:                                              │
   │    set pcl_target_override(3) → 모든 유닛 LiDAR 시작 → 이미지       │
   │    → LiDAR 완료 대기 → cam_<u>.png / lidar_<u>.pcd / meta.json     │
   └────────────────┬──────────────────────────────────────────────────┘
                    ▼
   ┌── /calib/stage2/run_ba ─────────────────────────────────────────┐
   │  Stage 1 결과 로드 (각 유닛 K, D, T_lc)                            │
   │  detect_all_units (ChArUco + cone in image + cone in lidar)        │
   │  step2b: T_u_to_world median 초기값                                 │
   │  Joint BA (월드 유닛 고정, T_lc는 prior로 부드럽게 갱신)              │
   │  공분산 근사 (J^T J pseudo-inverse)                                  │
   └─────────────┬──────────────────────────────────────────────────────┘
                 ▼
   unit_extrinsics.yaml + validation_report.md
```

---

## 11. 주요 수식 / 좌표 변환 약속

### 11.1 표기

- `T_a_b`: a → b 좌표 변환 (점을 a 프레임에서 b 프레임으로 옮김)
- `transform_points(T_a_b, p_a) = p_b`
- `cv2.solvePnP` 출력 `(rvec, tvec)`은 `T_board_to_cam`에 해당 (board 좌표 점 → cam 좌표 점). 본 코드는 이를 `T_cam_board`로 명명하지만 **transform_points의 인자로 board 좌표를 받아 cam 좌표를 반환**한다는 의미로 사용. (즉 코드 내부 표기는 `_board→cam_`).

### 11.2 Stage 1 핵심 식

```
T_lidar_to_board = inverse(T_cam_to_board(=T_board_to_cam)) @ T_lidar_to_cam
                 = T_cam_to_board⁻¹ @ T_lidar_to_cam

apex_in_board    = T_lidar_to_board · apex_lidar
잔차_3d          = apex_in_board - apex_3d_board (expected)
```

### 11.3 Stage 2 핵심 식

```
T_world_to_cam_u  = inverse(T_u_to_world)
T_board_to_cam_u  = T_world_to_cam_u @ T_board_to_world
                  = inverse(T_u_to_world) @ T_board_to_world

obj_pts_in_cam_u  = transform_points(T_board_to_cam_u, corners_3d_board)
                  → cv2.projectPoints으로 픽셀 → 코너 잔차

T_lidar_to_board  = inverse(T_board_to_cam_u) @ T_lc[u]
apex_in_board     = transform_points(T_lidar_to_board, apex_3d_lidar[u])
                  → apex 3D 잔차

T_lc prior 잔차    = se3_log(inverse(T_lc_prior) @ T_lc[u]) / σ
```

### 11.4 SE(3) median (강건 평균)

`se3_median(Ts)`:

```
T0 = T_first
for k in 1..10:
    δᵢ      = se3_log(T0⁻¹ @ Tᵢ)         # 6-vector tangent
    wᵢ      = 1 / max(‖δᵢ‖, 1e-6)         # L1 가중
    wᵢ      ← wᵢ / Σwᵢ                     # 정규화
    mean_δ  = Σ wᵢ δᵢ
    if ‖mean_δ‖ < 1e-8: break
    T0     ← T0 @ exp(mean_δ)
return T0
```

> 측지 평균을 L1 IRLS 근사로 10회 갱신 — outlier 강건성.

---

## 부록 A: PASS 임계값 정리

| 단계 | 항목 | cfg 키 | 기본값 |
| --- | --- | --- | --- |
| Stage 1 | 내부 RMS | `intrinsic.pass_rms_px` | 0.15 px |
| Stage 1 | 내부 max-per-corner | `intrinsic.pass_per_corner_px` | 0.5 px |
| Stage 1 | apex 3D RMS | `lidar_cam.pass_apex_3d_mm` | 1.5 mm |
| Stage 2 | corner RMS | `pass_criteria.reproj_rms_px` | 0.8 px |
| Stage 2 | apex 3D RMS | `pass_criteria.inter_apex_mm` × 10 | 20 mm (=2.0×10) |
| Stage 2 | σ_rot 최대 | `pass_criteria.sigma_rot_deg` | 0.01° |
| Stage 2 | σ_trans 최대 | `pass_criteria.sigma_trans_mm` | 5 mm |

## 부록 B: 외부 설정 파일 (코드에서 참조)

본 패키지 외부 (`/root/sam_ws/config/`)에 위치해야 하는 파일들. 본 코드만으로는 실행 불가 — 별도 제공 필요.

| 파일 | 사용처 |
| --- | --- |
| `board_a_r2_params.yaml` | Stage 1 보드 형상 |
| `board_b_r3_params.yaml` | Stage 2 보드 형상 |
| `stage1_config.yaml` | Stage 1 가중치/임계값/솔버 |
| `stage2_config.yaml` | Stage 2 가중치/임계값/prior |
| `unit_mapping.yaml` | `world_frame_unit` 등 |

## 부록 C: 산출물 디렉토리 (기본 경로)

```
/root/sam_ws/calibration_data/
  ├── stage1/<unit>/<session>/
  │     ├── intrinsic/img_pose001.png ...
  │     └── lidar_cam/pose001/{cam.png, lidar.pcd, meta.json} ...
  └── stage2/<session>/pose001/{cam_<u>.png, lidar_<u>.pcd, meta.json} ...

/root/sam_ws/calibration_results/
  ├── stage1/<unit>/<session>/{intrinsic.yaml, lidar_to_cam.yaml, validation_report.md}
  ├── stage2/<session>/{unit_extrinsics.yaml, validation_report.md}
  ├── board_a_r2_print.png
  └── board_b_r3_print.png
```

---

## 부록 D: 콘솔 스크립트 빠른 참조

```bash
# 의존성/설정 점검
calib_verify_install

# 보드 인쇄 이미지 생성
calib_generate_board_a --dpi 300
calib_generate_board_b --dpi 150

# Stage 1 (단일 유닛)
calib_stage1 --unit Gantry_Global1 --session-id 20250506_120000

# Stage 2 (다중 유닛)
calib_stage2 --units Gantry_Global1,Gantry_Global2,Gantry_Global3,Gantry_Global4 \
             --session-id 20250506_130000

# Synthetic 검증
python -m calibration.tools.synth_smoke_test
python -m calibration.tools.synth_stage1_test

# pytest
pytest calibration/test/test_basic.py
```

---

> **문서 생성**: 2026-05-06 / `calibration` v0.1.0
> 본 문서는 `calibration/**/*.py` 전체 분석에 기반하여 작성됨.
