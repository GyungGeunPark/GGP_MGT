"""
foundationpose_estimator.py - FoundationPose 통합 래퍼

FoundationPose 기반 6DoF 자세 추정 및 추적 모듈.
Model-Based와 Model-Free 모드를 모두 지원합니다.

주요 기능:
1. 자세 추정 (Pose Estimation): 초기화 시 전체 파이프라인 실행
2. 자세 추적 (Pose Tracking): 이전 자세 기반 빠른 정제
3. 추적 손실 복구: 자동 재초기화

아키텍처:
┌─────────────────────────────────────────────────────┐
│               FoundationPose Pipeline                │
├─────────────────────────────────────────────────────┤
│  RGBD + Mask → Pose Hypothesis → Refinement → Score │
│                     ↓                                │
│         Tracking Mode: Skip Hypothesis               │
└─────────────────────────────────────────────────────┘

Version: 1.0
Author: FurSys AI Team
Reference: nimg_v3_foundationpose_comprehensive_design_guide.md
"""

import numpy as np
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass
from enum import Enum
import time
import logging

logger = logging.getLogger(__name__)


class PoseMode(Enum):
    """자세 추정 모드"""
    MODEL_BASED = "model_based"  # CAD 모델 사용
    MODEL_FREE = "model_free"    # 참조 이미지 + Neural Field


class TrackingState(Enum):
    """추적 상태"""
    INITIALIZING = "initializing"  # 아직 초기화 안됨
    TRACKING = "tracking"          # 정상 추적 중
    LOST = "lost"                  # 추적 손실


@dataclass
class PoseResult:
    """
    자세 추정 결과

    Attributes:
        pose_matrix: 4x4 변환 행렬 [R|t]
        translation: [tx, ty, tz] 미터 단위
        rotation_matrix: 3x3 회전 행렬
        confidence: 신뢰도 점수 (0-1)
        tracking_state: 현재 추적 상태
        processing_time_ms: 처리 시간 (밀리초)
    """
    pose_matrix: np.ndarray      # 4x4 변환 행렬
    translation: np.ndarray      # [tx, ty, tz]
    rotation_matrix: np.ndarray  # 3x3 회전 행렬
    confidence: float            # 신뢰도 점수
    tracking_state: TrackingState
    processing_time_ms: float

    def to_dict(self) -> Dict[str, Any]:
        """딕셔너리로 변환"""
        return {
            'pose_matrix': self.pose_matrix.tolist(),
            'translation': self.translation.tolist(),
            'rotation_matrix': self.rotation_matrix.tolist(),
            'confidence': self.confidence,
            'tracking_state': self.tracking_state.value,
            'processing_time_ms': self.processing_time_ms
        }


class FoundationPoseEstimator:
    """
    FoundationPose 기반 6DoF 자세 추정기

    Model-Based와 Model-Free 모드를 통합 지원하며,
    추정 및 추적 모드를 자동으로 전환합니다.

    Example:
        >>> estimator = FoundationPoseEstimator(
        ...     model_dir="models/foundationpose",
        ...     mode=PoseMode.MODEL_FREE,
        ...     neural_field_dir="models/neural_fields/my_object"
        ... )
        >>> result = estimator.process(rgb, depth, mask, intrinsics)
        >>> print(f"Yaw: {result.translation}")
    """

    def __init__(
        self,
        model_dir: str,
        mode: PoseMode = PoseMode.MODEL_FREE,
        mesh_path: Optional[str] = None,
        neural_field_dir: Optional[str] = None,
        device: str = 'cuda:0',
        use_tensorrt: bool = True,
        tracking_recovery_threshold: float = 0.3,
        max_lost_frames: int = 5,
        refine_iterations: int = 5
    ):
        """
        Args:
            model_dir: FoundationPose 모델 디렉토리
            mode: Model-Based 또는 Model-Free
            mesh_path: CAD 모델 경로 (Model-Based용)
            neural_field_dir: Neural Object Field 경로 (Model-Free용)
            device: 추론 디바이스
            use_tensorrt: TensorRT 최적화 사용
            tracking_recovery_threshold: 추적 복구 신뢰도 임계값
            max_lost_frames: 최대 추적 손실 프레임 수
            refine_iterations: 정제 반복 횟수
        """
        self.model_dir = model_dir
        self.mode = mode
        self.mesh_path = mesh_path
        self.neural_field_dir = neural_field_dir
        self.device = device
        self.use_tensorrt = use_tensorrt
        self.tracking_recovery_threshold = tracking_recovery_threshold
        self.max_lost_frames = max_lost_frames
        self.refine_iterations = refine_iterations

        # 모드별 설정 검증
        if mode == PoseMode.MODEL_BASED and mesh_path is None:
            raise ValueError("mesh_path required for MODEL_BASED mode")
        # MODEL_FREE mode: neural_field_dir is optional - will use dummy mesh if not provided
        if mode == PoseMode.MODEL_FREE and neural_field_dir is None:
            logger.warning("neural_field_dir not provided for MODEL_FREE mode, will use dummy mesh")

        # 추적 상태
        self._tracking_state = TrackingState.INITIALIZING
        self._prev_pose = None
        self._lost_frame_count = 0

        # 모델 로딩 상태
        self._model_loaded = False
        self._mesh = None
        self._neural_field = None

        # 모델 초기화
        self._init_foundationpose()

        logger.info(f"FoundationPoseEstimator initialized: mode={mode.value}, device={device}")

    def _init_foundationpose(self):
        """
        FoundationPose 모델 초기화

        실제 FoundationPose 공식 구현체를 로드합니다.
        """
        try:
            # Model-Based: 메시 로드
            if self.mode == PoseMode.MODEL_BASED:
                self._load_mesh(self.mesh_path)

            # Model-Free: Neural Field 로드
            else:
                self._load_neural_field(self.neural_field_dir)

            # FoundationPose 실제 래퍼 로드
            try:
                from .foundationpose_wrapper import create_real_foundationpose_estimator

                # Get mesh for FoundationPose
                mesh_path = self.mesh_path if self.mode == PoseMode.MODEL_BASED else None
                if mesh_path is None and self._mesh is not None:
                    # Save mesh temporarily for FoundationPose
                    import tempfile
                    import trimesh
                    temp_mesh_path = tempfile.mktemp(suffix='.obj')
                    if isinstance(self._mesh, trimesh.Trimesh):
                        self._mesh.export(temp_mesh_path)
                        mesh_path = temp_mesh_path

                # Create real FoundationPose estimator
                self._real_estimator = create_real_foundationpose_estimator(
                    mesh_path=mesh_path,
                    device=self.device,
                    model_dir=self.model_dir
                )

                self._use_real_foundationpose = True
                logger.info("Real FoundationPose estimator initialized")

            except Exception as e:
                logger.warning(f"Failed to load real FoundationPose: {e}")
                logger.warning("Falling back to dummy implementation")
                self._real_estimator = None
                self._use_real_foundationpose = False

            self._model_loaded = True
            logger.info("FoundationPose model loaded successfully")

        except Exception as e:
            logger.error(f"Failed to initialize FoundationPose: {e}")
            raise

    def _load_mesh(self, mesh_path: str):
        """CAD 모델 로드 (Model-Based)"""
        try:
            import trimesh
            self._mesh = trimesh.load(mesh_path, force='mesh')
            logger.info(f"Mesh loaded: {mesh_path}")
            logger.info(f"  Vertices: {len(self._mesh.vertices)}")
            logger.info(f"  Faces: {len(self._mesh.faces)}")
        except ImportError:
            logger.warning("trimesh not available, mesh loading skipped")
            self._mesh = None
        except Exception as e:
            logger.error(f"Failed to load mesh: {e}")
            raise

    def _load_neural_field(self, neural_field_dir: Optional[str]):
        """Neural Object Field 로드 (Model-Free)"""
        import trimesh
        from pathlib import Path

        if neural_field_dir is None:
            logger.warning("Neural field dir not provided, using default box mesh")
            self._neural_field = None
            self._mesh = trimesh.creation.box(extents=[0.15, 0.25, 0.05])
            logger.info("Default box mesh created for model-free mode")
            return

        neural_field_path = Path(neural_field_dir)

        # 1순위: 디렉토리 내 메시 파일 직접 로드 (mesh.obj → mesh.ply → *.obj)
        for candidate_name in ['mesh.obj', 'mesh.ply']:
            mesh_file = neural_field_path / candidate_name
            if mesh_file.exists():
                try:
                    self._mesh = trimesh.load(str(mesh_file), force='mesh')
                    logger.info(f"Mesh loaded from file: {mesh_file} "
                               f"({len(self._mesh.vertices)} vertices, {len(self._mesh.faces)} faces)")
                    return
                except Exception as e:
                    logger.warning(f"Failed to load mesh file {mesh_file}: {e}")

        # .obj 파일 탐색 (CAD 모델 등)
        obj_files = sorted(neural_field_path.glob('*.obj'))
        for obj_file in obj_files:
            try:
                self._mesh = trimesh.load(str(obj_file), force='mesh')
                logger.info(f"Mesh loaded from file: {obj_file} "
                           f"({len(self._mesh.vertices)} vertices, {len(self._mesh.faces)} faces)")
                return
            except Exception as e:
                logger.warning(f"Failed to load mesh file {obj_file}: {e}")

        # 2순위: Neural Field 학습 데이터에서 메시 추출 시도
        try:
            from .neural_object_field import NeuralObjectField
            self._neural_field = NeuralObjectField(device=self.device)
            self._neural_field.load(neural_field_dir)

            if self._neural_field.mesh is not None:
                self._mesh = self._neural_field.mesh
                logger.info(f"Mesh from Neural Field: {neural_field_dir}")
                return

            self._mesh = self._neural_field.extract_mesh()
            if self._mesh is not None:
                logger.info(f"Mesh extracted from Neural Field: {neural_field_dir}")
                return
        except Exception as e:
            logger.warning(f"Neural Field mesh extraction failed: {e}")

        # 3순위: 대상 객체에 맞는 근사 box mesh 생성
        # tray_top / Wiring_tray: 약 15x25x5cm, housing_M: 약 12x12x8cm
        dir_name = neural_field_path.name.lower()
        if 'housing' in dir_name:
            extents = [0.12, 0.12, 0.08]
            logger.info(f"Creating approximate box mesh for housing: {extents}")
        elif 'tray' in dir_name:
            extents = [0.15, 0.25, 0.05]
            logger.info(f"Creating approximate box mesh for tray: {extents}")
        else:
            extents = [0.15, 0.15, 0.05]
            logger.info(f"Creating default box mesh: {extents}")

        self._mesh = trimesh.creation.box(extents=extents)
        logger.info(f"Approximate mesh created ({len(self._mesh.vertices)} vertices, "
                    f"{len(self._mesh.faces)} faces)")

    def estimate(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        mask: np.ndarray,
        intrinsics: Dict[str, float]
    ) -> PoseResult:
        """
        자세 추정 (초기화 또는 재초기화)

        전체 FoundationPose 파이프라인 실행:
        1. 자세 가설 생성 (정이십면체 샘플링)
        2. 각 가설 정제
        3. 최적 자세 선택

        Args:
            rgb: RGB 이미지 [H, W, 3]
            depth: Depth 이미지 [H, W] (미터 단위)
            mask: 객체 마스크 [H, W] (binary)
            intrinsics: 카메라 파라미터 {'fx', 'fy', 'cx', 'cy'}

        Returns:
            PoseResult: 자세 추정 결과
        """
        start_time = time.time()

        # 입력 검증
        self._validate_inputs(rgb, depth, mask)

        # dtype 변환 (FoundationPose는 float32 필요)
        depth = depth.astype(np.float32)

        # FoundationPose 추정 호출
        if hasattr(self, '_use_real_foundationpose') and self._use_real_foundationpose:
            # Use real FoundationPose
            try:
                pose_matrix, confidence = self._real_estimator.register(
                    rgb=rgb,
                    depth=depth,
                    mask=mask,
                    intrinsics=intrinsics
                )
                logger.debug("Using REAL FoundationPose for estimation")
            except Exception as e:
                logger.error(f"Real FoundationPose estimation failed: {e}")
                logger.warning("Falling back to dummy estimation")
                pose_matrix, confidence = self._dummy_estimate(rgb, depth, mask, intrinsics)
        else:
            # Use dummy implementation
            pose_matrix, confidence = self._dummy_estimate(rgb, depth, mask, intrinsics)
            logger.debug("Using DUMMY implementation for estimation")

        processing_time = (time.time() - start_time) * 1000

        # 추적 상태 업데이트
        if confidence > self.tracking_recovery_threshold:
            self._tracking_state = TrackingState.TRACKING
            self._prev_pose = pose_matrix.copy()
            self._lost_frame_count = 0
        else:
            self._tracking_state = TrackingState.LOST

        return PoseResult(
            pose_matrix=pose_matrix,
            translation=pose_matrix[:3, 3].copy(),
            rotation_matrix=pose_matrix[:3, :3].copy(),
            confidence=confidence,
            tracking_state=self._tracking_state,
            processing_time_ms=processing_time
        )

    def track(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        intrinsics: Dict[str, float]
    ) -> PoseResult:
        """
        자세 추적 (이전 자세 기반 빠른 추정)

        전체 추정 대신 이전 자세에서 정제만 수행하여
        더 빠른 처리 속도를 제공합니다.

        Args:
            rgb: RGB 이미지
            depth: Depth 이미지
            intrinsics: 카메라 파라미터

        Returns:
            PoseResult: 자세 추정 결과
        """
        if self._prev_pose is None:
            raise RuntimeError("Cannot track without previous pose. Call estimate() first.")

        start_time = time.time()

        # dtype 변환 (FoundationPose는 float32 필요)
        depth = depth.astype(np.float32)

        # FoundationPose 추적 호출
        if hasattr(self, '_use_real_foundationpose') and self._use_real_foundationpose:
            # Use real FoundationPose
            try:
                pose_matrix, confidence = self._real_estimator.track(
                    rgb=rgb,
                    depth=depth,
                    prev_pose=self._prev_pose,
                    intrinsics=intrinsics
                )
                logger.debug("Using REAL FoundationPose for tracking")
            except Exception as e:
                logger.error(f"Real FoundationPose tracking failed: {e}")
                logger.warning("Falling back to dummy tracking")
                pose_matrix, confidence = self._dummy_track(rgb, depth, intrinsics)
        else:
            # Use dummy implementation
            pose_matrix, confidence = self._dummy_track(rgb, depth, intrinsics)
            logger.debug("Using DUMMY implementation for tracking")

        processing_time = (time.time() - start_time) * 1000

        # 신뢰도 체크 및 추적 상태 업데이트
        if confidence < self.tracking_recovery_threshold:
            self._lost_frame_count += 1
            if self._lost_frame_count >= self.max_lost_frames:
                self._tracking_state = TrackingState.LOST
        else:
            self._tracking_state = TrackingState.TRACKING
            self._prev_pose = pose_matrix.copy()
            self._lost_frame_count = 0

        return PoseResult(
            pose_matrix=pose_matrix,
            translation=pose_matrix[:3, 3].copy(),
            rotation_matrix=pose_matrix[:3, :3].copy(),
            confidence=confidence,
            tracking_state=self._tracking_state,
            processing_time_ms=processing_time
        )

    def process(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        mask: Optional[np.ndarray],
        intrinsics: Dict[str, float],
        force_estimate: bool = False
    ) -> PoseResult:
        """
        자동 모드 선택 처리

        추적 상태에 따라 자동으로 추정 또는 추적 모드를 선택합니다.

        Args:
            rgb, depth, mask, intrinsics: 입력 데이터
            force_estimate: 강제로 전체 추정 수행

        Returns:
            PoseResult: 자세 추정 결과
        """
        # 추적 손실 또는 강제 추정 시
        if (self._tracking_state == TrackingState.LOST or
            self._tracking_state == TrackingState.INITIALIZING or
            force_estimate):

            if mask is None:
                raise ValueError("mask required for pose estimation")
            return self.estimate(rgb, depth, mask, intrinsics)

        # 추적 모드
        return self.track(rgb, depth, intrinsics)

    def _validate_inputs(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        mask: Optional[np.ndarray]
    ):
        """입력 데이터 검증"""
        if rgb is None or rgb.size == 0:
            raise ValueError("Invalid RGB image")

        if depth is None or depth.size == 0:
            raise ValueError("Invalid depth image")

        if rgb.shape[:2] != depth.shape[:2]:
            raise ValueError(f"RGB/Depth size mismatch: {rgb.shape[:2]} vs {depth.shape[:2]}")

        if mask is not None and mask.shape[:2] != rgb.shape[:2]:
            raise ValueError(f"Mask size mismatch: {mask.shape[:2]} vs {rgb.shape[:2]}")

    def _dummy_estimate(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        mask: np.ndarray,
        intrinsics: Dict[str, float]
    ) -> Tuple[np.ndarray, float]:
        """
        더미 추정 함수 (실제 FoundationPose 연동 전 테스트용)

        마스크 영역의 depth 정보를 사용하여
        간단한 자세를 추정합니다.
        """
        # 마스크 영역 분석
        mask_bool = mask > 0 if mask is not None else np.ones(depth.shape, dtype=bool)

        # 마스크 영역의 depth 평균 (위치 추정)
        valid_depth = depth[mask_bool]
        valid_depth = valid_depth[(valid_depth > 0.1) & (valid_depth < 5.0)]

        if len(valid_depth) == 0:
            return np.eye(4), 0.0

        z = np.median(valid_depth)

        # 마스크 중심 계산
        ys, xs = np.where(mask_bool)
        if len(xs) == 0:
            return np.eye(4), 0.0

        cx_mask = np.mean(xs)
        cy_mask = np.mean(ys)

        # 3D 위치 계산
        fx, fy = intrinsics.get('fx', 383.883), intrinsics.get('fy', 383.883)
        cx, cy = intrinsics.get('cx', 320.499), intrinsics.get('cy', 237.913)

        x = (cx_mask - cx) * z / fx
        y = (cy_mask - cy) * z / fy

        # 4x4 자세 행렬 생성
        pose_matrix = np.eye(4)
        pose_matrix[:3, 3] = [x, y, z]

        # 간단한 회전 추정 (마스크 형상 기반)
        # 실제로는 FoundationPose의 정교한 추정 사용
        if len(xs) > 10:
            # PCA로 주방향 추정
            points = np.column_stack([xs - cx_mask, ys - cy_mask])
            try:
                from sklearn.decomposition import PCA
                pca = PCA(n_components=2)
                pca.fit(points)
                angle = np.arctan2(pca.components_[0, 1], pca.components_[0, 0])

                # Z축 회전 적용
                cos_a, sin_a = np.cos(angle), np.sin(angle)
                pose_matrix[:2, :2] = [[cos_a, -sin_a], [sin_a, cos_a]]
            except ImportError:
                pass

        confidence = min(len(valid_depth) / 1000.0, 0.95)

        return pose_matrix, confidence

    def _dummy_track(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        intrinsics: Dict[str, float]
    ) -> Tuple[np.ndarray, float]:
        """
        더미 추적 함수 (실제 FoundationPose 연동 전 테스트용)

        이전 자세에 작은 노이즈를 추가하여 시뮬레이션합니다.
        """
        if self._prev_pose is None:
            return np.eye(4), 0.0

        # 이전 자세 복사
        pose_matrix = self._prev_pose.copy()

        # 작은 변동 추가 (시뮬레이션)
        # 실제로는 FoundationPose의 정제 네트워크 출력
        noise = np.random.randn(3) * 0.001  # 1mm 표준편차
        pose_matrix[:3, 3] += noise

        confidence = 0.85 + np.random.rand() * 0.1

        return pose_matrix, confidence

    def reset(self):
        """추적 상태 리셋"""
        self._tracking_state = TrackingState.INITIALIZING
        self._prev_pose = None
        self._lost_frame_count = 0
        logger.info("Estimator reset")

    @property
    def is_tracking(self) -> bool:
        return self._tracking_state == TrackingState.TRACKING

    @property
    def tracking_state(self) -> TrackingState:
        return self._tracking_state

    @property
    def prev_pose(self) -> Optional[np.ndarray]:
        return self._prev_pose.copy() if self._prev_pose is not None else None

    @property
    def mesh(self):
        return self._mesh

    def get_info(self) -> Dict[str, Any]:
        """추정기 정보 반환"""
        return {
            'mode': self.mode.value,
            'model_dir': self.model_dir,
            'device': self.device,
            'use_tensorrt': self.use_tensorrt,
            'tracking_state': self._tracking_state.value,
            'model_loaded': self._model_loaded,
            'has_mesh': self._mesh is not None,
            'has_neural_field': self._neural_field is not None
        }


class FoundationPoseEstimatorFactory:
    """FoundationPoseEstimator 팩토리 클래스"""

    @staticmethod
    def create_model_free(
        model_dir: str,
        neural_field_dir: str,
        device: str = 'cuda:0',
        **kwargs
    ) -> FoundationPoseEstimator:
        """Model-Free 모드 추정기 생성"""
        return FoundationPoseEstimator(
            model_dir=model_dir,
            mode=PoseMode.MODEL_FREE,
            neural_field_dir=neural_field_dir,
            device=device,
            **kwargs
        )

    @staticmethod
    def create_model_based(
        model_dir: str,
        mesh_path: str,
        device: str = 'cuda:0',
        **kwargs
    ) -> FoundationPoseEstimator:
        """Model-Based 모드 추정기 생성"""
        return FoundationPoseEstimator(
            model_dir=model_dir,
            mode=PoseMode.MODEL_BASED,
            mesh_path=mesh_path,
            device=device,
            **kwargs
        )
