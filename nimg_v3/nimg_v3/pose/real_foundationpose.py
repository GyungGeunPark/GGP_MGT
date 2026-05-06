"""
real_foundationpose.py - 실제 NVIDIA FoundationPose 연동 모듈

NVIDIA FoundationPose 공식 구현을 nimg_v3에 연동합니다.
이 모듈은 Model-Free 방식으로 6DoF 자세 추정을 수행합니다.

Requirements:
- FoundationPose weights in ../../../FoundationPose/weights/
- CUDA GPU
- nvdiffrast (built via build_all.sh)

Version: 1.0
Author: FurSys AI Team
"""

import os
import sys
import numpy as np
from pathlib import Path
from typing import Optional, Dict, Any, Tuple
import logging
import time

logger = logging.getLogger(__name__)

# FoundationPose 경로 추가
FOUNDATION_POSE_DIR = Path(__file__).parents[4] / "FoundationPose"
if FOUNDATION_POSE_DIR.exists():
    sys.path.insert(0, str(FOUNDATION_POSE_DIR))


class RealFoundationPoseEstimator:
    """
    실제 NVIDIA FoundationPose를 사용하는 6DoF 자세 추정기

    Model-Free 방식으로 RGBD 이미지와 마스크를 사용하여
    객체의 6DoF 자세를 추정합니다.
    """

    def __init__(
        self,
        weights_dir: Optional[str] = None,
        mesh_path: Optional[str] = None,
        device: str = 'cuda:0',
        debug: int = 0
    ):
        """
        Args:
            weights_dir: FoundationPose 가중치 디렉토리
            mesh_path: 3D 메시 파일 경로 (.obj)
            device: CUDA 디바이스
            debug: 디버그 레벨 (0-2)
        """
        self.device = device
        self.debug = debug
        self.mesh_path = mesh_path

        # 가중치 경로 설정
        if weights_dir is None:
            weights_dir = str(FOUNDATION_POSE_DIR / "weights" / "no_diffusion")
        self.weights_dir = Path(weights_dir)

        # 상태
        self._initialized = False
        self._estimator = None
        self._scorer = None
        self._refiner = None
        self._mesh = None
        self._glctx = None
        self.pose_last = None

        # 초기화 시도
        self._try_initialize()

    def _try_initialize(self):
        """FoundationPose 초기화 시도"""
        try:
            self._initialize_foundationpose()
            self._initialized = True
            logger.info("RealFoundationPoseEstimator initialized successfully")
        except ImportError as e:
            logger.warning(f"FoundationPose import failed: {e}")
            logger.warning("Falling back to depth-based estimation")
            self._initialized = False
        except Exception as e:
            logger.error(f"FoundationPose initialization failed: {e}")
            self._initialized = False

    def _initialize_foundationpose(self):
        """FoundationPose 컴포넌트 초기화"""
        # FoundationPose 모듈 임포트
        from estimater import FoundationPose
        from learning.training.predict_score import ScorePredictor
        from learning.training.predict_pose_refine import PoseRefinePredictor
        import trimesh

        # 가중치 경로
        refiner_path = self.weights_dir / "2023-10-28-18-33-37"
        scorer_path = self.weights_dir / "2024-01-11-20-02-45"

        if not refiner_path.exists() or not scorer_path.exists():
            raise FileNotFoundError(f"Weights not found in {self.weights_dir}")

        # Scorer 초기화
        logger.info("Loading ScorePredictor...")
        self._scorer = ScorePredictor()
        self._scorer.load(str(scorer_path))

        # Refiner 초기화
        logger.info("Loading PoseRefinePredictor...")
        self._refiner = PoseRefinePredictor()
        self._refiner.load(str(refiner_path))

        # 메시 로드
        if self.mesh_path and Path(self.mesh_path).exists():
            logger.info(f"Loading mesh from {self.mesh_path}")
            self._mesh = trimesh.load(self.mesh_path)
        else:
            # 기본 박스 메시 생성
            logger.info("Creating default box mesh")
            self._mesh = trimesh.creation.box(extents=[0.1, 0.1, 0.1])

        # FoundationPose 초기화
        model_pts = self._mesh.vertices
        model_normals = self._mesh.vertex_normals

        self._estimator = FoundationPose(
            model_pts=model_pts,
            model_normals=model_normals,
            mesh=self._mesh,
            scorer=self._scorer,
            refiner=self._refiner,
            debug=self.debug
        )

        logger.info("FoundationPose initialized")

    def estimate(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        mask: np.ndarray,
        K: np.ndarray
    ) -> Tuple[np.ndarray, float]:
        """
        자세 추정 (초기화)

        Args:
            rgb: RGB 이미지 [H, W, 3] uint8
            depth: Depth 이미지 [H, W] float32 (미터)
            mask: 객체 마스크 [H, W] uint8
            K: 카메라 내부 행렬 [3, 3]

        Returns:
            pose: 4x4 변환 행렬
            confidence: 신뢰도 점수
        """
        if not self._initialized:
            return self._fallback_estimate(rgb, depth, mask, K)

        try:
            start_time = time.time()

            # FoundationPose register 호출
            pose = self._estimator.register(
                K=K,
                rgb=rgb,
                depth=depth,
                ob_mask=mask.astype(np.float32) / 255.0
            )

            self.pose_last = pose

            # 신뢰도 계산 (scores 기반)
            if hasattr(self._estimator, 'scores') and len(self._estimator.scores) > 0:
                confidence = float(self._estimator.scores[0])
            else:
                confidence = 0.8

            elapsed = time.time() - start_time
            logger.debug(f"Pose estimation: {elapsed*1000:.1f}ms, conf={confidence:.3f}")

            return pose, confidence

        except Exception as e:
            logger.error(f"FoundationPose estimation failed: {e}")
            return self._fallback_estimate(rgb, depth, mask, K)

    def track(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        K: np.ndarray,
        iteration: int = 2
    ) -> Tuple[np.ndarray, float]:
        """
        자세 추적

        Args:
            rgb: RGB 이미지
            depth: Depth 이미지
            K: 카메라 내부 행렬
            iteration: 정제 반복 횟수

        Returns:
            pose: 4x4 변환 행렬
            confidence: 신뢰도 점수
        """
        if not self._initialized or self.pose_last is None:
            logger.warning("Cannot track without initialization")
            return np.eye(4), 0.0

        try:
            start_time = time.time()

            # FoundationPose track_one 호출
            pose = self._estimator.track_one(
                rgb=rgb,
                depth=depth,
                K=K,
                iteration=iteration
            )

            self.pose_last = pose
            confidence = 0.9  # 추적 시 고정 신뢰도

            elapsed = time.time() - start_time
            logger.debug(f"Pose tracking: {elapsed*1000:.1f}ms")

            return pose, confidence

        except Exception as e:
            logger.error(f"FoundationPose tracking failed: {e}")
            return self.pose_last if self.pose_last is not None else np.eye(4), 0.3

    def _fallback_estimate(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        mask: np.ndarray,
        K: np.ndarray
    ) -> Tuple[np.ndarray, float]:
        """
        Fallback: Depth 기반 간단한 자세 추정

        FoundationPose가 사용 불가능할 때 사용
        """
        mask_bool = mask > 0

        # 유효한 depth 추출
        valid_depth = depth[mask_bool]
        valid_depth = valid_depth[(valid_depth > 0.1) & (valid_depth < 5.0)]

        if len(valid_depth) == 0:
            return np.eye(4), 0.0

        # Z값 (깊이)
        z = np.median(valid_depth)

        # 마스크 중심
        ys, xs = np.where(mask_bool)
        if len(xs) == 0:
            return np.eye(4), 0.0

        cx_mask = np.mean(xs)
        cy_mask = np.mean(ys)

        # 3D 위치 계산
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        x = (cx_mask - cx) * z / fx
        y = (cy_mask - cy) * z / fy

        # 4x4 자세 행렬
        pose = np.eye(4)
        pose[:3, 3] = [x, y, z]

        # 마스크 형상 기반 회전 추정
        if len(xs) > 10:
            try:
                points = np.column_stack([xs - cx_mask, ys - cy_mask])
                cov = np.cov(points.T)
                eigenvalues, eigenvectors = np.linalg.eigh(cov)
                angle = np.arctan2(eigenvectors[1, 1], eigenvectors[0, 1])

                cos_a, sin_a = np.cos(angle), np.sin(angle)
                pose[:2, :2] = [[cos_a, -sin_a], [sin_a, cos_a]]
            except:
                pass

        confidence = min(len(valid_depth) / 2000.0, 0.7)
        self.pose_last = pose

        return pose, confidence

    def reset(self):
        """추적 상태 리셋"""
        self.pose_last = None
        if self._estimator is not None:
            self._estimator.pose_last = None

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    def get_info(self) -> Dict[str, Any]:
        """추정기 정보 반환"""
        return {
            'initialized': self._initialized,
            'weights_dir': str(self.weights_dir),
            'mesh_path': self.mesh_path,
            'device': self.device,
            'has_pose': self.pose_last is not None
        }


def create_intrinsic_matrix(
    fx: float, fy: float, cx: float, cy: float
) -> np.ndarray:
    """카메라 내부 행렬 생성"""
    K = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float32)
    return K
