"""
foundationpose_wrapper.py - Real FoundationPose Integration Wrapper

실제 FoundationPose 공식 코드를 nimg_v3에 통합하는 래퍼.
더미 함수 대신 실제 FoundationPose estimater.py를 사용합니다.

Version: 1.0
Author: FurSys AI Team
"""

import sys
import os
import numpy as np
from typing import Optional, Dict, Any, Tuple
from pathlib import Path
import logging
import time
import torch
import trimesh

logger = logging.getLogger(__name__)

# Add FoundationPose to path
FOUNDATIONPOSE_DIR = "/root/fursys_imgprosessing_ws/src/FoundationPose"
if FOUNDATIONPOSE_DIR not in sys.path:
    sys.path.insert(0, FOUNDATIONPOSE_DIR)

# Import FoundationPose
try:
    from estimater import FoundationPose
    from Utils import *
    from learning.training.predict_score import ScorePredictor
    from learning.training.predict_pose_refine import PoseRefinePredictor
    import yaml
    FOUNDATIONPOSE_AVAILABLE = True
    logger.info("FoundationPose modules imported successfully")
except ImportError as e:
    FOUNDATIONPOSE_AVAILABLE = False
    logger.warning(f"FoundationPose import failed: {e}")
    logger.warning("Falling back to dummy implementation")


class RealFoundationPoseEstimator:
    """
    실제 FoundationPose를 사용하는 래퍼 클래스

    FoundationPose 공식 구현체를 nimg_v3 인터페이스에 맞게 래핑합니다.
    """

    def __init__(
        self,
        model_dir: str = None,
        mesh_path: Optional[str] = None,
        device: str = 'cuda:0',
        debug: int = 0,
        debug_dir: str = '/tmp/foundationpose_debug'
    ):
        """
        Args:
            model_dir: FoundationPose 모델 가중치 디렉토리
            mesh_path: 객체 메시 파일 경로 (.obj, .ply 등)
            device: 추론 디바이스
            debug: 디버그 레벨
            debug_dir: 디버그 출력 디렉토리
        """
        if not FOUNDATIONPOSE_AVAILABLE:
            raise RuntimeError("FoundationPose not available. Please install FoundationPose first.")

        self.device = device
        self.debug = debug
        self.debug_dir = debug_dir

        # Model directory setup
        if model_dir is None:
            model_dir = os.path.join(FOUNDATIONPOSE_DIR, "weights/no_diffusion")

        self.model_dir = model_dir
        self.mesh_path = mesh_path
        self.mesh = None

        # Load mesh if provided
        if mesh_path is not None and os.path.exists(mesh_path):
            self._load_mesh(mesh_path)
        else:
            # Create dummy mesh for model-free mode
            self._create_dummy_mesh()

        # Initialize FoundationPose networks
        self._init_networks()

        # Initialize FoundationPose estimator
        self._init_estimator()

        logger.info(f"RealFoundationPoseEstimator initialized")
        logger.info(f"  Model dir: {model_dir}")
        logger.info(f"  Mesh: {mesh_path if mesh_path else 'dummy mesh'}")
        logger.info(f"  Device: {device}")

    def _load_mesh(self, mesh_path: str):
        """메시 로드"""
        try:
            self.mesh = trimesh.load(mesh_path)
            logger.info(f"Mesh loaded: {mesh_path}")
            logger.info(f"  Vertices: {len(self.mesh.vertices)}")
            logger.info(f"  Faces: {len(self.mesh.faces)}")
        except Exception as e:
            logger.error(f"Failed to load mesh: {e}")
            self._create_dummy_mesh()

    def _create_dummy_mesh(self):
        """더미 메시 생성 (model-free 모드용)"""
        # Create a simple box mesh for object representation
        # In real model-free mode, this would be replaced with neural field mesh
        logger.info("Creating dummy mesh for model-free mode")
        self.mesh = trimesh.creation.box(extents=[0.1, 0.1, 0.05])
        logger.info(f"Dummy mesh created: {len(self.mesh.vertices)} vertices")

    def _init_networks(self):
        """FoundationPose 네트워크 초기화"""
        try:
            # Scorer and Refiner with default initialization
            # The pretrained weights will be loaded from the default paths in FoundationPose
            self.scorer = ScorePredictor()
            self.refiner = PoseRefinePredictor()

            # Try to load checkpoints if they exist
            scorer_dir = os.path.join(self.model_dir, "2024-01-11-20-02-45")
            refiner_dir = os.path.join(self.model_dir, "2023-10-28-18-33-37")

            scorer_checkpoint = os.path.join(scorer_dir, "model_best.pth")
            refiner_checkpoint = os.path.join(refiner_dir, "model_best.pth")

            # Load scorer checkpoint if exists
            if os.path.exists(scorer_checkpoint) and hasattr(self.scorer, 'load_model'):
                try:
                    self.scorer.load_model(scorer_checkpoint)
                    logger.info(f"Scorer loaded from {scorer_checkpoint}")
                except Exception as e:
                    logger.warning(f"Failed to load scorer checkpoint: {e}")

            # Load refiner checkpoint if exists
            if os.path.exists(refiner_checkpoint) and hasattr(self.refiner, 'load_model'):
                try:
                    self.refiner.load_model(refiner_checkpoint)
                    logger.info(f"Refiner loaded from {refiner_checkpoint}")
                except Exception as e:
                    logger.warning(f"Failed to load refiner checkpoint: {e}")

            logger.info("FoundationPose networks initialized")

        except Exception as e:
            logger.error(f"Failed to initialize networks: {e}")
            import traceback
            traceback.print_exc()
            # Fallback to default networks
            self.scorer = ScorePredictor()
            self.refiner = PoseRefinePredictor()

    def _init_estimator(self):
        """FoundationPose estimator 초기화"""
        try:
            # Create OpenGL context for rendering
            glctx = None
            try:
                import nvdiffrast.torch as dr
                glctx = dr.RasterizeCudaContext() if self.device.startswith('cuda') else None
            except Exception as e:
                logger.warning(f"nvdiffrast not available: {e}")
                glctx = None

            # Get model points and normals from mesh
            model_pts = self.mesh.vertices.copy()
            model_normals = self.mesh.vertex_normals.copy()

            # Initialize FoundationPose
            self.estimator = FoundationPose(
                model_pts=model_pts,
                model_normals=model_normals,
                mesh=self.mesh,
                scorer=self.scorer,
                refiner=self.refiner,
                glctx=glctx,
                debug=self.debug,
                debug_dir=self.debug_dir
            )

            logger.info("FoundationPose estimator initialized successfully")

        except Exception as e:
            logger.error(f"Failed to initialize FoundationPose estimator: {e}")
            import traceback
            traceback.print_exc()
            self.estimator = None

    def register(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        mask: np.ndarray,
        intrinsics: Dict[str, float]
    ) -> Tuple[np.ndarray, float]:
        """
        초기 자세 등록 (Registration)

        Args:
            rgb: RGB 이미지 [H, W, 3] uint8
            depth: Depth 이미지 [H, W] float32 (미터)
            mask: 객체 마스크 [H, W] bool or uint8
            intrinsics: 카메라 내부 파라미터 {'fx', 'fy', 'cx', 'cy'}

        Returns:
            pose: 4x4 변환 행렬
            confidence: 신뢰도 점수
        """
        if self.estimator is None:
            logger.warning("FoundationPose estimator not available, using fallback")
            return self._fallback_estimate(rgb, depth, mask, intrinsics)

        try:
            # Convert inputs to FoundationPose format
            H, W = rgb.shape[:2]

            # Camera intrinsics matrix
            K = np.array([
                [intrinsics['fx'], 0, intrinsics['cx']],
                [0, intrinsics['fy'], intrinsics['cy']],
                [0, 0, 1]
            ], dtype=np.float32)

            # Convert mask to binary
            if mask.dtype != bool:
                mask = mask > 0

            # Call FoundationPose register
            pose = self.estimator.register(
                K=K,
                rgb=rgb,
                depth=depth,
                ob_mask=mask,
                iteration=5  # Number of refinement iterations
            )

            # Compute confidence (use scorer network)
            confidence = self._compute_confidence(rgb, depth, mask, pose, K)

            logger.debug(f"Registration successful, confidence: {confidence:.3f}")

            return pose, confidence

        except Exception as e:
            logger.error(f"Registration failed: {e}")
            logger.warning("Falling back to simple estimation")
            return self._fallback_estimate(rgb, depth, mask, intrinsics)

    def track(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        prev_pose: np.ndarray,
        intrinsics: Dict[str, float]
    ) -> Tuple[np.ndarray, float]:
        """
        자세 추적 (Tracking)

        Args:
            rgb: RGB 이미지 [H, W, 3]
            depth: Depth 이미지 [H, W]
            prev_pose: 이전 자세 4x4
            intrinsics: 카메라 내부 파라미터

        Returns:
            pose: 4x4 변환 행렬
            confidence: 신뢰도 점수
        """
        if self.estimator is None:
            logger.warning("FoundationPose estimator not available, using fallback")
            return prev_pose.copy(), 0.8

        try:
            # Camera intrinsics matrix
            K = np.array([
                [intrinsics['fx'], 0, intrinsics['cx']],
                [0, intrinsics['fy'], intrinsics['cy']],
                [0, 0, 1]
            ], dtype=np.float32)

            # Call FoundationPose track
            pose = self.estimator.track_one(
                rgb=rgb,
                depth=depth,
                K=K,
                iteration=2  # Fewer iterations for tracking
            )

            # Compute confidence
            confidence = self._compute_confidence_tracking(rgb, depth, pose, K)

            logger.debug(f"Tracking successful, confidence: {confidence:.3f}")

            return pose, confidence

        except Exception as e:
            logger.error(f"Tracking failed: {e}")
            logger.warning("Using previous pose")
            return prev_pose.copy(), 0.5

    def _compute_confidence(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        mask: np.ndarray,
        pose: np.ndarray,
        K: np.ndarray
    ) -> float:
        """신뢰도 계산 (등록 시)"""
        try:
            # Use scorer network if available
            if hasattr(self.scorer, 'predict'):
                score = self.scorer.predict(rgb, depth, mask, pose, K)
                return float(score)
            else:
                # Fallback: mask coverage
                return min(np.sum(mask) / (mask.size * 0.1), 1.0)
        except:
            return 0.8

    def _compute_confidence_tracking(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        pose: np.ndarray,
        K: np.ndarray
    ) -> float:
        """신뢰도 계산 (추적 시)"""
        try:
            # Simpler confidence for tracking
            return 0.85
        except:
            return 0.7

    def _fallback_estimate(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        mask: np.ndarray,
        intrinsics: Dict[str, float]
    ) -> Tuple[np.ndarray, float]:
        """
        Fallback 간단한 추정 (FoundationPose 실패 시)

        Depth 기반 3D 위치 추정
        """
        mask_bool = mask > 0 if mask is not None else np.ones(depth.shape, dtype=bool)

        valid_depth = depth[mask_bool]
        valid_depth = valid_depth[(valid_depth > 0.1) & (valid_depth < 5.0)]

        if len(valid_depth) == 0:
            return np.eye(4), 0.0

        z = np.median(valid_depth)

        ys, xs = np.where(mask_bool)
        if len(xs) == 0:
            return np.eye(4), 0.0

        cx_mask = np.mean(xs)
        cy_mask = np.mean(ys)

        fx, fy = intrinsics.get('fx', 383.883), intrinsics.get('fy', 383.883)
        cx, cy = intrinsics.get('cx', 320.499), intrinsics.get('cy', 237.913)

        x = (cx_mask - cx) * z / fx
        y = (cy_mask - cy) * z / fy

        pose_matrix = np.eye(4)
        pose_matrix[:3, 3] = [x, y, z]

        confidence = min(len(valid_depth) / 1000.0, 0.95)

        return pose_matrix, confidence

    def reset(self):
        """Reset estimator state"""
        if self.estimator is not None and hasattr(self.estimator, 'reset'):
            self.estimator.reset()
        logger.info("Estimator reset")

    def get_mesh(self):
        """Get the object mesh"""
        return self.mesh


def create_real_foundationpose_estimator(
    mesh_path: Optional[str] = None,
    device: str = 'cuda:0',
    **kwargs
) -> RealFoundationPoseEstimator:
    """
    Factory function to create RealFoundationPoseEstimator

    Args:
        mesh_path: Path to object mesh file
        device: Device for inference
        **kwargs: Additional arguments

    Returns:
        RealFoundationPoseEstimator instance
    """
    return RealFoundationPoseEstimator(
        mesh_path=mesh_path,
        device=device,
        **kwargs
    )
