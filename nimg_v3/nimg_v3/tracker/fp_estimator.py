"""
FoundationPose estimator — register / track_one 단일 인터페이스.

NVlabs FoundationPose 가중치/CUDA extension 이 정상 빌드되어 있으면 실제 FP 사용.
그렇지 않으면 PCA + depth centroid 기반의 **검증용 폴백** 을 사용한다. 폴백은
정확한 6DoF 를 복원하지 못하지만 (1) 파이프라인 전체를 동작시키고 (2) 추후
실제 FP 를 붙였을 때 인터페이스 변경 없이 교체 가능하다.
"""
from __future__ import annotations

import logging
import os
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)


# ----------------------------------------------------------------------
# FoundationPose 실 경로 (NVlabs)
# ----------------------------------------------------------------------

_FP_DIR = Path("/root/rvc_scan_ws/src/FoundationPose")
_FP_AVAILABLE = False
try:
    if _FP_DIR.is_dir():
        if str(_FP_DIR) not in sys.path:
            sys.path.insert(0, str(_FP_DIR))
        from estimater import FoundationPose        # type: ignore
        from learning.training.predict_score import ScorePredictor   # type: ignore
        from learning.training.predict_pose_refine import PoseRefinePredictor  # type: ignore
        _FP_AVAILABLE = True
except Exception as e:
    logger.warning("FoundationPose runtime not importable (%s). Using fallback.", e)
    _FP_AVAILABLE = False


def _bbox_mask(mask: Optional[np.ndarray], bbox) -> np.ndarray:
    if mask is not None:
        return mask.astype(bool)
    H, W = 480, 640
    m = np.zeros((H, W), dtype=bool)
    if bbox is None:
        return m
    m[bbox.y1:bbox.y2, bbox.x1:bbox.x2] = True
    return m


# ----------------------------------------------------------------------
# Real wrapper
# ----------------------------------------------------------------------

def _load_textured_mesh(mesh_path: str, *, prefer_textured: bool = True,
                        allow_flat_gray_fallback: bool = True):
    """FoundationPose 는 텍스처 이미지가 있는 메쉬를 요구. 없으면 flat gray 추가.

    Args:
        prefer_textured: True 면 mesh_path 와 같은 stem + "_textured" suffix 의 메쉬가
            존재하면 그것을 우선 로드 (Rank 2.C — bake_texture.py 산출물).
        allow_flat_gray_fallback: False 면 텍스처 이미지 없을 때 raise (Rank 2 strict).
    """
    import trimesh
    from trimesh.visual.texture import TextureVisuals
    from PIL import Image
    from pathlib import Path

    p = Path(mesh_path)
    if prefer_textured:
        cand = p.parent / f"{p.stem}_textured{p.suffix}"
        if cand.exists():
            logger.info("Using textured mesh: %s", cand.name)
            mesh_path = str(cand)
            p = cand

    m = trimesh.load(mesh_path, force="mesh")
    need_pad = (m.visual is None
                or not hasattr(m.visual, "material")
                or m.visual.material is None
                or getattr(m.visual.material, "image", None) is None)
    if need_pad:
        if not allow_flat_gray_fallback:
            raise RuntimeError(
                f"Mesh {mesh_path} has no texture and flat-gray fallback disabled.")
        pil = Image.fromarray(np.full((32, 32, 3), 180, dtype=np.uint8))
        if hasattr(m.visual, "uv") and m.visual.uv is not None:
            uv = m.visual.uv
        else:
            uv = m.vertices[:, :2] - m.vertices[:, :2].min(axis=0)
            uv /= (uv.max(axis=0) + 1e-6)
        m.visual = TextureVisuals(uv=uv, image=pil)
        logger.warning("Mesh %s: flat-gray texture fallback applied", p.name)
    m.apply_translation(-m.center_mass)
    return m


class RealFPEstimator:
    def __init__(self, mesh_path: str, weights_root: str,
                 symmetry_tfs: Optional[List[np.ndarray]] = None,
                 prefer_textured: bool = True):
        """
        Args:
            symmetry_tfs: 4x4 transform list (정합되지 않는 self-symmetry 변환).
                          첫 entry 는 identity. None 면 FP 가 단일 identity 사용.
            prefer_textured: <mesh_stem>_textured.<ext> 가 있으면 우선 사용.
        """
        self.mesh = _load_textured_mesh(mesh_path, prefer_textured=prefer_textured)
        self.scorer = ScorePredictor()
        self.refiner = PoseRefinePredictor()
        fp_kwargs = dict(
            model_pts=np.asarray(self.mesh.vertices, dtype=np.float32),
            model_normals=np.asarray(self.mesh.vertex_normals, dtype=np.float32),
            mesh=self.mesh,
            scorer=self.scorer,
            refiner=self.refiner,
            debug=0,
        )
        if symmetry_tfs is not None and len(symmetry_tfs) > 0:
            fp_kwargs["symmetry_tfs"] = np.stack(symmetry_tfs, axis=0).astype(np.float32)
            logger.info("FP %s with %d symmetry transforms",
                        mesh_path, len(symmetry_tfs))
        self.fp = FoundationPose(**fp_kwargs)
        self.source = "foundationpose"
        self._registered = False

    def register(self, rgb: np.ndarray, depth_m: np.ndarray, K: np.ndarray,
                 mask: np.ndarray, iter_n: int = 8) -> Tuple[np.ndarray, float]:
        T = self.fp.register(
            K=K.astype(np.float32), rgb=rgb, depth=depth_m.astype(np.float32),
            ob_mask=mask.astype(bool), iteration=iter_n,
        )
        self._registered = True
        # FP scorer 점수 추출: register 직후 self.fp.pose_score 가 있으면 사용
        score = float(getattr(self.fp, "pose_last_score", 0.9))
        return np.asarray(T, dtype=np.float64), score

    def track_one(self, rgb: np.ndarray, depth_m: np.ndarray, K: np.ndarray,
                  prev_T: np.ndarray, iter_n: int = 3,
                  mask: Optional[np.ndarray] = None) -> Tuple[np.ndarray, float]:
        # track_one 은 내부의 pose_last 상태를 사용 → prev_T 가 있으면 먼저 주입
        if prev_T is not None and prev_T.shape == (4, 4):
            import torch
            self.fp.pose_last = torch.as_tensor(prev_T, dtype=torch.float32, device="cuda")
        if not self._registered:
            # 안전장치: register 없이 track_one 호출되면 prev_T 유지 후 dummy 반환
            return np.asarray(prev_T, dtype=np.float64), 0.1
        T = self.fp.track_one(
            rgb=rgb, depth=depth_m.astype(np.float32),
            K=K.astype(np.float32), iteration=iter_n,
        )
        score = float(getattr(self.fp, "pose_last_score", 0.85))
        return np.asarray(T, dtype=np.float64), score


# ----------------------------------------------------------------------
# Fallback wrapper (PCA + depth centroid)
# ----------------------------------------------------------------------

class FallbackFPEstimator:
    """실 FP 가 없을 때의 검증용 폴백. mask + depth 로 6DoF 추정.

    - translation: mask 유효 depth 픽셀의 3D centroid (PCA principal axes)
    - rotation   : PCA 주축 → Z 는 depth gradient, yaw 는 mask principal axis angle,
                   pitch/roll 은 depth plane normal 로 근사

    **부호 일관성**:
    - PCA 고유벡터 ±부호 모호성을 해결하기 위해 principal axis 의 [x,y] 부호를
      `_prev_axis_2d` 기준으로 inner-product 양이 되도록 뒤집는다.
    - Yaw 의 convention: image 좌표계에서 v(=y) 축이 아래를 향하므로
      `atan2(principal[1], principal[0])` 가 이미 **시계방향=+** 에 대응한다.
      (image 상에서 clockwise 회전 → 주축이 우상향 → 우하향 → ... → yaw 증가.)
    """

    def __init__(self, mesh_path: Optional[str] = None):
        self.source = "fallback_pca"
        self.mesh_path = mesh_path
        self._prev_axis_2d: Optional[np.ndarray] = None   # 이전 프레임 (ux, uy)
        self._prev_normal: Optional[np.ndarray] = None

    def _mask_to_6dof(self, rgb, depth_m, K, mask) -> Tuple[np.ndarray, float]:
        H, W = depth_m.shape
        # depth 인코딩이 매우 압축된 비디오(예: max 0.3m)에서도 동작하도록
        # 0 (무효 측정) 만 제외. 상한은 비디오 디코더의 max_range_m 에서 이미 cap.
        valid = mask & (depth_m > 0.01)
        n_mask = int(mask.sum())
        if valid.sum() < 30:
            # mask 자체가 충분하면 mask 중심을 이용해 2D-only fallback
            if n_mask >= 100:
                ys, xs = np.where(mask)
                u = float(xs.mean()); v = float(ys.mean())
                # depth 는 mask 중앙값의 최대치 (0 제외)
                z_cand = depth_m[mask]
                z_cand = z_cand[z_cand > 0.01]
                z = float(np.median(z_cand)) if z_cand.size else 0.7
                fx, fy = K[0, 0], K[1, 1]; cx, cy = K[0, 2], K[1, 2]
                x = (u - cx) * z / fx; y = (v - cy) * z / fy
                T = np.eye(4); T[:3, 3] = [x, y, z]
                return T, 0.5
            T = np.eye(4); T[2, 3] = 0.7
            return T, 0.1
        ys, xs = np.where(valid)
        zs = depth_m[ys, xs]
        fx, fy = K[0, 0], K[1, 1]; cx, cy = K[0, 2], K[1, 2]
        X = (xs - cx) * zs / fx
        Y = (ys - cy) * zs / fy
        Z = zs
        pts = np.stack([X, Y, Z], axis=1)
        cent = pts.mean(axis=0)
        # Yaw via PCA on mask pixels (2D)
        xs_c = xs - xs.mean(); ys_c = ys - ys.mean()
        C = np.cov(np.stack([xs_c, ys_c]))
        evals, evecs = np.linalg.eigh(C)
        principal = evecs[:, -1].astype(np.float64)
        # 부호 모호성 해결 1단계: principal[0] >= 0 (오른쪽 반평면 규약)
        if principal[0] < 0:
            principal = -principal
        # 부호 모호성 해결 2단계: 이전 프레임과의 연속성.
        # _prev_axis_2d 가 있으면 내적 양수인 쪽으로 정렬 (180° 플립 방지).
        if self._prev_axis_2d is not None:
            if float(np.dot(principal, self._prev_axis_2d)) < 0:
                principal = -principal
        self._prev_axis_2d = principal.copy()
        # image 좌표계: v 축이 아래 → atan2(principal[1], principal[0]) 이 곧 CW=+.
        yaw = float(np.arctan2(principal[1], principal[0]))
        # Pitch/roll via plane normal of 3D points
        pts_c = pts - cent
        _, _, Vt = np.linalg.svd(pts_c, full_matrices=False)
        normal = Vt[-1].astype(np.float64)
        # camera-forward 반대 (normal.z < 0) 로 정규화
        if normal[2] > 0:
            normal = -normal
        # 이전 프레임 normal 과 같은 반구면 선택 (부호 플립 방지)
        if self._prev_normal is not None:
            if float(np.dot(normal, self._prev_normal)) < 0:
                normal = -normal
        self._prev_normal = normal.copy()
        pitch = float(np.arctan2(normal[1], -normal[2]))
        roll = float(np.arctan2(normal[0], -normal[2]))
        from scipy.spatial.transform import Rotation as R
        Rm = R.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
        T = np.eye(4); T[:3, :3] = Rm; T[:3, 3] = cent
        # fallback score: **mask pixel count** 기반 (valid-depth 가 아니라).
        # YOLO-seg 가 유효 mask 를 준 경우는 신뢰할 만함을 전제로 한다.
        n = max(int(valid.sum()), n_mask // 2)
        if n > 2000:
            score = 0.80
        elif n > 800:
            score = 0.65
        elif n > 200:
            score = 0.50
        else:
            score = 0.35
        return T, score

    def register(self, rgb, depth_m, K, mask, iter_n: int = 8):
        return self._mask_to_6dof(rgb, depth_m, K, mask)

    def track_one(self, rgb, depth_m, K, prev_T, iter_n: int = 3,
                  mask: Optional[np.ndarray] = None):
        """Optional fresh mask 로 포즈 재추정 (드리프트 방지).

        `mask` 가 주어지면 그것을 그대로 사용. 없으면 prev_T 주변 ±80px 상자에서
        median depth 부근의 픽셀만 수집 (기존 로직).
        """
        if mask is not None and mask.any():
            return self._mask_to_6dof(rgb, depth_m, K, mask.astype(bool))
        H, W = depth_m.shape
        z_prev = float(prev_T[2, 3])
        x_prev = float(prev_T[0, 3])
        y_prev = float(prev_T[1, 3])
        fx, fy = K[0, 0], K[1, 1]; cx, cy = K[0, 2], K[1, 2]
        u = int(np.clip(x_prev * fx / max(z_prev, 1e-3) + cx, 0, W - 1))
        v = int(np.clip(y_prev * fy / max(z_prev, 1e-3) + cy, 0, H - 1))
        r = 80
        u1, u2 = max(0, u - r), min(W, u + r)
        v1, v2 = max(0, v - r), min(H, v + r)
        box_depth = depth_m[v1:v2, u1:u2]
        mask_full = np.zeros_like(depth_m, dtype=bool)
        mask_full[v1:v2, u1:u2] = (box_depth > z_prev - 0.15) & (box_depth < z_prev + 0.15)
        return self._mask_to_6dof(rgb, depth_m, K, mask_full)


# ----------------------------------------------------------------------
# Factory
# ----------------------------------------------------------------------

def build_fp_estimator(mesh_path: str, weights_root: str = "",
                       symmetry_tfs: Optional[List[np.ndarray]] = None,
                       prefer_textured: bool = True):
    """실 FP 우선, 실패 시 fallback."""
    if _FP_AVAILABLE:
        try:
            est = RealFPEstimator(mesh_path, weights_root,
                                  symmetry_tfs=symmetry_tfs,
                                  prefer_textured=prefer_textured)
            logger.info("FP estimator: NVlabs FoundationPose (%s)", mesh_path)
            return est
        except Exception as e:
            logger.warning("RealFPEstimator init failed (%s), falling back", e)
    logger.warning("FP estimator: FALLBACK (PCA+depth) — mesh=%s", mesh_path)
    return FallbackFPEstimator(mesh_path=mesh_path)
