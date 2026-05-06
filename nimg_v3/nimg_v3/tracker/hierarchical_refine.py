"""
Rank 4 — DynamicPose-style hierarchical refinement.

설계: research/260420_fp_top5_implementation_design.md §5.

알고리즘 (DynamicPose IROS 2025 §4):
  1. KF predict (q_pred, t_pred, omega).
  2. K 후보 회전 sampling: δR_k = sample_3d_rotation(scale=σ_rot)
     σ_rot ∝ ||omega · dt|| (정지 시 σ=0, 빠른 회전 시 σ↑).
  3. for k: T_refined_k = fp.track_one(T_hyp_k, iter=3) → ΔR_k = ||δR after refine||₂
  4. argmin_k ΔR_k → "refine 가 가장 적게 움직인 hypothesis" 가 정답.
"""
from __future__ import annotations

import logging
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


def _rotation_magnitude_deg(R: np.ndarray) -> float:
    """3x3 회전 → angle (deg)."""
    cos = max(-1.0, min(1.0, (np.trace(R) - 1.0) * 0.5))
    return float(np.degrees(np.arccos(cos)))


class HierarchicalRefiner:
    """다중 후보 회전 → FP refine → 최소 ΔR 선택.

    Args:
        fp_estimator: RealFPEstimator (또는 FallbackFPEstimator) 인스턴스.
        n_hypotheses: 1 + perturbation 수. 1 이면 비활성 (단일 호출과 동일).
        sigma_rot_deg: perturbation σ.
        omega_scale: σ ← σ_base + omega_scale × ||ω·dt|| (정지 시 σ=σ_base).
    """

    def __init__(self, fp_estimator,
                 n_hypotheses: int = 2,
                 sigma_rot_deg: float = 15.0,
                 omega_scale: float = 1.0,
                 track_refine_iter: int = 3):
        self.fp = fp_estimator
        self.n = max(1, int(n_hypotheses))
        self.sigma = float(sigma_rot_deg)
        self.omega_scale = float(omega_scale)
        self.iter = int(track_refine_iter)
        self._rng = np.random.default_rng(0xF00D)

    def _sample_perturbation(self, sigma_deg: float) -> np.ndarray:
        """4x4 transform — random axis-angle perturbation."""
        from scipy.spatial.transform import Rotation as R
        if sigma_deg < 1e-3:
            return np.eye(4)
        axis = self._rng.normal(size=3)
        axis = axis / (np.linalg.norm(axis) + 1e-9)
        angle_deg = self._rng.normal(0.0, sigma_deg)
        Rm = R.from_rotvec(axis * np.radians(angle_deg)).as_matrix()
        T = np.eye(4); T[:3, :3] = Rm
        return T

    def refine(self, frame, T_hyp: np.ndarray, omega: Optional[np.ndarray],
               dt: float = 1.0 / 30.0,
               mask: Optional[np.ndarray] = None) -> Tuple[np.ndarray, float]:
        """Returns (T_best 4x4, score_best).

        n=1 이면 fallback 으로 단일 fp.track_one 만 호출.
        """
        if self.n == 1:
            T_new, score = self.fp.track_one(
                frame.rgb, frame.depth, frame.K,
                prev_T=T_hyp, iter_n=self.iter, mask=mask,
            )
            return T_new, score

        # σ 동적 조정
        omega_norm = float(np.linalg.norm(omega)) if omega is not None else 0.0
        sigma_eff = self.sigma + self.omega_scale * np.degrees(omega_norm * dt)
        sigma_eff = min(sigma_eff, 60.0)    # safety cap

        candidates = [T_hyp]
        for _ in range(self.n - 1):
            dT = self._sample_perturbation(sigma_eff)
            candidates.append(T_hyp @ dT)

        results = []
        for c_idx, T_c in enumerate(candidates):
            try:
                T_r, score = self.fp.track_one(
                    frame.rgb, frame.depth, frame.K,
                    prev_T=T_c, iter_n=self.iter, mask=mask,
                )
            except Exception as e:
                logger.debug("hierarchical c_idx=%d track_one failed: %s",
                             c_idx, e)
                T_r, score = T_c.copy(), 0.0
            # ΔR magnitude after refinement
            R_diff = T_r[:3, :3] @ T_c[:3, :3].T
            d_deg = _rotation_magnitude_deg(R_diff)
            results.append((d_deg, score, T_r))

        # DynamicPose criterion: refinement 가 가장 적게 움직인 hypothesis 선택.
        # tie-break 으로 score 최댓값 사용.
        results.sort(key=lambda x: (x[0], -x[1]))
        best_d, best_score, best_T = results[0]
        return best_T, float(best_score)
