"""RANSAC cone model fit.

Simplified cone model with axis + half-angle priors (fast, stable).
Axis = board plane normal (from RANSAC plane fit), half-angle = known.
Only apex position (x, y, z) is estimated.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional
import math
import numpy as np


@dataclass
class ConeFitResult:
    apex: np.ndarray          # (3,) apex position
    axis: np.ndarray          # (3,) unit vector (direction from apex to base)
    half_angle_rad: float
    inliers: np.ndarray       # bool mask
    rms_m: float              # residual RMS (meters)
    n_inliers: int


def _cone_residual(pts, apex, axis, half_angle):
    """Signed distance from each point to cone surface along axis-perpendicular.

    For a point P on a right cone with apex A and axis direction u (unit vec)
    and half-angle α, the perpendicular distance from P to the cone surface is:

        d = |proj_perp| - |proj_axis| * tan(α)

    where proj_axis = (P-A)·u, proj_perp = |(P-A) - proj_axis·u|.
    """
    v = pts - apex[None, :]
    proj_axis = v @ axis
    perp = v - proj_axis[:, None] * axis[None, :]
    perp_norm = np.linalg.norm(perp, axis=1)
    return perp_norm - np.abs(proj_axis) * math.tan(half_angle)


def _refine_apex(pts, axis, half_angle, apex_init, max_iters=50, tol=1e-8):
    """Gauss-Newton refinement of apex position with axis/angle fixed."""
    apex = apex_init.astype(float).copy()
    tan_a = math.tan(half_angle)
    for _ in range(max_iters):
        v = pts - apex[None, :]
        proj_axis = v @ axis
        perp = v - proj_axis[:, None] * axis[None, :]
        perp_norm = np.linalg.norm(perp, axis=1)
        # Avoid division by zero
        perp_unit = np.divide(perp, np.maximum(perp_norm[:, None], 1e-12))
        sign_proj = np.sign(proj_axis)
        # Residual
        r = perp_norm - np.abs(proj_axis) * tan_a
        # Jacobian dr/d(apex): derivative wrt apex (3)
        # perp_norm depends on apex via perp = v - (v·u)u
        # d(perp)/d(apex) = -I + u uᵀ  (3x3), and
        # d(perp_norm)/d(apex) = perp_unit · d(perp)/d(apex) = perp_unit · (-I + u uᵀ)
        # d(proj_axis)/d(apex) = -u
        # So dr/d(apex) = perp_unit · (-I + u uᵀ) - sign_proj * (-tan_a) * u
        #                = -perp_unit + (perp_unit·u) * u + sign_proj * tan_a * u
        # But perp_unit·u = 0 by construction, so
        #   dr/d(apex) = -perp_unit + sign_proj * tan_a * u
        J = -perp_unit + (sign_proj * tan_a)[:, None] * axis[None, :]
        # Normal equations
        H = J.T @ J
        g = J.T @ r
        try:
            dx = np.linalg.solve(H, -g)
        except np.linalg.LinAlgError:
            break
        apex = apex + dx
        if np.linalg.norm(dx) < tol:
            break
    return apex


def fit_cone_ransac(pts: np.ndarray,
                    axis_prior: np.ndarray,
                    half_angle_rad: float,
                    tol_m: float,
                    min_inliers_frac: float = 0.6,
                    n_iters: int = 50,
                    apex_init_mode: str = 'centroid_along_axis',
                    apex_offset_from_centroid_m: float = 0.05,
                    seed: Optional[int] = 42,
                    ) -> Optional[ConeFitResult]:
    """Fit a solid cone to a 3D point cluster.

    Args:
        pts: (N, 3)
        axis_prior: (3,) — board plane normal (unit vector or will be normalized)
        half_angle_rad: known half-angle
        tol_m: inlier threshold (meters)
        min_inliers_frac: minimum inlier fraction to accept
        n_iters: RANSAC iterations (apex candidate sampling)
        apex_init_mode: 'centroid_along_axis' or 'random_point'
        apex_offset_from_centroid_m: initial apex offset along axis from cluster centroid

    Returns:
        ConeFitResult or None on failure
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 8:
        return None
    axis = np.asarray(axis_prior, dtype=float).ravel()
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    # Ensure axis points roughly from cluster body toward apex:
    # If centroid projection along axis is positive vs expected base, flip.
    centroid = pts.mean(axis=0)
    rng = np.random.default_rng(seed)

    best: Optional[ConeFitResult] = None
    for _ in range(n_iters):
        # Sample an initial apex candidate along the axis above/below centroid
        if apex_init_mode == 'random_point':
            idx = rng.integers(0, len(pts))
            apex_candidate = pts[idx] + axis * apex_offset_from_centroid_m
        else:
            sign = rng.choice([-1.0, 1.0])
            apex_candidate = centroid + sign * axis * apex_offset_from_centroid_m

        apex = _refine_apex(pts, axis, half_angle_rad, apex_candidate)
        res = _cone_residual(pts, apex, axis, half_angle_rad)
        inliers = np.abs(res) < tol_m
        n_in = int(inliers.sum())
        if n_in < int(min_inliers_frac * len(pts)):
            continue
        if n_in >= 3:
            apex_refined = _refine_apex(pts[inliers], axis, half_angle_rad, apex)
            res_in = _cone_residual(pts[inliers], apex_refined, axis, half_angle_rad)
            rms = float(math.sqrt(np.mean(res_in ** 2))) if n_in > 0 else float('inf')
            if best is None or n_in > best.n_inliers or (
                    n_in == best.n_inliers and rms < best.rms_m):
                best = ConeFitResult(
                    apex=apex_refined,
                    axis=axis,
                    half_angle_rad=half_angle_rad,
                    inliers=inliers,
                    rms_m=rms,
                    n_inliers=n_in,
                )

    # Try a direct (no RANSAC) refinement as fallback
    if best is None:
        apex = _refine_apex(pts, axis, half_angle_rad, centroid + axis * apex_offset_from_centroid_m)
        res = _cone_residual(pts, apex, axis, half_angle_rad)
        inliers = np.abs(res) < tol_m
        if inliers.sum() >= 4:
            rms = float(math.sqrt(np.mean(res[inliers] ** 2)))
            best = ConeFitResult(apex=apex, axis=axis,
                                 half_angle_rad=half_angle_rad,
                                 inliers=inliers, rms_m=rms,
                                 n_inliers=int(inliers.sum()))
    return best
