"""Stage 2 Joint BA (inter-system), strategy C.

Variables:
  T_unit_u_to_world       (6 × N_units, first unit fixed)
  T_lidar_to_cam_u        (6 × N_units, soft prior)
  T_board_pose_p          (6 × N_poses)

Intrinsics (K, D) are fixed (from Stage 1).

Residual terms (pose p, unit u, board frame):
  - ChArUco corner reprojection in unit cam
  - Cone apex reprojection in unit cam
  - Cone apex 3D (LiDAR frame → board frame) vs model
  - LiDAR-Cam prior (Gaussian) vs Stage 1 results
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional
import math
import numpy as np

from scipy.optimize import least_squares

from ..common.transforms import (se3_from_rvec_tvec, se3_to_rvec_tvec,
                                  se3_inverse, transform_points, se3_log)


@dataclass
class Stage2BAResult:
    T_unit_to_world: Dict[str, np.ndarray]
    T_lidar_to_cam: Dict[str, np.ndarray]
    T_board: List[np.ndarray]      # 각 포즈의 T_board_to_world
    residuals_corner_px: np.ndarray
    residuals_apex_px: Optional[np.ndarray]
    residuals_apex_3d_m: np.ndarray
    optimiser_info: dict
    sigma_rot_deg: Dict[str, np.ndarray]
    sigma_trans_mm: Dict[str, np.ndarray]


def _pack_params(units_order, T_u_to_world, T_lc, T_boards, fixed_unit_idx=0):
    """Variables: [T_u(6)] for u!=fixed + [T_lc(6)] all units + [T_board(6)] all poses."""
    x = []
    for i, u in enumerate(units_order):
        if i == fixed_unit_idx:
            continue
        r, t = se3_to_rvec_tvec(T_u_to_world[u])
        x.append(r); x.append(t)
    for u in units_order:
        r, t = se3_to_rvec_tvec(T_lc[u])
        x.append(r); x.append(t)
    for T in T_boards:
        r, t = se3_to_rvec_tvec(T)
        x.append(r); x.append(t)
    return np.concatenate(x)


def _unpack_params(x, units_order, n_poses, fixed_unit_idx, T_fixed_world):
    offs = 0
    T_u_to_world = {}
    for i, u in enumerate(units_order):
        if i == fixed_unit_idx:
            T_u_to_world[u] = T_fixed_world
            continue
        r = x[offs:offs + 3]; t = x[offs + 3:offs + 6]; offs += 6
        T_u_to_world[u] = se3_from_rvec_tvec(r, t)
    T_lc = {}
    for u in units_order:
        r = x[offs:offs + 3]; t = x[offs + 3:offs + 6]; offs += 6
        T_lc[u] = se3_from_rvec_tvec(r, t)
    T_boards = []
    for _ in range(n_poses):
        r = x[offs:offs + 3]; t = x[offs + 3:offs + 6]; offs += 6
        T_boards.append(se3_from_rvec_tvec(r, t))
    return T_u_to_world, T_lc, T_boards


def _project(K, D, pts_cam):
    import cv2
    if len(pts_cam) == 0:
        return np.zeros((0, 2))
    proj, _ = cv2.projectPoints(
        np.asarray(pts_cam, dtype=np.float64).reshape(-1, 1, 3),
        np.zeros(3), np.zeros(3), K, D)
    return proj.reshape(-1, 2)


def _residuals(x, per_pose_obs, units_order, board_def, intrinsics, weights,
               fixed_unit_idx, T_fixed_world,
               T_lc_prior, prior_sigma_rot_rad, prior_sigma_trans_m):
    n_poses = len(per_pose_obs)
    T_u_to_world, T_lc, T_boards = _unpack_params(
        x, units_order, n_poses, fixed_unit_idx, T_fixed_world)

    apex_3d_board = board_def.apex_3d_board()
    res_parts = []

    w_corner = float(weights.get('reproj_corner', 1.0))
    w_apex   = float(weights.get('reproj_apex', 3.0))
    w_apex3d = float(weights.get('apex_3d', 10.0))
    w_prior  = float(weights.get('prior_lc', 1.0))

    for p, obs in enumerate(per_pose_obs):
        T_board_to_world = T_boards[p]
        for u in units_order:
            if u not in obs:
                continue
            K = intrinsics[u]['K']; D = intrinsics[u]['D']
            # Board-frame points → world → unit camera
            T_world_to_cam_u = se3_inverse(T_u_to_world[u])
            T_board_to_cam_u = T_world_to_cam_u @ T_board_to_world
            obj_pts = obs[u]['corners_3d_board']
            pts_cam = transform_points(T_board_to_cam_u, obj_pts)
            proj = _project(K, D, pts_cam)
            r_corner = (proj - obs[u]['corners_2d']).ravel() * w_corner
            res_parts.append(r_corner)

            # Cone apex 2D
            apex_cam = transform_points(T_board_to_cam_u, apex_3d_board)
            apex_proj = _project(K, D, apex_cam)
            if obs[u].get('apex_2d') is not None:
                r_apex = (apex_proj - obs[u]['apex_2d']).ravel() * w_apex
                res_parts.append(r_apex)

            # Cone apex 3D (LiDAR frame → board frame)
            # T_lidar_to_board = inverse(T_board_to_cam_u) @ T_lc_u
            T_cam_to_board = se3_inverse(T_board_to_cam_u)
            T_lidar_to_board = T_cam_to_board @ T_lc[u]
            apex_in_board = transform_points(T_lidar_to_board, obs[u]['apex_3d_lidar'])
            r_apex3d = (apex_in_board - apex_3d_board).ravel() * w_apex3d
            res_parts.append(r_apex3d)

    # Prior on T_lc
    for u in units_order:
        if u not in T_lc_prior:
            continue
        T_delta = se3_inverse(T_lc_prior[u]) @ T_lc[u]
        delta = se3_log(T_delta)  # (rx, ry, rz, tx, ty, tz)
        r_prior = np.zeros(6)
        r_prior[0:3] = delta[0:3] / max(prior_sigma_rot_rad, 1e-8)
        r_prior[3:6] = delta[3:6] / max(prior_sigma_trans_m, 1e-8)
        res_parts.append(r_prior * w_prior)

    return np.concatenate(res_parts)


def run_stage2_joint_ba(per_pose_obs, units_order, T_u_to_world_init,
                        T_lc_init, T_board_init, board_def, intrinsics,
                        config, world_frame_unit: str) -> Stage2BAResult:
    cfg = config
    weights = cfg.get('weights', {})
    prior = cfg.get('prior', {})
    prior_sigma_rot = math.radians(float(prior.get('sigma_rot_deg', 0.1)))
    prior_sigma_trans = float(prior.get('sigma_trans_m', 0.005))

    fixed_idx = units_order.index(world_frame_unit)
    T_fixed_world = np.eye(4)   # world frame unit = identity by convention
    T_u_to_world_init[world_frame_unit] = T_fixed_world

    x0 = _pack_params(units_order, T_u_to_world_init, T_lc_init, T_board_init,
                      fixed_unit_idx=fixed_idx)

    solver_cfg = cfg.get('solver', {})

    result = least_squares(
        _residuals, x0,
        args=(per_pose_obs, units_order, board_def, intrinsics,
              weights, fixed_idx, T_fixed_world,
              T_lc_init,  # prior mean = init from Stage 1
              prior_sigma_rot, prior_sigma_trans),
        method=solver_cfg.get('method', 'trf'),
        max_nfev=solver_cfg.get('max_iters', 500) * max(10, len(x0)),
        xtol=solver_cfg.get('parameter_tolerance', 1e-12),
        ftol=solver_cfg.get('function_tolerance', 1e-12),
        verbose=solver_cfg.get('verbose', 0),
        loss=cfg.get('robust_loss', {}).get('type', 'huber'),
        f_scale=float(cfg.get('robust_loss', {}).get('k', 1.0)),
    )

    T_u_to_world, T_lc, T_boards = _unpack_params(
        result.x, units_order, len(per_pose_obs), fixed_idx, T_fixed_world)

    # Diagnostics
    apex_3d_board = board_def.apex_3d_board()
    corner_r, apex_r, apex3d_r = [], [], []
    for p, obs in enumerate(per_pose_obs):
        T_board_to_world = T_boards[p]
        for u in units_order:
            if u not in obs:
                continue
            K = intrinsics[u]['K']; D = intrinsics[u]['D']
            T_world_to_cam_u = se3_inverse(T_u_to_world[u])
            T_board_to_cam_u = T_world_to_cam_u @ T_board_to_world
            obj_pts = obs[u]['corners_3d_board']
            pts_cam = transform_points(T_board_to_cam_u, obj_pts)
            proj = _project(K, D, pts_cam)
            corner_r.append(np.linalg.norm(proj - obs[u]['corners_2d'], axis=1))
            apex_cam = transform_points(T_board_to_cam_u, apex_3d_board)
            apex_proj = _project(K, D, apex_cam)
            if obs[u].get('apex_2d') is not None:
                apex_r.append(np.linalg.norm(apex_proj - obs[u]['apex_2d'], axis=1))
            T_cam_to_board = se3_inverse(T_board_to_cam_u)
            T_lidar_to_board = T_cam_to_board @ T_lc[u]
            apex_in_board = transform_points(T_lidar_to_board, obs[u]['apex_3d_lidar'])
            apex3d_r.append(np.linalg.norm(apex_in_board - apex_3d_board, axis=1))

    corner_r = np.concatenate(corner_r) if corner_r else np.zeros(0)
    apex_r_cat = np.concatenate(apex_r) if apex_r else None
    apex3d_r_cat = np.concatenate(apex3d_r) if apex3d_r else np.zeros(0)

    # Rough covariance via JᵀJ⁻¹ from least_squares' jac (trf returns jac)
    sigma_rot_deg, sigma_trans_mm = _approx_sigma_from_jac(
        result, units_order, fixed_idx)

    return Stage2BAResult(
        T_unit_to_world=T_u_to_world,
        T_lidar_to_cam=T_lc,
        T_board=T_boards,
        residuals_corner_px=corner_r,
        residuals_apex_px=apex_r_cat,
        residuals_apex_3d_m=apex3d_r_cat,
        optimiser_info={
            'cost': float(result.cost),
            'n_evals': int(result.nfev),
            'status': int(result.status),
            'message': str(result.message),
            'n_variables': int(len(x0)),
            'n_residuals': int(len(result.fun)),
        },
        sigma_rot_deg=sigma_rot_deg,
        sigma_trans_mm=sigma_trans_mm,
    )


def _approx_sigma_from_jac(result, units_order, fixed_idx):
    """Approximate per-unit σ_rot, σ_trans from JᵀJ⁻¹ (small scale problem)."""
    sig_rot_deg = {u: np.array([0.0, 0.0, 0.0]) for u in units_order}
    sig_trans_mm = {u: np.array([0.0, 0.0, 0.0]) for u in units_order}
    try:
        jac = getattr(result, 'jac', None)
        if jac is None:
            return sig_rot_deg, sig_trans_mm
        # Diagonal of (JᵀJ)⁻¹ provides variance per parameter
        JtJ = jac.T @ jac
        try:
            cov_diag = np.diag(np.linalg.pinv(JtJ))
        except Exception:
            return sig_rot_deg, sig_trans_mm
        # Parameter layout: [T_u(6) for u!=fixed] + [T_lc(6) for all] + [T_board(6) × P]
        offs = 0
        for i, u in enumerate(units_order):
            if i == fixed_idx:
                continue
            var = cov_diag[offs:offs + 6]; offs += 6
            # rvec sigma in rad → deg
            sig_rot_deg[u] = np.sqrt(np.abs(var[0:3])) * (180.0 / math.pi)
            sig_trans_mm[u] = np.sqrt(np.abs(var[3:6])) * 1000.0
    except Exception:
        pass
    return sig_rot_deg, sig_trans_mm
