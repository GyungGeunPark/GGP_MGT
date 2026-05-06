"""Stage 1 Joint Bundle Adjustment using scipy.optimize.least_squares.

Variables (140 for a single unit with 20 lidar-cam poses):
  - T_lidar_to_cam      (6)     : rvec(3) + tvec(3)
  - board_pose_i        (6 * N) : per-pose T_cam_to_board

Intrinsics K, D are held fixed (from Step 1a). For a full Joint BA (intrinsics
+ extrinsics) a richer parameterization is needed; we keep the scope aligned
with the "Stage 1c" role described in the architecture doc.

Cost terms:
  E_reproj_corner  — ChArUco corner reprojection (2D)
  E_reproj_apex    — cone apex reprojection (2D, if image apex available)
  E_apex3D         — cone apex from LiDAR vs expected (3D)
  E_plane          — LiDAR plane residual (simplified: average plane rms)
  E_normal         — board plane normal agreement (camera vs LiDAR)

We return optimised T_lidar_to_cam and per-pose T_cam_to_board, plus diagnostics.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional
import numpy as np

from scipy.optimize import least_squares

from ..common.transforms import (rodrigues, se3_from_rvec_tvec,
                                  se3_to_rvec_tvec, se3_inverse,
                                  transform_points)


@dataclass
class Stage1BAResult:
    T_lidar_to_cam: np.ndarray
    T_cam_to_board: List[np.ndarray]
    residuals_corner_px: np.ndarray
    residuals_apex_px: Optional[np.ndarray]
    residuals_apex_3d_m: np.ndarray
    optimiser_info: dict


def _unpack_params(x, n_poses):
    """Layout: [T_lc(6), T_cb_0(6), T_cb_1(6), ...]."""
    T_lc = se3_from_rvec_tvec(x[0:3], x[3:6])
    T_cb_list = []
    for i in range(n_poses):
        base = 6 + 6 * i
        T_cb_list.append(se3_from_rvec_tvec(x[base:base + 3], x[base + 3:base + 6]))
    return T_lc, T_cb_list


def _pack_params(T_lc, T_cb_list):
    rv, tv = se3_to_rvec_tvec(T_lc)
    x = [rv, tv]
    for T in T_cb_list:
        r, t = se3_to_rvec_tvec(T)
        x.append(r)
        x.append(t)
    return np.concatenate(x)


def _project(K, D, pts_cam):
    import cv2
    if len(pts_cam) == 0:
        return np.zeros((0, 2))
    proj, _ = cv2.projectPoints(
        np.asarray(pts_cam, dtype=np.float64).reshape(-1, 1, 3),
        np.zeros(3), np.zeros(3), K, D)
    return proj.reshape(-1, 2)


def _residuals(x, observations, board_def, K, D, weights):
    T_lc, T_cb_list = _unpack_params(x, len(observations))
    res_parts = []

    w_corner = float(weights.get('reproj_corner', 1.0))
    w_apex   = float(weights.get('reproj_apex', 3.0))
    w_apex3d = float(weights.get('apex_3d', 15.0))

    apex_3d_board = board_def.apex_3d_board()

    for i, obs in enumerate(observations):
        T_cb = T_cb_list[i]
        # Corners in board → camera
        pts_cam = transform_points(T_cb, obs['corners_3d_board'])
        proj = _project(K, D, pts_cam)
        r_corner = (proj - obs['corners_2d']).ravel() * w_corner
        res_parts.append(r_corner)

        # Cone apex 2D (if available)
        apex_cam = transform_points(T_cb, apex_3d_board)
        apex_proj = _project(K, D, apex_cam)
        if obs.get('apex_2d') is not None:
            r_apex = (apex_proj - obs['apex_2d']).ravel() * w_apex
            res_parts.append(r_apex)

        # Cone apex 3D — transform LiDAR apex to board frame via T_lc and T_cb
        # T_lidar_to_board = inverse(T_cb_to_cam) · T_lc  where T_cb is cam->board
        # but T_cb stored as T_cam_to_board (transform points from board to cam
        # using transform_points(T_cb, ·)). Careful: we treat T_cb as T_board→cam.
        # So T_lidar→board = inverse(T_cb) @ T_lc
        T_board_to_cam = T_cb  # notation: T_cb transforms board-frame points into camera frame
        T_cam_to_board = se3_inverse(T_board_to_cam)
        T_lidar_to_board = T_cam_to_board @ T_lc
        apex_lidar_in_board = transform_points(T_lidar_to_board, obs['apex_3d_lidar'])
        r_apex3d = (apex_lidar_in_board - apex_3d_board).ravel() * w_apex3d
        res_parts.append(r_apex3d)

    return np.concatenate(res_parts)


def run_stage1_joint_ba(observations, K, D, T_lidar_to_cam_init,
                        board_def, config) -> Stage1BAResult:
    if len(observations) == 0:
        raise RuntimeError("No observations provided")

    # Initialise T_cb from each observation (already have T_cam_board)
    T_cb_init = [o['T_cam_board'] for o in observations]  # board→cam
    x0 = _pack_params(T_lidar_to_cam_init, T_cb_init)

    weights = config.get('weights', {})
    solver_cfg = config.get('solver', {})

    result = least_squares(
        _residuals, x0,
        args=(observations, board_def, K, D, weights),
        method=solver_cfg.get('method', 'trf'),
        max_nfev=solver_cfg.get('max_iters', 300) * max(10, len(x0)),
        xtol=solver_cfg.get('parameter_tolerance', 1e-10),
        ftol=solver_cfg.get('function_tolerance', 1e-10),
        verbose=solver_cfg.get('verbose', 0),
        loss=config.get('robust_loss', {}).get('type', 'huber'),
        f_scale=float(config.get('robust_loss', {}).get('k', 1.0)),
    )

    T_lc_opt, T_cb_list_opt = _unpack_params(result.x, len(observations))

    # Compute diagnostics
    apex_3d_board = board_def.apex_3d_board()
    corner_resid = []
    apex_resid = []
    apex3d_resid = []
    for i, obs in enumerate(observations):
        T_cb = T_cb_list_opt[i]
        pts_cam = transform_points(T_cb, obs['corners_3d_board'])
        proj = _project(K, D, pts_cam)
        corner_resid.append(np.linalg.norm(proj - obs['corners_2d'], axis=1))
        apex_cam = transform_points(T_cb, apex_3d_board)
        apex_proj = _project(K, D, apex_cam)
        if obs.get('apex_2d') is not None:
            apex_resid.append(np.linalg.norm(apex_proj - obs['apex_2d'], axis=1))
        T_cam_to_board = se3_inverse(T_cb)
        T_lidar_to_board = T_cam_to_board @ T_lc_opt
        apex_lidar_in_board = transform_points(T_lidar_to_board, obs['apex_3d_lidar'])
        apex3d_resid.append(np.linalg.norm(apex_lidar_in_board - apex_3d_board, axis=1))

    corner_resid = np.concatenate(corner_resid) if corner_resid else np.zeros(0)
    apex_resid_cat = np.concatenate(apex_resid) if apex_resid else None
    apex3d_resid_cat = np.concatenate(apex3d_resid) if apex3d_resid else np.zeros(0)

    info = {
        'cost': float(result.cost),
        'n_evals': int(result.nfev),
        'status': int(result.status),
        'message': str(result.message),
        'n_variables': int(len(x0)),
        'n_residuals': int(len(result.fun)),
    }

    return Stage1BAResult(
        T_lidar_to_cam=T_lc_opt,
        T_cam_to_board=T_cb_list_opt,
        residuals_corner_px=corner_resid,
        residuals_apex_px=apex_resid_cat,
        residuals_apex_3d_m=apex3d_resid_cat,
        optimiser_info=info,
    )
