"""End-to-end Stage 1 BA smoke test with synthetic ground-truth observations.

Bypasses image/point-cloud rendering (detection) and directly injects
synthetic observations into the Joint BA to verify convergence.
"""
from __future__ import annotations

import math
import numpy as np

from ..common.board_definition import BoardDef
from ..common.transforms import (rodrigues, se3_from_rvec_tvec, se3_inverse,
                                  transform_points, se3_to_rvec_tvec,
                                  rigid_transform_3d)
from ..stage1.joint_ba_scipy import run_stage1_joint_ba

BOARD_YAML = '/root/sam_ws/config/board_a_r2_params.yaml'


def _project_with_K(K, D, pts_cam):
    import cv2
    proj, _ = cv2.projectPoints(pts_cam.reshape(-1, 1, 3),
                                np.zeros(3), np.zeros(3), K, D)
    return proj.reshape(-1, 2)


def run():
    board_def = BoardDef.load(BOARD_YAML)
    K = np.array([[900.0, 0, 600.0], [0, 900.0, 600.0], [0, 0, 1.0]])
    D = np.zeros(5)

    # Ground-truth T_lidar_to_cam
    gt_rvec = np.array([0.02, -0.015, 0.01])
    gt_tvec = np.array([0.10, -0.02, 0.04])
    T_lc_gt = se3_from_rvec_tvec(gt_rvec, gt_tvec)

    rng = np.random.default_rng(123)

    # Generate N poses of the board in camera frame
    observations = []
    N = 8
    for i in range(N):
        r = rng.uniform(-0.3, 0.3, size=3)
        t = np.array([rng.uniform(-0.15, 0.15),
                      rng.uniform(-0.15, 0.15),
                      rng.uniform(2.5, 3.5)])
        T_board_to_cam = se3_from_rvec_tvec(r, t)

        # Corners in camera frame + projection (add small pixel noise)
        corners_board = _board_charuco_corners(board_def)
        corners_cam = transform_points(T_board_to_cam, corners_board)
        corners_2d = _project_with_K(K, D, corners_cam)
        corners_2d += rng.normal(scale=0.15, size=corners_2d.shape)

        # Apex 2D (observed in image)
        apex_board = board_def.apex_3d_board()
        apex_cam = transform_points(T_board_to_cam, apex_board)
        apex_2d = _project_with_K(K, D, apex_cam) + rng.normal(scale=0.2, size=(4, 2))

        # Apex 3D in LiDAR frame (truth + noise)
        T_board_to_lidar = se3_inverse(T_lc_gt) @ T_board_to_cam
        apex_lidar = transform_points(T_board_to_lidar, apex_board)
        apex_lidar += rng.normal(scale=0.002, size=apex_lidar.shape)

        # Initial T_cam_board from PnP-like noisy estimate
        T_bc_noisy = T_board_to_cam.copy()
        noise_r = rng.normal(scale=0.005, size=3)
        noise_t = rng.normal(scale=0.003, size=3)
        T_bc_noisy = se3_from_rvec_tvec(*[a + b for a, b in zip(
            se3_to_rvec_tvec(T_bc_noisy), (noise_r, noise_t))])

        observations.append({
            'T_cam_board': T_bc_noisy,
            'corners_2d': corners_2d,
            'corners_3d_board': corners_board,
            'ids': np.arange(len(corners_board)),
            'apex_2d': apex_2d,
            'apex_3d_lidar': apex_lidar,
        })

    # Initial T_lc guess (add noise to GT)
    T_lc_init = se3_from_rvec_tvec(
        gt_rvec + rng.normal(scale=0.01, size=3),
        gt_tvec + rng.normal(scale=0.01, size=3),
    )

    cfg = {
        'weights': {'reproj_corner': 1.0, 'reproj_apex': 1.0, 'apex_3d': 10.0},
        'robust_loss': {'type': 'linear', 'k': 1.0},
        'solver': {'method': 'trf', 'max_iters': 200,
                   'function_tolerance': 1e-10,
                   'parameter_tolerance': 1e-10, 'verbose': 0},
    }

    result = run_stage1_joint_ba(observations, K, D, T_lc_init, board_def, cfg)

    R_rec = result.T_lidar_to_cam[:3, :3]
    t_rec = result.T_lidar_to_cam[:3, 3]
    rot_err = math.degrees(math.acos(
        max(-1.0, min(1.0, 0.5 * (np.trace(R_rec @ rodrigues(gt_rvec).T) - 1)))))
    trans_err_mm = np.linalg.norm(t_rec - gt_tvec) * 1000.0

    corner_rms = math.sqrt(np.mean(result.residuals_corner_px ** 2))
    apex3d_rms_mm = math.sqrt(np.mean(result.residuals_apex_3d_m ** 2)) * 1000

    print("== Stage 1 BA (synthetic GT, no detection) ==")
    print(f"  rot error      = {rot_err:.4f} deg   (GT noise-limited)")
    print(f"  trans error    = {trans_err_mm:.3f} mm")
    print(f"  corner 2D RMS  = {corner_rms:.3f} px")
    print(f"  apex 3D RMS    = {apex3d_rms_mm:.2f} mm")
    print(f"  BA status      = {result.optimiser_info['message']}")

    ok = rot_err < 0.5 and trans_err_mm < 30.0
    print(f"  result: {'PASS' if ok else 'FAIL'}")
    return ok


def _board_charuco_corners(board_def):
    """Analytic internal corner grid in board frame (z=0)."""
    sx = board_def.charuco.squares_x
    sy = board_def.charuco.squares_y
    sl = board_def.charuco.square_length_m
    pts = []
    for j in range(1, sy):
        for i in range(1, sx):
            pts.append([i * sl, j * sl, 0.0])
    return np.array(pts)


def main():
    ok = run()
    import sys
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
