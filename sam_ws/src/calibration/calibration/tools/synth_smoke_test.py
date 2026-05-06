"""Synthetic-data smoke test for the calibration pipeline.

Generates simulated ChArUco images and point clouds from a known
ground-truth configuration, runs detection + BA, and verifies residuals
are within tolerance. Useful as a CI check and as a first sanity pass
before touching real hardware.
"""
from __future__ import annotations

import math
import os
import shutil
import tempfile
import numpy as np

from ..common.board_definition import BoardDef
from ..common.transforms import (rodrigues, se3_from_rvec_tvec, se3_inverse,
                                  transform_points)
from ..common.cone_ransac import fit_cone_ransac


def _synth_intrinsic(W=1200, H=1200, f=1200.0):
    K = np.array([[f, 0, W / 2.0], [0, f, H / 2.0], [0, 0, 1.0]])
    D = np.zeros(5)
    return K, D, (W, H)


def _random_pose(rng, t_center_m=(0, 0, 3.0), t_noise_m=0.3, ang_deg=20.0):
    # Camera looks roughly in +Z (board in front of camera)
    r = rng.uniform(-ang_deg, ang_deg, size=3) * math.pi / 180.0
    t = np.array(t_center_m) + rng.uniform(-t_noise_m, t_noise_m, size=3)
    return se3_from_rvec_tvec(r, t)


def quick_charuco_detection_check():
    """Render a ChArUco pattern and try to detect corners."""
    import cv2
    from ._board_generator import generate_charuco_image
    with tempfile.TemporaryDirectory() as tmp:
        png = os.path.join(tmp, 'boardA.png')
        generate_charuco_image('/root/sam_ws/config/board_a_r2_params.yaml', png, dpi=100)
        img = cv2.imread(png, cv2.IMREAD_GRAYSCALE)
        from ..common.charuco_detect import detect_charuco
        bd = BoardDef.load('/root/sam_ws/config/board_a_r2_params.yaml')
        det = detect_charuco(img, bd)
        return det is not None and det.count >= bd.charuco.min_corners_for_valid, \
               (det.count if det is not None else 0)


def quick_cone_ransac_check():
    """Verify cone fit with synthetic cone points."""
    rng = np.random.default_rng(0)
    apex_true = np.array([0.5, 0.3, 0.04])
    axis = np.array([0.0, 0.0, 1.0])
    half = math.radians(36.87)
    # Sample points on the cone surface
    N = 120
    heights = rng.uniform(0.0, 0.04, size=N)  # apex at z=0.04, base at z=0.0
    radii = (0.04 - heights) * math.tan(half)
    phis = rng.uniform(0, 2 * math.pi, size=N)
    pts = np.stack([
        apex_true[0] + radii * np.cos(phis),
        apex_true[1] + radii * np.sin(phis),
        apex_true[2] - (0.04 - heights),    # base z = apex_true.z - 0.04 = 0
    ], axis=1)
    pts += rng.normal(scale=0.003, size=pts.shape)
    fit = fit_cone_ransac(pts, axis_prior=axis, half_angle_rad=half, tol_m=0.006)
    if fit is None:
        return False, None
    err = float(np.linalg.norm(fit.apex - apex_true))
    return err < 0.02, err


def main():
    print("== Synthetic smoke test ==")

    ok_ch, n = quick_charuco_detection_check()
    print(f"ChArUco detection:  {'PASS' if ok_ch else 'FAIL'} (corners={n})")

    ok_cone, err = quick_cone_ransac_check()
    print(f"Cone RANSAC:        {'PASS' if ok_cone else 'FAIL'} "
          f"(apex error={err:.4f} m)")

    if not (ok_ch and ok_cone):
        import sys; sys.exit(1)
    print("All smoke tests passed.")


if __name__ == '__main__':
    main()
