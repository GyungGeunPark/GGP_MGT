"""Basic unit tests — can be run with `pytest` without ROS2 / mainwindow."""
import math
import os
import numpy as np

import pytest

CONFIG_A = '/root/sam_ws/config/board_a_r2_params.yaml'
CONFIG_B = '/root/sam_ws/config/board_b_r3_params.yaml'


def test_board_a_load():
    from calibration.common.board_definition import BoardDef
    b = BoardDef.load(CONFIG_A)
    assert b.name == 'board_a_r2'
    assert b.charuco.squares_x == 8
    assert b.charuco.squares_y == 8
    assert len(b.cones) == 4
    apex = b.apex_3d_board()
    assert apex.shape == (4, 3)
    assert np.allclose(apex[:, 2], 0.040)


def test_board_b_load():
    from calibration.common.board_definition import BoardDef
    b = BoardDef.load(CONFIG_B)
    assert b.name == 'board_b_r3'
    assert b.charuco.squares_x == 6
    assert b.charuco.squares_y == 6
    assert len(b.cones) == 4


def test_transforms_rodrigues():
    from calibration.common.transforms import rodrigues, rotation_matrix_to_rvec
    rng = np.random.default_rng(0)
    for _ in range(20):
        rv = rng.normal(size=3) * 0.5
        R = rodrigues(rv)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-10)
        rv_back = rotation_matrix_to_rvec(R)
        R2 = rodrigues(rv_back)
        assert np.allclose(R, R2, atol=1e-10)


def test_rigid_transform_3d():
    from calibration.common.transforms import rigid_transform_3d, transform_points
    rng = np.random.default_rng(1)
    src = rng.uniform(-1, 1, size=(20, 3))
    from calibration.common.transforms import rodrigues
    R = rodrigues(np.array([0.1, 0.2, 0.3]))
    t = np.array([0.5, -0.3, 1.0])
    dst = src @ R.T + t
    T = rigid_transform_3d(src, dst)
    assert np.allclose(T[:3, :3], R, atol=1e-8)
    assert np.allclose(T[:3, 3], t, atol=1e-8)


def test_cone_ransac_synthetic():
    from calibration.common.cone_ransac import fit_cone_ransac
    rng = np.random.default_rng(7)
    apex = np.array([1.0, 2.0, 3.0])
    axis = np.array([0.0, 0.0, 1.0])
    half = math.radians(30.0)
    N = 200
    hs = rng.uniform(0.0, 0.1, N)
    rs = hs * math.tan(half)
    phis = rng.uniform(0, 2 * math.pi, N)
    pts = np.stack([
        apex[0] + rs * np.cos(phis),
        apex[1] + rs * np.sin(phis),
        apex[2] - hs,
    ], axis=1)
    pts += rng.normal(scale=0.002, size=pts.shape)
    fit = fit_cone_ransac(pts, axis_prior=axis, half_angle_rad=half, tol_m=0.005)
    assert fit is not None
    err = float(np.linalg.norm(fit.apex - apex))
    assert err < 0.01, f"apex err {err:.4f}"


def test_validation_thresholds():
    from calibration.common.validation import compare_criteria
    got = {'a': 0.1, 'b': 2.0}
    thresh = {'a': ('<', 0.2), 'b': ('<', 1.0)}
    res = compare_criteria(got, thresh)
    assert res['a']['pass'] is True
    assert res['b']['pass'] is False
    assert res['_all_pass'] is False


def test_board_image_generation_tmp(tmp_path):
    from calibration.tools._board_generator import generate_charuco_image
    out = str(tmp_path / 'board_a.png')
    path = generate_charuco_image(CONFIG_A, out, dpi=50)
    assert os.path.exists(path)
    assert os.path.getsize(path) > 1000
