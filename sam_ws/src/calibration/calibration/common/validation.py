"""Validation helpers — reprojection error, apex 3D residuals, plane RMS."""
from __future__ import annotations

import math
import numpy as np


def reprojection_rms(obj_pts, img_pts, rvec, tvec, K, D) -> float:
    """OpenCV projectPoints-based RMS reprojection error (pixels)."""
    import cv2
    obj = np.asarray(obj_pts, dtype=np.float64).reshape(-1, 1, 3)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    proj = proj.reshape(-1, 2)
    img = np.asarray(img_pts, dtype=np.float64).reshape(-1, 2)
    err = proj - img
    return float(math.sqrt(np.mean(err ** 2)))


def per_corner_max_error(obj_pts, img_pts, rvec, tvec, K, D) -> float:
    import cv2
    proj, _ = cv2.projectPoints(np.asarray(obj_pts).reshape(-1, 1, 3),
                                rvec, tvec, K, D)
    proj = proj.reshape(-1, 2)
    diff = np.linalg.norm(proj - np.asarray(img_pts).reshape(-1, 2), axis=1)
    return float(diff.max())


def inter_apex_distance_error(apexes_got: np.ndarray,
                              apexes_expected: np.ndarray) -> float:
    """Returns max abs error of pairwise distances (meters)."""
    def D(a):
        return np.linalg.norm(a[:, None, :] - a[None, :, :], axis=-1)
    return float(np.max(np.abs(D(apexes_got) - D(apexes_expected))))


def plane_rms(pts_3d, plane_abcd) -> float:
    a, b, c, d = plane_abcd
    n = np.array([a, b, c])
    n = n / (np.linalg.norm(n) + 1e-12)
    dists = pts_3d @ n + d
    return float(math.sqrt(np.mean(dists ** 2)))


def compare_criteria(value_dict: dict, pass_dict: dict) -> dict:
    """Return per-metric pass/fail and summary."""
    results = {}
    all_pass = True
    for k, (op, thr) in pass_dict.items():
        v = value_dict.get(k)
        if v is None:
            results[k] = {'value': None, 'threshold': thr, 'pass': False, 'missing': True}
            all_pass = False
            continue
        if op == '<':
            ok = v < thr
        elif op == '<=':
            ok = v <= thr
        elif op == '>':
            ok = v > thr
        else:
            ok = False
        results[k] = {'value': v, 'threshold': thr, 'op': op, 'pass': bool(ok)}
        all_pass = all_pass and ok
    results['_all_pass'] = all_pass
    return results
