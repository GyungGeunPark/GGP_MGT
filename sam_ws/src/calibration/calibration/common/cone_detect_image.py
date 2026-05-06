"""Cone apex detection in image (coarse + subpixel refinement).

Strategy:
  1. Project expected apex (using T_cam_board + K,D) into the image as a seed.
  2. Around each seed: extract ROI, threshold (white cone vs dark board),
     find contour with largest convex-hull vertex closest to expected apex.
  3. Subpixel refinement via cv2.cornerSubPix.
"""
from __future__ import annotations

from typing import Optional
import numpy as np


def project_points(pts_board, T_cam_board, K, D):
    """Project board-frame 3D points into image (N,2)."""
    import cv2
    rvec, tvec = _se3_to_rvec_tvec(T_cam_board)
    pts = np.asarray(pts_board, dtype=np.float64).reshape(-1, 1, 3)
    imgp, _ = cv2.projectPoints(pts, rvec, tvec, K, D)
    return imgp.reshape(-1, 2)


def _se3_to_rvec_tvec(T):
    from .transforms import se3_to_rvec_tvec
    return se3_to_rvec_tvec(T)


def detect_apex_in_image(image, seed_xy, roi_size_px: int = 60,
                         threshold: str = 'otsu') -> Optional[np.ndarray]:
    """Find the apex (a white convex point) around seed_xy.

    Returns (x, y) subpixel or None.
    """
    import cv2
    h, w = image.shape[:2]
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    cx, cy = int(round(seed_xy[0])), int(round(seed_xy[1]))
    r = roi_size_px
    x0, y0 = max(cx - r, 0), max(cy - r, 0)
    x1, y1 = min(cx + r, w), min(cy + r, h)
    if x1 - x0 < 4 or y1 - y0 < 4:
        return None
    roi = gray[y0:y1, x0:x1]
    if threshold == 'otsu':
        _, bw = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    else:
        _, bw = cv2.threshold(roi, int(threshold), 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    # Largest contour by area
    contour = max(contours, key=cv2.contourArea)
    if len(contour) < 4:
        return None
    hull = cv2.convexHull(contour)
    # Apex candidate = hull vertex farthest from hull centroid toward seed direction
    # but we simply use hull vertex with minimum y (toward image top for upward cone)
    # or closest to ROI center — since cone is viewed with apex pointing toward camera,
    # apex is the hull point farthest from base line. We approximate by farthest
    # vertex from the contour centroid.
    M = cv2.moments(contour)
    if M['m00'] == 0:
        return None
    cx_cnt = M['m10'] / M['m00']
    cy_cnt = M['m01'] / M['m00']
    hull_pts = hull.reshape(-1, 2).astype(np.float64)
    dists = np.linalg.norm(hull_pts - np.array([cx_cnt, cy_cnt]), axis=1)
    apex_local = hull_pts[np.argmax(dists)]
    # Subpixel refinement
    apex_local_f32 = np.array([[apex_local.astype(np.float32)]])
    try:
        refined = cv2.cornerSubPix(
            roi,
            apex_local_f32,
            winSize=(5, 5),
            zeroZone=(-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01),
        )
        apex_local = refined.ravel()
    except Exception:
        pass
    return np.array([apex_local[0] + x0, apex_local[1] + y0], dtype=np.float64)


def detect_cone_apexes_in_image(image, board_def,
                                T_cam_board,
                                K, D,
                                roi_size_px: int = 60) -> Optional[np.ndarray]:
    """Find 4 cone apexes in image using T_cam_board seed. Returns (4,2) or None."""
    apex_3d_board = board_def.apex_3d_board()
    seeds = project_points(apex_3d_board, T_cam_board, K, D)
    found = []
    for seed in seeds:
        ap = detect_apex_in_image(image, seed, roi_size_px=roi_size_px)
        if ap is None:
            return None
        found.append(ap)
    return np.stack(found, axis=0)
