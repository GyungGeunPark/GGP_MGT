"""Minimal visualization helpers."""
from __future__ import annotations

import os
from typing import Optional
import numpy as np


def overlay_charuco(image, detection, out_path: str) -> None:
    import cv2
    img = image.copy()
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if detection is None:
        cv2.imwrite(out_path, img)
        return
    for (u, v), cid in zip(detection.corners_2d, detection.ids):
        cv2.circle(img, (int(round(u)), int(round(v))), 4, (0, 255, 0), -1)
        cv2.putText(img, str(int(cid)), (int(u) + 5, int(v)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    cv2.imwrite(out_path, img)


def project_lidar_onto_image(image, pcd_pts_cam, K, D, out_path: str,
                             max_distance_m: float = 25.0):
    import cv2
    img = image.copy()
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    # Filter forward-facing points with reasonable depth
    z = pcd_pts_cam[:, 2]
    mask = (z > 0.1) & (z < max_distance_m)
    pts = pcd_pts_cam[mask]
    if len(pts) == 0:
        cv2.imwrite(out_path, img)
        return
    rvec = np.zeros(3)
    tvec = np.zeros(3)
    proj, _ = cv2.projectPoints(pts.reshape(-1, 1, 3), rvec, tvec, K, D)
    proj = proj.reshape(-1, 2)
    H, W = img.shape[:2]
    for (u, v), depth in zip(proj, pts[:, 2]):
        if not (0 <= u < W and 0 <= v < H):
            continue
        # Color by depth (near=red, far=blue)
        t = float(min(max(depth / max_distance_m, 0.0), 1.0))
        b = int(255 * t)
        r = int(255 * (1.0 - t))
        cv2.circle(img, (int(u), int(v)), 1, (b, 0, r), -1)
    cv2.imwrite(out_path, img)


def residual_histogram(residuals, out_path: str, title: str = 'residuals',
                       xlabel: str = 'value', bins: int = 50):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    plt.figure(figsize=(6, 4))
    plt.hist(np.asarray(residuals).ravel(), bins=bins, color='steelblue', alpha=0.8)
    plt.xlabel(xlabel)
    plt.ylabel('count')
    plt.title(title)
    plt.tight_layout()
    plt.savefig(out_path, dpi=120)
    plt.close()
    return True
