"""Step 1a — Camera intrinsic calibration using Board A r2."""
from __future__ import annotations

import glob
import os
from typing import List, Tuple
import numpy as np

from ..common.board_definition import BoardDef
from ..common.charuco_detect import detect_charuco


def collect_intrinsic_images(session_dir: str, ext_candidates=('png', 'jpg', 'jpeg')) -> List[str]:
    files = []
    for ext in ext_candidates:
        files.extend(glob.glob(os.path.join(session_dir, 'intrinsic', f'*.{ext}')))
        files.extend(glob.glob(os.path.join(session_dir, 'intrinsic', f'*.{ext.upper()}')))
    return sorted(files)


def calibrate_intrinsic(image_paths: List[str], board_def: BoardDef,
                        image_size: Tuple[int, int],
                        flags: int = 0) -> dict:
    """Run OpenCV calibrateCamera and return a result dict.

    Args:
        image_paths: list of image file paths
        board_def: BoardDef (Board A typically)
        image_size: (width, height)
        flags: OpenCV calibrateCamera flags
    """
    import cv2
    all_obj = []
    all_img = []
    valid_paths = []
    for p in image_paths:
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        det = detect_charuco(img, board_def)
        if det is None or det.count < board_def.charuco.min_corners_for_valid:
            continue
        all_obj.append(det.corners_3d_board.astype(np.float32))
        all_img.append(det.corners_2d.astype(np.float32).reshape(-1, 1, 2))
        valid_paths.append(p)

    if len(all_obj) < 6:
        raise RuntimeError(f"Too few valid images for calibration: {len(all_obj)}")

    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
        all_obj, [ip.reshape(-1, 2) for ip in all_img],
        image_size, None, None, flags=flags)

    # Per-image reproj error & max
    per_image_rms = []
    max_per_corner = 0.0
    for i, obj in enumerate(all_obj):
        proj, _ = cv2.projectPoints(obj, rvecs[i], tvecs[i], K, D)
        proj = proj.reshape(-1, 2)
        diff = proj - all_img[i].reshape(-1, 2)
        norm = np.linalg.norm(diff, axis=1)
        per_image_rms.append(float(np.sqrt(np.mean(diff ** 2))))
        max_per_corner = max(max_per_corner, float(norm.max()))

    return {
        'K': np.asarray(K, dtype=float),
        'D': np.asarray(D, dtype=float).ravel(),
        'rms_px': float(ret),
        'per_image_rms': per_image_rms,
        'max_per_corner_px': max_per_corner,
        'n_images': len(all_obj),
        'n_corners_per_image': [int(len(o)) for o in all_obj],
        'used_paths': valid_paths,
        'rvecs': [np.asarray(r).ravel().tolist() for r in rvecs],
        'tvecs': [np.asarray(t).ravel().tolist() for t in tvecs],
        'image_size': list(image_size),
    }


def get_opencv_flags(names: List[str]) -> int:
    import cv2
    mask = 0
    for n in names:
        if hasattr(cv2, n):
            mask |= getattr(cv2, n)
    return mask
