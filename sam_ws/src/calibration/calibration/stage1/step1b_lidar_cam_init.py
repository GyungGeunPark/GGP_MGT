"""Step 1b — Per-unit LiDAR ↔ Camera extrinsic initialization.

For each pose directory containing `cam.png` and `lidar.pcd`:
  1. Detect ChArUco → PnP → T_cam_to_board
  2. Detect cone apexes in image (optional, for BA later)
  3. Detect cone apexes in LiDAR → 4 3D points
  4. Rigid 3D transform: T_lidar_to_board = rigid_3d(apex_lidar → apex_board)
  5. T_lidar_to_cam = T_cam_to_board @ inverse(T_lidar_to_board)

Median across poses provides robust initial value for Joint BA.
"""
from __future__ import annotations

import glob
import os
from typing import List, Optional
import numpy as np

from ..common.board_definition import BoardDef
from ..common.charuco_detect import detect_charuco, charuco_pnp
from ..common.cone_detect_image import detect_cone_apexes_in_image
from ..common.cone_detect_lidar import detect_cones_lidar
from ..common.transforms import (se3_from_rvec_tvec, se3_inverse, rigid_transform_3d,
                                  se3_median)


def collect_lidar_cam_poses(session_dir: str) -> List[str]:
    pose_dirs = sorted(glob.glob(os.path.join(session_dir, 'lidar_cam', 'pose*')))
    return [p for p in pose_dirs if os.path.isdir(p)]


def init_lidar_cam_observations(pose_dirs: List[str], board_def: BoardDef,
                                K: np.ndarray, D: np.ndarray) -> List[dict]:
    """Returns list of per-pose observation dicts, each with:
        T_cam_board (4x4), T_lidar_board (4x4), T_lidar_cam (4x4),
        corners_2d, corners_3d_board, apex_2d (or None),
        apex_3d_lidar, pcd_path, img_path
    """
    import cv2
    out = []
    for pose_dir in pose_dirs:
        img_path = os.path.join(pose_dir, 'cam.png')
        pcd_path = os.path.join(pose_dir, 'lidar.pcd')
        if not os.path.exists(img_path) or not os.path.exists(pcd_path):
            continue
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        det = detect_charuco(img, board_def)
        if det is None:
            continue
        T_cam_board = charuco_pnp(det, K, D)
        if T_cam_board is None:
            continue
        apex_2d = detect_cone_apexes_in_image(img, board_def, T_cam_board, K, D)
        apex_3d_lidar = detect_cones_lidar(pcd_path, board_def)
        if apex_3d_lidar is None:
            continue
        apex_3d_board = board_def.apex_3d_board()
        T_lidar_board = rigid_transform_3d(apex_3d_lidar, apex_3d_board)
        T_lidar_cam = T_cam_board @ se3_inverse(T_lidar_board)
        out.append({
            'pose_dir': pose_dir,
            'T_cam_board': T_cam_board,
            'T_lidar_board': T_lidar_board,
            'T_lidar_cam': T_lidar_cam,
            'corners_2d': det.corners_2d,
            'corners_3d_board': det.corners_3d_board,
            'ids': det.ids,
            'apex_2d': apex_2d,
            'apex_3d_lidar': apex_3d_lidar,
            'img_path': img_path,
            'pcd_path': pcd_path,
        })
    return out


def median_lidar_cam(observations: List[dict]) -> Optional[np.ndarray]:
    if len(observations) == 0:
        return None
    return se3_median([o['T_lidar_cam'] for o in observations])
