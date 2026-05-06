"""Stage 2 initial extrinsic estimation (Python-only, no MC-Calib dependency).

For each pose:
  1. Each unit detects ChArUco on Board B, solves PnP → T_cam_u_to_board
  2. Using the world-frame unit's pose as reference, compose:
     T_unit_u_to_world = inverse(T_cam_w_to_board) @ T_cam_u_to_board
  3. Average over poses via SE(3) median to get stable initial values.
"""
from __future__ import annotations

import glob
import os
from typing import Dict, List, Optional
import numpy as np

from ..common.board_definition import BoardDef
from ..common.charuco_detect import detect_charuco, charuco_pnp
from ..common.cone_detect_image import detect_cone_apexes_in_image
from ..common.cone_detect_lidar import detect_cones_lidar
from ..common.transforms import (se3_inverse, se3_median, rigid_transform_3d,
                                  transform_points)


def collect_stage2_poses(session_dir: str) -> List[str]:
    pose_dirs = sorted(glob.glob(os.path.join(session_dir, 'pose*')))
    return [p for p in pose_dirs if os.path.isdir(p)]


def detect_all_units(pose_dir: str, units: List[str], board_def: BoardDef,
                      intrinsics: Dict[str, dict]) -> Optional[Dict[str, dict]]:
    """Return {unit: {T_cam_board, corners_2d, corners_3d_board, apex_3d_lidar, ...}}
    for a single pose. None if any unit's detection fails."""
    import cv2
    out = {}
    for u in units:
        img_path = os.path.join(pose_dir, f'cam_{u}.png')
        pcd_path = os.path.join(pose_dir, f'lidar_{u}.pcd')
        if not os.path.exists(img_path) or not os.path.exists(pcd_path):
            return None
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        det = detect_charuco(img, board_def)
        if det is None:
            return None
        K = intrinsics[u]['K']
        D = intrinsics[u]['D']
        T_cam_board = charuco_pnp(det, K, D)
        if T_cam_board is None:
            return None
        apex_2d = detect_cone_apexes_in_image(img, board_def, T_cam_board, K, D)
        apex_3d_lidar = detect_cones_lidar(pcd_path, board_def)
        out[u] = {
            'T_cam_board': T_cam_board,
            'corners_2d': det.corners_2d,
            'corners_3d_board': det.corners_3d_board,
            'ids': det.ids,
            'apex_2d': apex_2d,
            'apex_3d_lidar': apex_3d_lidar,
            'img_path': img_path,
            'pcd_path': pcd_path,
        }
    return out


def compute_initial_unit_extrinsics(per_pose_observations: List[Dict[str, dict]],
                                     world_frame_unit: str,
                                     units: List[str]) -> Dict[str, np.ndarray]:
    """For each unit, compute T_unit_to_world via many poses and take SE(3) median."""
    T_world_board_per_pose = []
    T_unit_to_world_samples = {u: [] for u in units}
    for obs in per_pose_observations:
        if world_frame_unit not in obs:
            continue
        T_world_cam_to_board = obs[world_frame_unit]['T_cam_board']
        T_world_cam_to_board_inv = se3_inverse(T_world_cam_to_board)
        for u in units:
            if u not in obs:
                continue
            T_unit_cam_to_board = obs[u]['T_cam_board']
            # T_unit_cam_in_world_frame = T_world_cam_to_board^{-1} @ T_unit_cam_to_board
            # (board → world frame → unit cam)
            T_u_to_world = T_world_cam_to_board_inv @ T_unit_cam_to_board
            T_unit_to_world_samples[u].append(T_u_to_world)

    out = {}
    for u in units:
        samples = T_unit_to_world_samples[u]
        if len(samples) == 0:
            raise RuntimeError(f"No samples for unit {u}")
        out[u] = se3_median(samples)
    return out
