"""Session management: directory layout, meta.json, synchronous capture."""
from __future__ import annotations

import datetime
import hashlib
import json
import os
import shutil
import time
from dataclasses import dataclass, field
from typing import Dict, List, Literal, Optional

from . import bridge
from .common.io_yaml import sha256_file


@dataclass
class PoseMeta:
    stage: Literal['stage1', 'stage2']
    step: Literal['intrinsic', 'lidar_cam', 'interstage'] = 'interstage'
    pose_index: int = 0
    tilt_deg: float = 0.0
    distance_m: Optional[float] = None
    stand_position_m: Optional[tuple] = None
    note: str = ''


def new_session_id(prefix: str = '') -> str:
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    return f"{ts}_{prefix}" if prefix else ts


class CalibrationSession:
    """Wraps mainwindow.iface to capture calibration data.

    - Stage 1: single unit. Directory = <root>/stage1/<unit>/<session>/<step>/poseNNN/
    - Stage 2: multi-unit synchronized. Directory = <root>/stage2/<session>/poseNNN/
    """

    LIDAR_WAIT_TIMEOUT_S = 90

    def __init__(self, stage: str, session_id: str, board_yaml: str,
                 units: List[str],
                 out_root: str = '/root/sam_ws/calibration_data'):
        assert stage in ('stage1', 'stage2')
        self.stage = stage
        self.session_id = session_id
        self.board_yaml = board_yaml
        self.units = list(units)
        self.out_root = out_root
        if stage == 'stage1':
            # Stage 1 has multiple unit sub-directories; allow multiple units in 1 session
            self.root = os.path.join(out_root, 'stage1')
        else:
            self.root = os.path.join(out_root, 'stage2', session_id)
        os.makedirs(self.root, exist_ok=True)
        self._board_sha256 = sha256_file(board_yaml) if os.path.exists(board_yaml) else None

    # ---------------- directory helpers ----------------
    def pose_dir(self, pose: PoseMeta, unit: Optional[str] = None) -> str:
        if self.stage == 'stage1':
            assert unit is not None, "stage1 pose_dir requires unit"
            base = os.path.join(self.root, unit, self.session_id, pose.step)
            os.makedirs(base, exist_ok=True)
            if pose.step == 'intrinsic':
                return base  # flat directory, files named img_poseNNN.png
            return os.path.join(base, f"pose{pose.pose_index:03d}")
        return os.path.join(self.root, f"pose{pose.pose_index:03d}")

    # ---------------- Stage 1 single-unit capture ----------------
    def capture_stage1_intrinsic(self, unit: str, pose: PoseMeta) -> str:
        """Camera-only capture for intrinsic. Returns destination path."""
        iface = bridge.get_iface()
        result = iface.img_save(unit)
        if not result.get('file_path'):
            raise RuntimeError(f"img_save failed for {unit}: {result.get('message')}")
        src = os.path.join(iface.saveImgPath, result['file_path'])
        pose_dir = self.pose_dir(PoseMeta(stage='stage1', step='intrinsic',
                                          pose_index=pose.pose_index,
                                          tilt_deg=pose.tilt_deg,
                                          distance_m=pose.distance_m), unit)
        os.makedirs(pose_dir, exist_ok=True)
        dst = os.path.join(pose_dir, f"img_pose{pose.pose_index:03d}.png")
        shutil.move(src, dst)
        return dst

    def capture_stage1_lidar_cam(self, unit: str, pose: PoseMeta,
                                  lidar_settle_s: float = 2.0) -> Dict[str, str]:
        """Camera + LiDAR capture. Returns dict with 'image' and 'lidar' abs paths."""
        iface = bridge.get_iface()
        iface.pc_save_start(unit)
        time.sleep(lidar_settle_s)
        img_result = iface.img_save(unit)
        if not img_result.get('file_path'):
            raise RuntimeError(f"img_save failed for {unit}")
        # Wait LiDAR
        pcd_abs = bridge.wait_pcl_complete(unit, timeout_s=self.LIDAR_WAIT_TIMEOUT_S)
        # Move to session directory
        pose_dir = self.pose_dir(pose, unit)
        os.makedirs(pose_dir, exist_ok=True)
        img_src = os.path.join(iface.saveImgPath, img_result['file_path'])
        img_dst = os.path.join(pose_dir, 'cam.png')
        pcd_dst = os.path.join(pose_dir, 'lidar.pcd')
        shutil.move(img_src, img_dst)
        shutil.move(pcd_abs, pcd_dst)
        self._write_meta(pose_dir, pose, {
            unit: {'image': 'cam.png', 'lidar': 'lidar.pcd',
                   'image_sha256': sha256_file(img_dst)}
        })
        return {'image': img_dst, 'lidar': pcd_dst}

    # ---------------- Stage 2 multi-unit synchronized capture -------
    def capture_stage2_pose(self, pose: PoseMeta,
                             lidar_settle_s: float = 2.0,
                             pcl_target_override: Optional[int] = 3) -> Dict[str, Dict]:
        """Capture all units synchronously. Caller should set
        ``bridge.set_pcl_target_override`` BEFORE the first pose (done here if
        pcl_target_override is not None)."""
        iface = bridge.get_iface()
        if pcl_target_override is not None:
            bridge.set_pcl_target_override({u: pcl_target_override for u in self.units})

        # Start LiDAR capture for all units (non-blocking)
        for u in self.units:
            iface.pc_save_start(u)
        time.sleep(lidar_settle_s)

        # Capture all images in rapid succession
        img_results = {}
        for u in self.units:
            r = iface.img_save(u)
            img_results[u] = r
            if not r.get('file_path'):
                raise RuntimeError(f"img_save failed for {u}")

        # Wait all LiDAR
        pcd_paths = {u: bridge.wait_pcl_complete(u, timeout_s=self.LIDAR_WAIT_TIMEOUT_S)
                     for u in self.units}

        # Move to session directory
        pose_dir = self.pose_dir(pose)
        os.makedirs(pose_dir, exist_ok=True)
        capture = {}
        for u in self.units:
            img_src = os.path.join(iface.saveImgPath, img_results[u]['file_path'])
            img_dst = os.path.join(pose_dir, f'cam_{u}.png')
            pcd_dst = os.path.join(pose_dir, f'lidar_{u}.pcd')
            shutil.move(img_src, img_dst)
            shutil.move(pcd_paths[u], pcd_dst)
            capture[u] = {
                'image': os.path.basename(img_dst),
                'image_sha256': sha256_file(img_dst),
                'lidar': os.path.basename(pcd_dst),
            }
        self._write_meta(pose_dir, pose, capture)
        return capture

    # ---------------- meta.json ----------------
    def _write_meta(self, pose_dir: str, pose: PoseMeta, capture: dict):
        meta = {
            'session_id':          self.session_id,
            'stage':               self.stage,
            'pose_index':          pose.pose_index,
            'tilt_deg':            pose.tilt_deg,
            'distance_m':          pose.distance_m,
            'stand_position_m':    list(pose.stand_position_m) if pose.stand_position_m else None,
            'board_yaml':          self.board_yaml,
            'board_params_sha256': self._board_sha256,
            'timestamp_utc':       datetime.datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ'),
            'capture':             capture,
            'notes':               pose.note,
        }
        with open(os.path.join(pose_dir, 'meta.json'), 'w') as f:
            json.dump(meta, f, indent=2)
