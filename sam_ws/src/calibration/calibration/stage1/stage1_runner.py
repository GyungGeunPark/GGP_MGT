"""Stage 1 orchestrator: Step 1a → 1b → 1c + pass/fail judgement + YAML output."""
from __future__ import annotations

import argparse
import datetime
import math
import os
import sys
from typing import Callable, Optional
import numpy as np

from ..common.board_definition import BoardDef
from ..common.io_yaml import load_yaml, save_yaml
from ..common.transforms import se3_to_rvec_tvec
from .step1a_intrinsic import (calibrate_intrinsic, collect_intrinsic_images,
                                get_opencv_flags)
from .step1b_lidar_cam_init import (collect_lidar_cam_poses,
                                     init_lidar_cam_observations,
                                     median_lidar_cam)
from .joint_ba_scipy import run_stage1_joint_ba


def run_stage1(unit: str, session_id: str, *,
               data_root: str = '/root/sam_ws/calibration_data/stage1',
               results_root: str = '/root/sam_ws/calibration_results/stage1',
               board_yaml: str = '/root/sam_ws/config/board_a_r2_params.yaml',
               config_path: str = '/root/sam_ws/config/stage1_config.yaml',
               image_size=(4000, 4000),
               progress_cb: Optional[Callable[[float, str], None]] = None,
               ) -> dict:
    """End-to-end Stage 1 for a single unit.

    The session directory layout is:
        data_root/<unit>/<session_id>/intrinsic/*.png
        data_root/<unit>/<session_id>/lidar_cam/pose*/cam.png + lidar.pcd
    """
    if progress_cb is None:
        progress_cb = lambda p, m: print(f"[{int(p*100):3d}%] {m}")

    cfg = load_yaml(config_path)
    board_def = BoardDef.load(board_yaml)
    session_dir = os.path.join(data_root, unit, session_id)
    out_dir = os.path.join(results_root, unit, session_id)
    os.makedirs(out_dir, exist_ok=True)

    # ---------------- Step 1a ------------------
    progress_cb(0.05, "Step 1a: collecting intrinsic images")
    intrinsic_paths = collect_intrinsic_images(session_dir)
    if len(intrinsic_paths) < 10:
        raise RuntimeError(
            f"Need at least 10 intrinsic images, got {len(intrinsic_paths)} "
            f"in {os.path.join(session_dir, 'intrinsic')}")

    flags = get_opencv_flags(cfg.get('intrinsic', {}).get('opencv_flags', []))
    progress_cb(0.10, f"Step 1a: calibrating with {len(intrinsic_paths)} images")
    intr = calibrate_intrinsic(intrinsic_paths, board_def, image_size, flags=flags)
    progress_cb(0.30, f"Step 1a done: RMS={intr['rms_px']:.4f}px, n={intr['n_images']}")

    _write_intrinsic_yaml(intr, board_def, session_id, out_dir)

    # Stage 1a gate
    pass_rms = float(cfg.get('intrinsic', {}).get('pass_rms_px', 0.15))
    pass_max = float(cfg.get('intrinsic', {}).get('pass_per_corner_px', 0.5))
    intrinsic_pass = (intr['rms_px'] < pass_rms and
                      intr['max_per_corner_px'] < pass_max)

    # ---------------- Step 1b ------------------
    progress_cb(0.35, "Step 1b: initialising LiDAR-Camera from poses")
    pose_dirs = collect_lidar_cam_poses(session_dir)
    if len(pose_dirs) < 5:
        raise RuntimeError(
            f"Need at least 5 lidar_cam poses, got {len(pose_dirs)} "
            f"in {os.path.join(session_dir, 'lidar_cam')}")

    observations = init_lidar_cam_observations(pose_dirs, board_def,
                                                intr['K'], intr['D'])
    progress_cb(0.50, f"Step 1b: {len(observations)}/{len(pose_dirs)} poses valid")
    if len(observations) < 3:
        raise RuntimeError("Too few valid lidar-camera poses.")

    T_lc_init = median_lidar_cam(observations)

    # ---------------- Step 1c ------------------
    progress_cb(0.60, "Step 1c: Joint BA (scipy TRF)")
    ba_result = run_stage1_joint_ba(observations, intr['K'], intr['D'],
                                     T_lc_init, board_def, cfg)
    progress_cb(0.90, "Step 1c done")

    # Diagnostics
    rvec, tvec = se3_to_rvec_tvec(ba_result.T_lidar_to_cam)
    corner_rms = float(math.sqrt(np.mean(ba_result.residuals_corner_px ** 2))) \
        if len(ba_result.residuals_corner_px) else float('nan')
    apex_rms = float(math.sqrt(np.mean(ba_result.residuals_apex_px ** 2))) \
        if ba_result.residuals_apex_px is not None and len(ba_result.residuals_apex_px) else None
    apex3d_rms_mm = float(math.sqrt(np.mean(ba_result.residuals_apex_3d_m ** 2)) * 1000) \
        if len(ba_result.residuals_apex_3d_m) else float('nan')

    lc_result = {
        'unit': unit,
        'session_id': session_id,
        'intrinsic_pass': bool(intrinsic_pass),
        'intrinsic_rms_px': intr['rms_px'],
        'intrinsic_max_per_corner_px': intr['max_per_corner_px'],
        'n_intrinsic_images': intr['n_images'],
        'n_lidar_cam_poses': len(observations),
        'T_lidar_to_cam': {
            'rotation_matrix': ba_result.T_lidar_to_cam[:3, :3].tolist(),
            'translation_m':   ba_result.T_lidar_to_cam[:3, 3].tolist(),
            'rvec':            rvec.tolist(),
        },
        'residuals': {
            'corner_rms_px':      corner_rms,
            'apex_reproj_rms_px': apex_rms,
            'apex_3d_rms_mm':     apex3d_rms_mm,
        },
        'ba_info': ba_result.optimiser_info,
        'board': board_def.name,
        'board_yaml_sha256': board_def.yaml_sha256,
        'calibration_timestamp_utc': datetime.datetime.utcnow().isoformat() + 'Z',
    }
    save_yaml(os.path.join(out_dir, 'lidar_to_cam.yaml'), lc_result)

    # Pass criteria
    lc_pass_cfg = cfg.get('lidar_cam', {})
    pass_dict = {
        'intrinsic_rms_px':       intr['rms_px'] < pass_rms,
        'per_corner_max_px':      intr['max_per_corner_px'] < pass_max,
        'apex_3d_mm':             apex3d_rms_mm < float(lc_pass_cfg.get('pass_apex_3d_mm', 1.5)),
    }
    overall_pass = all(pass_dict.values())
    with open(os.path.join(out_dir, 'validation_report.md'), 'w') as f:
        f.write(f"# Stage 1 Validation — unit={unit} session={session_id}\n\n")
        for k, v in pass_dict.items():
            f.write(f"- {k}: {'PASS' if v else 'FAIL'}\n")
        f.write(f"\nintrinsic RMS = {intr['rms_px']:.4f} px\n")
        f.write(f"apex 3D RMS = {apex3d_rms_mm:.3f} mm\n")
        f.write(f"corner 2D RMS = {corner_rms:.4f} px\n")
        f.write(f"overall: {'PASS' if overall_pass else 'FAIL'}\n")

    status = "PASS" if overall_pass else "FAIL"
    progress_cb(1.0, f"Stage 1 {status}: intr={intr['rms_px']:.3f}px, apex3D={apex3d_rms_mm:.2f}mm")

    return {
        'pass': overall_pass,
        'result': lc_result,
        'out_dir': out_dir,
    }


def _write_intrinsic_yaml(intr: dict, board_def: BoardDef, session_id: str, out_dir: str):
    D = intr['D'].ravel().tolist()
    # RATIONAL model can yield up to 14 params: k1, k2, p1, p2, k3, k4, k5, k6, s1..s4, τx, τy
    # Store all present elements.
    K = intr['K']
    intrinsic_dict = {
        'camera_matrix': {
            'fx': float(K[0, 0]),
            'fy': float(K[1, 1]),
            'cx': float(K[0, 2]),
            'cy': float(K[1, 2]),
        },
        'distortion': {'model': 'RATIONAL' if len(D) > 4 else 'KANNALA',
                       'coeffs': [float(x) for x in D]},
        'image_size': list(intr['image_size']),
        'residuals': {
            'reprojection_rms_px': float(intr['rms_px']),
            'reprojection_max_px': float(intr['max_per_corner_px']),
            'n_images': int(intr['n_images']),
        },
        'calibration_timestamp': datetime.datetime.utcnow().isoformat() + 'Z',
        'session_id': session_id,
        'board': board_def.name,
    }
    save_yaml(os.path.join(out_dir, 'intrinsic.yaml'), intrinsic_dict)


# ---------------------- CLI entry point ----------------------

def main_cli():
    parser = argparse.ArgumentParser(description='Stage 1 calibration runner')
    parser.add_argument('--unit', required=True, help='Camera/LiDAR unit key (e.g., Gantry_Global1)')
    parser.add_argument('--session-id', required=True, help='Session identifier')
    parser.add_argument('--data-root', default='/root/sam_ws/calibration_data/stage1')
    parser.add_argument('--results-root', default='/root/sam_ws/calibration_results/stage1')
    parser.add_argument('--board-yaml', default='/root/sam_ws/config/board_a_r2_params.yaml')
    parser.add_argument('--config', default='/root/sam_ws/config/stage1_config.yaml')
    parser.add_argument('--image-width', type=int, default=4000)
    parser.add_argument('--image-height', type=int, default=4000)
    args = parser.parse_args()

    try:
        result = run_stage1(args.unit, args.session_id,
                             data_root=args.data_root,
                             results_root=args.results_root,
                             board_yaml=args.board_yaml,
                             config_path=args.config,
                             image_size=(args.image_width, args.image_height))
        sys.exit(0 if result['pass'] else 2)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main_cli()
