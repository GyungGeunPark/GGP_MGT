"""Stage 2 orchestrator."""
from __future__ import annotations

import argparse
import datetime
import glob
import math
import os
import sys
from typing import Callable, Dict, List, Optional
import numpy as np

from ..common.board_definition import BoardDef
from ..common.io_yaml import load_yaml, save_yaml
from ..common.transforms import (se3_from_rvec_tvec, se3_to_rvec_tvec,
                                  se3_inverse)
from .step2b_init import (collect_stage2_poses, detect_all_units,
                           compute_initial_unit_extrinsics)
from .joint_ba_scipy import run_stage2_joint_ba


def _load_stage1(unit: str, results_root: str = '/root/sam_ws/calibration_results/stage1',
                 session_id: Optional[str] = None) -> Dict:
    unit_dir = os.path.join(results_root, unit)
    if session_id is None:
        # Pick latest session by mtime
        sessions = sorted(glob.glob(os.path.join(unit_dir, '*')), key=os.path.getmtime)
        if not sessions:
            raise RuntimeError(f"No Stage 1 sessions for {unit} under {unit_dir}")
        session_dir = sessions[-1]
        session_id = os.path.basename(session_dir)
    else:
        session_dir = os.path.join(unit_dir, session_id)
    intr = load_yaml(os.path.join(session_dir, 'intrinsic.yaml'))
    lc = load_yaml(os.path.join(session_dir, 'lidar_to_cam.yaml'))
    K = np.array([[intr['camera_matrix']['fx'], 0, intr['camera_matrix']['cx']],
                  [0, intr['camera_matrix']['fy'], intr['camera_matrix']['cy']],
                  [0, 0, 1.0]])
    D = np.array(intr['distortion']['coeffs'])
    # Reconstruct T_lidar_to_cam from rotation_matrix + translation
    R = np.array(lc['T_lidar_to_cam']['rotation_matrix'])
    t = np.array(lc['T_lidar_to_cam']['translation_m'])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return {
        'K': K, 'D': D,
        'T_lidar_to_cam': T,
        'session_id': session_id,
        'image_size': intr['image_size'],
    }


def run_stage2(units: List[str], session_id: str, *,
               data_root: str = '/root/sam_ws/calibration_data/stage2',
               results_root: str = '/root/sam_ws/calibration_results/stage2',
               stage1_results_root: str = '/root/sam_ws/calibration_results/stage1',
               board_yaml: str = '/root/sam_ws/config/board_b_r3_params.yaml',
               config_path: str = '/root/sam_ws/config/stage2_config.yaml',
               unit_mapping_yaml: str = '/root/sam_ws/config/unit_mapping.yaml',
               progress_cb: Optional[Callable[[float, str], None]] = None,
               ) -> dict:
    if progress_cb is None:
        progress_cb = lambda p, m: print(f"[{int(p*100):3d}%] {m}")

    cfg = load_yaml(config_path)
    unit_cfg = load_yaml(unit_mapping_yaml)
    board_def = BoardDef.load(board_yaml)
    world_unit = unit_cfg.get('world_frame_unit', units[0])
    if world_unit not in units:
        raise ValueError(f"world_frame_unit '{world_unit}' not in provided units {units}")

    session_dir = os.path.join(data_root, session_id)
    out_dir = os.path.join(results_root, session_id)
    os.makedirs(out_dir, exist_ok=True)

    progress_cb(0.02, f"Stage 2 session_dir={session_dir}")

    # Load Stage 1 results per unit
    intrinsics = {}
    stage1_session_refs = {}
    for u in units:
        s1 = _load_stage1(u, stage1_results_root)
        intrinsics[u] = {'K': s1['K'], 'D': s1['D']}
        stage1_session_refs[u] = {
            'session_id': s1['session_id'],
            'T_lidar_to_cam': s1['T_lidar_to_cam'].tolist(),
        }
    progress_cb(0.10, "Stage 1 results loaded for all units")

    # Detect per pose
    pose_dirs = collect_stage2_poses(session_dir)
    if len(pose_dirs) < 6:
        raise RuntimeError(f"Need at least 6 poses, got {len(pose_dirs)}")
    per_pose = []
    for i, pd in enumerate(pose_dirs):
        obs = detect_all_units(pd, units, board_def, intrinsics)
        if obs is not None:
            per_pose.append(obs)
    if len(per_pose) < 6:
        raise RuntimeError(f"Too few valid poses after detection: {len(per_pose)}/{len(pose_dirs)}")
    progress_cb(0.40, f"Detection valid on {len(per_pose)}/{len(pose_dirs)} poses")

    # Step 2b: initial values
    T_u_to_world_init = compute_initial_unit_extrinsics(per_pose, world_unit, units)
    T_lc_init = {u: _load_stage1(u, stage1_results_root)['T_lidar_to_cam'] for u in units}
    # Board initial = world frame unit's T_cam_board
    T_board_init = []
    for obs in per_pose:
        T_board_world = obs[world_unit]['T_cam_board']
        # World frame unit has T_unit_to_world = I, so T_board_to_world =
        # T_cam_to_board from world unit's view? Actually the convention we use
        # internally: T_cam_u_to_board transforms board points into cam_u.
        # So in world frame (cam_w_frame = world), T_board_to_world = T_cam_u_to_board.
        T_board_init.append(T_board_world)
    progress_cb(0.50, "Initial values ready, running Joint BA")

    ba = run_stage2_joint_ba(per_pose, units,
                              T_u_to_world_init, T_lc_init, T_board_init,
                              board_def, intrinsics, cfg, world_unit)
    progress_cb(0.90, "BA done")

    # Validation
    corner_rms = float(math.sqrt(np.mean(ba.residuals_corner_px ** 2))) \
        if len(ba.residuals_corner_px) else float('nan')
    apex_rms = (float(math.sqrt(np.mean(ba.residuals_apex_px ** 2)))
                if ba.residuals_apex_px is not None and len(ba.residuals_apex_px) else None)
    apex3d_rms_mm = float(math.sqrt(np.mean(ba.residuals_apex_3d_m ** 2)) * 1000) \
        if len(ba.residuals_apex_3d_m) else float('nan')

    pass_cfg = cfg.get('pass_criteria', {})
    max_sigma_rot = max([float(ba.sigma_rot_deg[u].max()) for u in units])
    max_sigma_trans = max([float(ba.sigma_trans_mm[u].max()) for u in units])
    pass_dict = {
        'reproj_rms_px': corner_rms < float(pass_cfg.get('reproj_rms_px', 0.8)),
        'apex_3d_mm':    apex3d_rms_mm < float(pass_cfg.get('inter_apex_mm', 2.0)) * 10,
        'sigma_rot_deg': max_sigma_rot < float(pass_cfg.get('sigma_rot_deg', 0.01)),
        'sigma_trans_mm': max_sigma_trans < float(pass_cfg.get('sigma_trans_mm', 5.0)),
    }
    overall_pass = all(pass_dict.values())

    units_out = {}
    for u in units:
        T_uw = ba.T_unit_to_world[u]
        rvec, tvec = se3_to_rvec_tvec(T_uw)
        units_out[u] = {
            'T_cam_to_world': {
                'rotation_matrix': T_uw[:3, :3].tolist(),
                'translation_m':   T_uw[:3, 3].tolist(),
                'rvec':            rvec.tolist(),
            },
            'T_lidar_to_cam': {
                'rotation_matrix': ba.T_lidar_to_cam[u][:3, :3].tolist(),
                'translation_m':   ba.T_lidar_to_cam[u][:3, 3].tolist(),
            },
            'covariance': {
                'sigma_rot_deg':  ba.sigma_rot_deg[u].tolist(),
                'sigma_trans_mm': ba.sigma_trans_mm[u].tolist(),
            },
        }

    result = {
        'session_id': session_id,
        'world_frame_unit': world_unit,
        'units': units_out,
        'stage1_session_refs': stage1_session_refs,
        'strategy': cfg.get('strategy', 'C'),
        'residuals': {
            'corner_reprojection_rms_px': corner_rms,
            'apex_reprojection_rms_px':   apex_rms,
            'apex_3d_rms_mm':              apex3d_rms_mm,
        },
        'ba_info': ba.optimiser_info,
        'pass_details': pass_dict,
        'pass': overall_pass,
        'calibration_timestamp_utc': datetime.datetime.utcnow().isoformat() + 'Z',
    }
    save_yaml(os.path.join(out_dir, 'unit_extrinsics.yaml'), result)

    with open(os.path.join(out_dir, 'validation_report.md'), 'w') as f:
        f.write(f"# Stage 2 Validation — session={session_id}\n\n")
        for k, v in pass_dict.items():
            f.write(f"- {k}: {'PASS' if v else 'FAIL'}\n")
        f.write(f"\ncorner RMS = {corner_rms:.4f} px\n")
        f.write(f"apex 3D RMS = {apex3d_rms_mm:.3f} mm\n")
        f.write(f"max σ_rot  = {max_sigma_rot:.5f}°\n")
        f.write(f"max σ_trans = {max_sigma_trans:.3f} mm\n")
        f.write(f"overall: {'PASS' if overall_pass else 'FAIL'}\n")

    progress_cb(1.0,
                f"Stage 2 {'PASS' if overall_pass else 'FAIL'}: "
                f"corner={corner_rms:.3f}px, σ_rot={max_sigma_rot:.4f}°, "
                f"σ_trans={max_sigma_trans:.2f}mm")
    return {'pass': overall_pass, 'result': result, 'out_dir': out_dir}


def main_cli():
    parser = argparse.ArgumentParser(description='Stage 2 calibration runner')
    parser.add_argument('--units', required=True,
                        help='Comma-separated units (e.g., Gantry_Global1,Gantry_Global2,Gantry_Global3,Gantry_Global4)')
    parser.add_argument('--session-id', required=True)
    parser.add_argument('--data-root', default='/root/sam_ws/calibration_data/stage2')
    parser.add_argument('--results-root', default='/root/sam_ws/calibration_results/stage2')
    parser.add_argument('--stage1-results-root', default='/root/sam_ws/calibration_results/stage1')
    parser.add_argument('--board-yaml', default='/root/sam_ws/config/board_b_r3_params.yaml')
    parser.add_argument('--config', default='/root/sam_ws/config/stage2_config.yaml')
    parser.add_argument('--unit-mapping', default='/root/sam_ws/config/unit_mapping.yaml')
    args = parser.parse_args()

    units = [u.strip() for u in args.units.split(',') if u.strip()]
    try:
        result = run_stage2(units, args.session_id,
                             data_root=args.data_root,
                             results_root=args.results_root,
                             stage1_results_root=args.stage1_results_root,
                             board_yaml=args.board_yaml,
                             config_path=args.config,
                             unit_mapping_yaml=args.unit_mapping)
        sys.exit(0 if result['pass'] else 2)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        import traceback; traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main_cli()
