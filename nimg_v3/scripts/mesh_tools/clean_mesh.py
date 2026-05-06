#!/usr/bin/env python3
"""
clean_mesh.py — PyMeshLab 기반 메쉬 위생 처리.

설계: research/260420_fp_top5_implementation_design.md §3.3.

파이프라인:
  1. unreferenced_vertices 제거
  2. duplicate_vertices 병합
  3. close_holes (maxholesize=100)
  4. Taubin 스무딩 (체적 보존)
  5. quadric edge collapse decimation (target=5000 faces)
  6. compute_normal_per_vertex (외향 정렬)
  7. (검증) is_watertight 확인 — fail 시 warning

CLI:
  python scripts/mesh_tools/clean_mesh.py \
      --in src/nimg_v3/models/neural_fields/housing_M/Part_02.obj \
      --out src/nimg_v3/models/neural_fields/housing_M/Part_02_clean.obj \
      --target-faces 5000
"""
from __future__ import annotations

import argparse
import logging
from pathlib import Path

logger = logging.getLogger(__name__)


def clean(in_path: Path, out_path: Path,
          target_faces: int = 5000,
          fill_holes: bool = True,
          smooth: bool = True,
          orient_normals: bool = True,
          require_watertight: bool = False) -> dict:
    """메쉬 정리 후 통계 dict 반환."""
    import pymeshlab
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(str(in_path))

    n_v_in = ms.current_mesh().vertex_number()
    n_f_in = ms.current_mesh().face_number()

    try:
        ms.apply_filter("meshing_remove_unreferenced_vertices")
    except Exception:
        pass
    try:
        ms.apply_filter("meshing_remove_duplicate_vertices")
    except Exception:
        pass
    if fill_holes:
        try:
            ms.apply_filter("meshing_close_holes", maxholesize=100)
        except Exception as e:
            logger.warning("close_holes failed: %s", e)
    if smooth:
        try:
            ms.apply_filter("apply_coord_taubin_smoothing",
                            lambda1=0.5, mu=-0.53, stepsmoothnum=3)
        except Exception:
            try:
                ms.apply_filter("apply_coord_taubin_smoothing",
                                **{"lambda": 0.5}, mu=-0.53, stepsmoothnum=3)
            except Exception as e:
                logger.warning("taubin smoothing failed: %s", e)
    if target_faces and target_faces > 0:
        try:
            ms.apply_filter("meshing_decimation_quadric_edge_collapse",
                            targetfacenum=target_faces, preservenormal=True)
        except Exception as e:
            logger.warning("decimation failed: %s", e)
    if orient_normals:
        try:
            ms.apply_filter("compute_normal_per_vertex")
        except Exception:
            pass
        try:
            ms.apply_filter("meshing_re_orient_faces_coherentely")
        except Exception:
            pass

    out_path.parent.mkdir(parents=True, exist_ok=True)
    ms.save_current_mesh(str(out_path))

    n_v_out = ms.current_mesh().vertex_number()
    n_f_out = ms.current_mesh().face_number()

    # trimesh 로 watertight 확인
    import trimesh
    m = trimesh.load(str(out_path), force="mesh")
    is_water = bool(m.is_watertight)
    is_winding = bool(m.is_winding_consistent)

    stats = dict(
        in_verts=n_v_in, in_faces=n_f_in,
        out_verts=n_v_out, out_faces=n_f_out,
        is_watertight=is_water,
        is_winding_consistent=is_winding,
        bbox_diag_m=float(m.scale),
    )
    logger.info("Mesh cleaned: %s → %s\n  in: V=%d F=%d → out: V=%d F=%d  "
                "watertight=%s winding=%s diag=%.3fm",
                in_path.name, out_path.name, n_v_in, n_f_in,
                n_v_out, n_f_out, is_water, is_winding, stats["bbox_diag_m"])
    if require_watertight and not is_water:
        raise RuntimeError(f"{out_path}: NOT watertight after cleanup")
    return stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="in_path", type=Path, required=True)
    ap.add_argument("--out", dest="out_path", type=Path, required=True)
    ap.add_argument("--target-faces", type=int, default=5000)
    ap.add_argument("--no-fill-holes", action="store_true")
    ap.add_argument("--no-smooth", action="store_true")
    ap.add_argument("--require-watertight", action="store_true")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")
    clean(args.in_path, args.out_path,
          target_faces=args.target_faces,
          fill_holes=not args.no_fill_holes,
          smooth=not args.no_smooth,
          require_watertight=args.require_watertight)


if __name__ == "__main__":
    main()
