#!/usr/bin/env python3
"""
Phase 1 — 오프라인 템플릿 DB 빌드.

입력: `src/nimg_v3/models/neural_fields/<class>/Part_XX.obj`
출력: `src/nimg_v3/models/neural_fields/<class>/templates/template_db.npz`

PyRender 가 사용 가능하면 42뷰 icosphere 렌더, 아니면 reference_images/ 폴백.
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]       # /root/rvc_scan_ws
sys.path.insert(0, str(ROOT / "src" / "nimg_v3"))

from nimg_v3.recognition import TemplateDB, build_templates_for_class, render_icosphere_views

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("build_templates")


CLASS_CONFIG = {
    0: {
        "class_name": "housing_M",
        "mesh_candidates": ["Part_02.obj", "Part_01.obj"],   # 우선순위
        "ref_dir": "reference_images",
    },
    1: {
        "class_name": "Wiring_tray",
        "mesh_candidates": ["Part_02.obj"],
        "ref_dir": "reference_images",
    },
}


def _pick_mesh(class_dir: Path, candidates: list[str]) -> Path | None:
    for c in candidates:
        p = class_dir / c
        if p.exists():
            return p
    # 첫 번째 .obj 를 대체
    any_obj = list(class_dir.glob("*.obj"))
    return any_obj[0] if any_obj else None


def build(root: Path, encoder: str, out: Path, img_size: int, n_views: int,
          save_views: bool = True):
    db = TemplateDB()
    for class_id, meta in CLASS_CONFIG.items():
        class_dir = root / meta["class_name"]
        if not class_dir.exists():
            logger.warning("Class dir missing: %s", class_dir)
            continue
        mesh_path = _pick_mesh(class_dir, meta["mesh_candidates"])
        if not mesh_path:
            logger.warning("No mesh for class %s", meta["class_name"])
            continue
        ref_dir = class_dir / meta["ref_dir"]
        logger.info("=== class_id=%d name=%s mesh=%s ===",
                    class_id, meta["class_name"], mesh_path.name)
        try:
            feats, poses, enc_name = build_templates_for_class(
                class_id=class_id,
                class_name=meta["class_name"],
                mesh_path=str(mesh_path),
                ref_dir=ref_dir,
                encoder_name=encoder,
                n_views=n_views,
                img_size=img_size,
            )
        except Exception as e:
            logger.error("class %s failed: %s", meta["class_name"], e, exc_info=True)
            continue
        db.add_class(class_id=class_id, class_name=meta["class_name"],
                     mesh_path=str(mesh_path), encoder_name=enc_name,
                     feats=feats, view_poses=poses)
        if save_views:
            # Save preview grid
            preview_dir = class_dir / "templates"
            preview_dir.mkdir(parents=True, exist_ok=True)
            rendered = render_icosphere_views(str(mesh_path), n_views=n_views, img_size=img_size)
            if rendered is not None:
                images, _ = rendered
                for i, img in enumerate(images[:12]):
                    cv2.imwrite(str(preview_dir / f"view_{i:02d}.png"),
                                cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                logger.info("Saved preview views for %s", meta["class_name"])
    db.save(out)
    return db


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path,
                    default=ROOT / "src" / "nimg_v3" / "models" / "neural_fields")
    ap.add_argument("--out", type=Path,
                    default=ROOT / "src" / "nimg_v3" / "models" / "neural_fields" / "template_db.npz")
    ap.add_argument("--encoder", default="dinov3-vitl16",
                    help="preferred encoder; fallback chain → dinov2-large → hsvhist")
    ap.add_argument("--n-views", type=int, default=42)
    ap.add_argument("--img-size", type=int, default=224)
    args = ap.parse_args()
    build(args.root, args.encoder, args.out, args.img_size, args.n_views)


if __name__ == "__main__":
    main()
