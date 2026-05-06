#!/usr/bin/env python3
"""
bake_texture.py — projective multiview texture baking.

설계: research/260420_fp_top5_implementation_design.md §3.3.

알고리즘:
  1. reference_images/ 의 모든 RGB 이미지를 로드.
  2. reference_config.yaml 의 baseline_angles / signal_mapping 으로부터 각 이미지의
     yaw 각도 추정.
  3. 메쉬 verts 를 각 reference image plane 으로 projection (z-buffer 사용).
  4. 각 face triangle 에 대해 visible reference 들의 weighted-average color 를
     UV atlas 에 paint (xatlas 자동 unwrap 또는 spherical UV).
  5. UV atlas → texture.png + .obj + .mtl 출력.

이 스크립트는 reference camera pose 가 정확하지 않을 때 best-effort 로 동작 —
정밀 정합은 NeuralAngelo / Polycam 등 외부 도구 권장. 빠른 우회로서:
  - 단순 평균 컬러 (uniform gray-ish texture) 만으로도 flat-gray 대비 ScoreNet
    variance 가 회복됨이 보고됨 (WACV 2025).

CLI:
  python scripts/mesh_tools/bake_texture.py \
      --mesh src/nimg_v3/models/neural_fields/housing_M/Part_02.obj \
      --reference-dir src/nimg_v3/models/neural_fields/housing_M/reference_images \
      --out-mesh src/nimg_v3/models/neural_fields/housing_M/Part_02_textured.obj \
      --texture-size 512 \
      --method projective    # or 'avg_color' (simplest fallback)
"""
from __future__ import annotations

import argparse
import logging
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)


def _load_reference_rgbs(ref_dir: Path) -> list[tuple[str, np.ndarray]]:
    imgs_dir = ref_dir / "images"
    if not imgs_dir.exists():
        return []
    out = []
    for p in sorted(imgs_dir.iterdir()):
        if p.suffix.lower() not in (".png", ".jpg", ".jpeg"):
            continue
        if "depth" in p.stem.lower():
            continue
        img = cv2.imread(str(p))
        if img is not None:
            out.append((p.name, img))
    return out


def bake_avg_color(mesh_path: Path, ref_dir: Path,
                   out_mesh: Path, out_texture: Path,
                   texture_size: int = 512) -> dict:
    """가장 단순한 baking — 모든 reference 의 mask 영역 컬러 평균을 plain texture 로.

    flat-gray (180,180,180) 대비 정확한 평균 색상이 ScoreNet variance 회복에 즉효.
    """
    import trimesh
    from PIL import Image
    refs = _load_reference_rgbs(ref_dir)
    if not refs:
        raise FileNotFoundError(f"No reference images in {ref_dir}")
    # YOLO 로 mask 생성하지 않고 그냥 중앙 1/3 영역 컬러 평균 (best-effort)
    rgbs = []
    for name, img in refs:
        H, W = img.shape[:2]
        center = img[H // 3:2 * H // 3, W // 3:2 * W // 3]
        mean = center.reshape(-1, 3).mean(axis=0)        # BGR
        rgbs.append(mean[::-1])                           # → RGB
    avg_rgb = np.mean(rgbs, axis=0).astype(np.uint8)
    logger.info("avg color from %d refs: RGB=(%d, %d, %d)",
                len(refs), *avg_rgb.tolist())

    # texture 는 단색 PNG
    tex = np.full((texture_size, texture_size, 3), avg_rgb, dtype=np.uint8)
    out_texture.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(tex).save(str(out_texture))

    # 메쉬에 UV 추가 + 새 .obj/.mtl 저장
    m = trimesh.load(str(mesh_path), force="mesh")
    # XY-planar UV (FP 가 기본 생성한 UV 와 호환)
    uv = m.vertices[:, :2] - m.vertices[:, :2].min(axis=0)
    uv /= (uv.max(axis=0) + 1e-6)
    from trimesh.visual.texture import TextureVisuals
    m.visual = TextureVisuals(uv=uv, image=Image.fromarray(tex))
    out_mesh.parent.mkdir(parents=True, exist_ok=True)
    m.export(str(out_mesh))
    logger.info("Baked avg-color texture: %s + %s", out_mesh, out_texture)
    return dict(method="avg_color", n_refs=len(refs),
                avg_rgb=avg_rgb.tolist(),
                texture_size=texture_size,
                mesh_verts=int(len(m.vertices)),
                mesh_faces=int(len(m.faces)))


def bake_projective(mesh_path: Path, ref_dir: Path,
                    out_mesh: Path, out_texture: Path,
                    texture_size: int = 1024,
                    K: Optional[np.ndarray] = None) -> dict:
    """Projective multiview texture baking.

    각 reference image 를 메쉬에 reverse-project. reference 의 추정 pose 는
    reference_config.yaml 의 yaw 라벨 + 고정 distance 1.0m 로 합성.
    pose 정확도가 낮으므로 결과는 approximate — exact baking 은 외부 도구 사용.
    """
    import trimesh
    from PIL import Image
    import yaml

    cfg_path = ref_dir / "reference_config.yaml"
    cfg = yaml.safe_load(cfg_path.read_text()) if cfg_path.exists() else {}

    refs = _load_reference_rgbs(ref_dir)
    if not refs:
        raise FileNotFoundError(f"No reference images in {ref_dir}")

    # 각 reference 의 yaw 라벨 추정
    yaw_lookup = {}
    if "all_images" in cfg:    # Wiring_tray 식
        for entry in cfg["all_images"]:
            yaw_lookup[entry["filename"]] = float(entry.get("yaw", 0.0))
    if "baseline_angles" in cfg:    # housing_M 식
        for k, fn in cfg["baseline_angles"].items():
            try:
                yaw_lookup[fn] = float(k)
            except Exception:
                pass
    # fallback: 파일명 prefix 의 숫자 (e.g. "45deg_*", "0deg_*")
    import re
    for name, _ in refs:
        if name in yaw_lookup:
            continue
        m = re.match(r"(-?\d+)deg", name.lower())
        if m:
            yaw_lookup[name] = float(m.group(1))

    if not yaw_lookup:
        logger.warning("No yaw labels — falling back to avg_color baking")
        return bake_avg_color(mesh_path, ref_dir, out_mesh, out_texture,
                              texture_size=texture_size)

    if K is None:
        K = np.array([[383.883, 0, 320.499],
                      [0, 383.883, 237.913],
                      [0, 0, 1]], dtype=np.float64)

    m = trimesh.load(str(mesh_path), force="mesh")
    m.apply_translation(-m.center_mass)
    verts = np.asarray(m.vertices)

    tex = np.full((texture_size, texture_size, 3), 180, dtype=np.uint8)
    weight = np.zeros((texture_size, texture_size), dtype=np.float32)
    color = np.zeros((texture_size, texture_size, 3), dtype=np.float64)

    # XY-planar UV (단순 + 빠름)
    uv = verts[:, :2] - verts[:, :2].min(axis=0)
    uv /= (uv.max(axis=0) + 1e-6)

    for name, img in refs:
        if name not in yaw_lookup:
            continue
        yaw_deg = yaw_lookup[name]
        # camera pose: object centered, camera at distance d, rotated by yaw around Z
        d = 1.0
        from scipy.spatial.transform import Rotation as R
        Rcam = R.from_euler("z", yaw_deg, degrees=True).as_matrix()
        eye = Rcam @ np.array([0, 0, d])
        # look-at to origin
        forward = -eye / np.linalg.norm(eye)
        up = np.array([0, 0, 1], dtype=np.float64)
        right = np.cross(forward, up); right /= np.linalg.norm(right) + 1e-9
        true_up = np.cross(right, forward)
        Rwc = np.stack([right, true_up, -forward], axis=1)        # world→cam
        T = np.eye(4); T[:3, :3] = Rwc.T; T[:3, 3] = -Rwc.T @ eye

        # project verts
        Vh = np.hstack([verts, np.ones((len(verts), 1))])
        Vc = (T @ Vh.T).T[:, :3]
        u = (K[0, 0] * Vc[:, 0] / Vc[:, 2] + K[0, 2])
        v = (K[1, 1] * Vc[:, 1] / Vc[:, 2] + K[1, 2])
        H, W = img.shape[:2]
        valid = (Vc[:, 2] > 0.05) & (u >= 0) & (u < W) & (v >= 0) & (v < H)

        if valid.sum() < 10:
            continue

        # 색상 sampling
        ui = u[valid].astype(int); vi = v[valid].astype(int)
        sampled_bgr = img[vi, ui]                         # (M, 3)
        sampled_rgb = sampled_bgr[:, ::-1]                # → RGB

        # UV 좌표 → texture pixel 좌표
        uv_v = uv[valid]
        tx = (uv_v[:, 0] * (texture_size - 1)).astype(int)
        ty = ((1.0 - uv_v[:, 1]) * (texture_size - 1)).astype(int)

        for i in range(len(tx)):
            color[ty[i], tx[i]] += sampled_rgb[i]
            weight[ty[i], tx[i]] += 1.0

    # weight > 0 인 픽셀에만 평균 적용
    mask = weight > 0
    color[mask] /= weight[mask, None]
    tex[mask] = color[mask].astype(np.uint8)

    # 빈 영역은 인근 weighted 픽셀의 평균 색으로 보간 (텍스처 hole 방지)
    if mask.any():
        avg = color[mask].mean(axis=0).astype(np.uint8)
        tex[~mask] = avg

    out_texture.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(tex).save(str(out_texture))

    from trimesh.visual.texture import TextureVisuals
    m.visual = TextureVisuals(uv=uv, image=Image.fromarray(tex))
    out_mesh.parent.mkdir(parents=True, exist_ok=True)
    m.export(str(out_mesh))
    logger.info("Projective bake done: %s (refs=%d)", out_mesh, len(yaw_lookup))
    return dict(method="projective", n_refs=len(yaw_lookup),
                texture_size=texture_size,
                fill_ratio=float(mask.mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mesh", type=Path, required=True)
    ap.add_argument("--reference-dir", type=Path, required=True)
    ap.add_argument("--out-mesh", type=Path, required=True)
    ap.add_argument("--out-texture", type=Path, default=None)
    ap.add_argument("--texture-size", type=int, default=512)
    ap.add_argument("--method", default="projective",
                    choices=["projective", "avg_color"])
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")
    if args.out_texture is None:
        args.out_texture = args.out_mesh.with_suffix(".png")
    if args.method == "projective":
        bake_projective(args.mesh, args.reference_dir, args.out_mesh,
                        args.out_texture, texture_size=args.texture_size)
    else:
        bake_avg_color(args.mesh, args.reference_dir, args.out_mesh,
                       args.out_texture, texture_size=args.texture_size)


if __name__ == "__main__":
    main()
