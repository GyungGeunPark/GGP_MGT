"""
Template Builder — 메쉬 → 42뷰 icosphere 렌더 → 인코딩 → TemplateDB.

architecture §5.1–5.3 에 대응. PyRender 가 없어도(EGL 실패 등) `reference_images/`
의 기존 실촬 이미지를 대체 템플릿으로 쓸 수 있는 폴백 경로를 제공한다.
"""
from __future__ import annotations

import logging
import os
import re
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .dinov3_encoder import build_encoder
from .template_db import TemplateDB

logger = logging.getLogger(__name__)

# 헤드리스 환경에서 pyrender 가 OpenGL context 를 얻지 못하는 경우가 많다.
# pyrender > EGL > osmesa 순으로 시도.
os.environ.setdefault("PYOPENGL_PLATFORM", "egl")


def _look_at(eye: np.ndarray, target: np.ndarray, up: np.ndarray) -> np.ndarray:
    f = (target - eye); f /= np.linalg.norm(f)
    s = np.cross(f, up); s /= np.linalg.norm(s) + 1e-8
    u = np.cross(s, f)
    M = np.eye(4)
    M[:3, 0] = s
    M[:3, 1] = u
    M[:3, 2] = -f
    M[:3, 3] = eye
    return M


def render_icosphere_views(mesh_path: str, n_views: int = 42,
                           img_size: int = 224) -> Optional[Tuple[List[np.ndarray], np.ndarray]]:
    """
    Returns (images list [H,W,3] uint8 RGB, poses (N, 4, 4)) or None if renderer unusable.
    """
    try:
        import trimesh
        import pyrender
    except ImportError as e:
        logger.warning("pyrender/trimesh not available: %s", e)
        return None

    try:
        mesh_tm = trimesh.load(mesh_path, force="mesh")
    except Exception as e:
        logger.error("mesh load failed: %s", e)
        return None
    if not hasattr(mesh_tm, "extents") or mesh_tm.extents is None:
        logger.error("mesh has no extents")
        return None
    extent = float(mesh_tm.extents.max())
    if extent <= 0:
        logger.error("mesh extent == 0")
        return None
    # center at origin
    mesh_tm.apply_translation(-mesh_tm.center_mass)

    ico = trimesh.creation.icosphere(subdivisions=1)
    verts = ico.vertices
    if n_views < len(verts):
        verts = verts[:n_views]

    r_cam = 2.5 * extent
    try:
        renderer = pyrender.OffscreenRenderer(img_size, img_size)
    except Exception as e:
        logger.warning("pyrender OffscreenRenderer failed: %s", e)
        return None

    images: List[np.ndarray] = []
    poses: List[np.ndarray] = []
    try:
        for v in verts:
            eye = v * r_cam
            target = np.zeros(3)
            up = np.array([0.0, 0.0, 1.0]) if abs(v[2]) < 0.9 else np.array([0.0, 1.0, 0.0])
            cam_pose = _look_at(eye, target, up)
            scene = pyrender.Scene(bg_color=np.array([0, 0, 0, 0]), ambient_light=[0.3, 0.3, 0.3])
            scene.add(pyrender.Mesh.from_trimesh(mesh_tm, smooth=False))
            scene.add(pyrender.PerspectiveCamera(yfov=np.pi / 3.0), pose=cam_pose)
            scene.add(pyrender.DirectionalLight(color=[1, 1, 1], intensity=3.0), pose=cam_pose)
            try:
                color, _ = renderer.render(scene)
            except Exception as e:
                logger.warning("render failed mid-loop: %s", e)
                renderer.delete()
                return None
            images.append(color)
            poses.append(cam_pose)
    finally:
        renderer.delete()
    return images, np.stack(poses, axis=0)


def _angle_from_filename(name: str) -> Optional[int]:
    m = re.match(r"(bottom_|top_)?(-?\d+)deg", name)
    if not m:
        return None
    prefix, deg = m.group(1) or "", int(m.group(2))
    if prefix == "top_": deg += 360
    elif prefix == "bottom_": deg -= 360
    return deg


def load_reference_images(ref_dir: str | Path) -> Tuple[List[np.ndarray], List[int]]:
    """기존 reference_images/images/ 의 실촬 RGB 이미지를 로드. PyRender 폴백용."""
    ref_dir = Path(ref_dir)
    imgs_dir = ref_dir / "images"
    if not imgs_dir.exists():
        return [], []
    rgb_files = sorted([p for p in imgs_dir.glob("*.png") if "depth" not in p.stem])
    images, deg_list = [], []
    for p in rgb_files:
        deg = _angle_from_filename(p.stem)
        if deg is None:
            continue
        img = cv2.imread(str(p))
        if img is None:
            continue
        images.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        deg_list.append(deg)
    return images, deg_list


def build_templates_for_class(class_id: int, class_name: str, mesh_path: str,
                              ref_dir: str | Path,
                              encoder_name: str = "dinov3-vitl16",
                              n_views: int = 42,
                              img_size: int = 224,
                              device: str = "cuda:0") -> Tuple[np.ndarray, np.ndarray, str]:
    """
    Returns (feats (N, D), poses (N, 4, 4), encoder_actual_name).
    """
    enc = build_encoder(encoder_name, device=device)

    # 1) PyRender 로 42뷰 합성 시도
    rendered = render_icosphere_views(mesh_path, n_views=n_views, img_size=img_size)
    if rendered is not None:
        images, poses = rendered
        logger.info("Class %s: %d rendered views via pyrender", class_name, len(images))
    else:
        # 2) Fallback — 실촬 reference_images
        images, _ = load_reference_images(ref_dir)
        if not images:
            raise RuntimeError(
                f"No renderer and no reference images for class {class_name} at {ref_dir}")
        logger.warning("Class %s: %d reference images used (no pyrender)",
                       class_name, len(images))
        # 임의 pose 할당 (yaw 기준 회전) — 추후 FP init 힌트로만 사용되므로 근사
        yaws = np.linspace(0, 2 * np.pi, num=len(images), endpoint=False)
        poses = []
        for y in yaws:
            R = np.array([[np.cos(y), -np.sin(y), 0],
                          [np.sin(y),  np.cos(y), 0],
                          [0, 0, 1]])
            P = np.eye(4); P[:3, :3] = R; P[2, 3] = 0.5
            poses.append(P)
        poses = np.stack(poses, axis=0)

    feats = enc.encode(images)
    return feats.astype(np.float32), poses.astype(np.float64), enc.name
