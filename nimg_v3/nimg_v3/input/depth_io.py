"""
Rank 1.B — Raw 16-bit depth I/O.

설계: research/260420_fp_top5_implementation_design.md §2.3.

지원 포맷:
- `depth_dir/frame_<6digit>.png` — uint16 mm 단위, RGB MP4 와 frame_idx 1:1 매칭
- `depth_<tag>.npz` — `arr_0` shape (N, H, W) uint16, 옵션 `K` shape (3, 3) float64

기존 PairedVideoSource (BGR-MP4 lossy) 의 alternative 채널.
"""
from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from ..common import Frame
from ..config.system_config import CameraConfig
from .adapters import InputAdapter

logger = logging.getLogger(__name__)


class RawDepthReader(InputAdapter):
    """16-bit PNG sequence 또는 NPZ 의 raw depth 를 RGB MP4 와 동기화."""

    def __init__(
        self,
        rgb_video: str | Path,
        depth_source: str | Path,           # 디렉터리(*.png) 또는 *.npz
        K_path: Optional[str | Path] = None,
        camera_cfg: Optional[CameraConfig] = None,
        loop: bool = False,
        depth_scale: float = 0.001,         # uint16 → m (mm/1000)
    ):
        self.rgb_video = str(rgb_video)
        self.depth_source = Path(depth_source)
        self.K_path = Path(K_path) if K_path else None
        self.cfg = camera_cfg or CameraConfig()
        self.loop = loop
        self.depth_scale = depth_scale
        self.cap_rgb: Optional[cv2.VideoCapture] = None
        self._npz_arr: Optional[np.ndarray] = None
        self._png_files: list[Path] = []
        self._idx = 0
        self._total = 0

    def start(self) -> bool:
        self.cap_rgb = cv2.VideoCapture(self.rgb_video)
        if not self.cap_rgb.isOpened():
            logger.error("RGB video open failed: %s", self.rgb_video)
            return False
        n_rgb = int(self.cap_rgb.get(cv2.CAP_PROP_FRAME_COUNT))
        w = int(self.cap_rgb.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap_rgb.get(cv2.CAP_PROP_FRAME_HEIGHT))
        native_fps = float(self.cap_rgb.get(cv2.CAP_PROP_FPS) or 30.0)

        # depth source: NPZ vs PNG dir
        if self.depth_source.is_file() and self.depth_source.suffix.lower() == ".npz":
            data = np.load(str(self.depth_source))
            arr = data["arr_0"] if "arr_0" in data.files else data[data.files[0]]
            self._npz_arr = arr.astype(np.uint16)
            self._total = min(n_rgb, len(self._npz_arr))
            logger.info("RawDepthReader (NPZ): %d frames, depth shape=%s",
                        self._total, arr.shape)
        elif self.depth_source.is_dir():
            self._png_files = sorted(
                p for p in self.depth_source.iterdir()
                if p.suffix.lower() in (".png", ".tiff", ".tif")
            )
            self._total = min(n_rgb, len(self._png_files))
            logger.info("RawDepthReader (PNG dir): %d frames in %s",
                        self._total, self.depth_source)
        else:
            logger.error("Unsupported depth source: %s", self.depth_source)
            return False

        # Per-recording K (Rank 5)
        if self.K_path and self.K_path.exists():
            with open(self.K_path) as f:
                meta = json.load(f)
            self.cfg = CameraConfig(
                fx=float(meta.get("fx", self.cfg.fx)),
                fy=float(meta.get("fy", self.cfg.fy)),
                cx=float(meta.get("cx", self.cfg.cx)),
                cy=float(meta.get("cy", self.cfg.cy)),
                width=int(meta.get("width", w)),
                height=int(meta.get("height", h)),
                fps=float(meta.get("fps", native_fps)),
                depth_scale=float(meta.get("depth_scale", self.depth_scale)),
                depth_min=self.cfg.depth_min, depth_max=self.cfg.depth_max,
            )
            self.depth_scale = self.cfg.depth_scale
            logger.info("RawDepthReader: per-recording K loaded from %s",
                        self.K_path)
        else:
            self.cfg = CameraConfig(
                fx=self.cfg.fx, fy=self.cfg.fy, cx=self.cfg.cx, cy=self.cfg.cy,
                width=w, height=h, fps=native_fps,
                depth_scale=self.cfg.depth_scale,
                depth_min=self.cfg.depth_min, depth_max=self.cfg.depth_max,
            )
        return True

    def read(self) -> Optional[Frame]:
        if self.cap_rgb is None or self._idx >= self._total:
            if not (self.loop and self._total > 0):
                return None
            self.cap_rgb.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self._idx = 0

        ok, rgb = self.cap_rgb.read()
        if not ok:
            return None

        if self._npz_arr is not None:
            depth_raw = self._npz_arr[self._idx]
        else:
            depth_raw = cv2.imread(str(self._png_files[self._idx]), cv2.IMREAD_UNCHANGED)
            if depth_raw is None:
                logger.warning("PNG read fail at idx=%d", self._idx)
                return None
            if depth_raw.ndim == 3:
                depth_raw = cv2.cvtColor(depth_raw, cv2.COLOR_BGR2GRAY)
        depth_m = depth_raw.astype(np.float32) * self.depth_scale

        ts = self._idx / float(self.cfg.fps if self.cfg.fps > 0 else 30.0)
        fr = Frame(rgb=rgb, depth=depth_m, K=self.get_K(),
                   timestamp=ts, frame_idx=self._idx)
        self._idx += 1
        return fr

    def stop(self) -> None:
        if self.cap_rgb:
            self.cap_rgb.release()
        self.cap_rgb = None
        self._npz_arr = None
        self._png_files = []

    def get_K(self) -> np.ndarray:
        return np.array([[self.cfg.fx, 0, self.cfg.cx],
                         [0, self.cfg.fy, self.cfg.cy],
                         [0, 0, 1]], dtype=np.float64)

    @property
    def total_frames(self) -> int:
        return self._total
