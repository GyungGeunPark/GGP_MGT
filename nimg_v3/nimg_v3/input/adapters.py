"""
Input Adapter — 단일 `Frame` 인터페이스로 RealSense/Video 소스를 동일하게 공급.

architecture §6.1 에 대응.
"""
from __future__ import annotations

import logging
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from ..common import Frame
from ..config.system_config import CameraConfig

logger = logging.getLogger(__name__)


class InputAdapter(ABC):
    """공통 인터페이스. 모든 소스가 이 형태로 `Frame` 을 공급한다."""

    @abstractmethod
    def start(self) -> bool: ...

    @abstractmethod
    def read(self) -> Optional[Frame]: ...

    @abstractmethod
    def stop(self) -> None: ...

    @abstractmethod
    def get_K(self) -> np.ndarray: ...

    @property
    @abstractmethod
    def total_frames(self) -> int: ...


# ======================================================================
# Depth 디코더: /video/ 폴더의 MP4 는 3채널 BGR 로 저장되어 있음 (uint8).
# 기존 realsense_6dof_stream_video.py 의 `_convert_depth` 와 호환되도록
# BGR→grayscale → 0-5m 선형 매핑을 사용한다.
# ======================================================================

def decode_depth_bgr(depth_bgr: np.ndarray, max_range_m: float = 5.0,
                     denoise: bool = False) -> np.ndarray:
    """3채널 uint8 depth 비디오 프레임 → float32 meter.

    실제 D455 원본은 uint16 mm 이지만 MP4 인코딩 과정에서 gray(0–255) 로 압축되어
    있다. 기존 파이프라인과 동일한 선형 매핑을 사용해 호환성을 유지한다.

    Args:
        denoise: True 시 cv2.bilateralFilter (d=5, sigmaColor=0.05, sigmaSpace=5)
                 적용 — 양자화 노이즈 부분 완화 (Rank 1.A best-effort).
    """
    if depth_bgr.ndim == 3:
        gray = cv2.cvtColor(depth_bgr, cv2.COLOR_BGR2GRAY)
    else:
        gray = depth_bgr
    depth_m = gray.astype(np.float32) * (max_range_m / 255.0)
    if denoise:
        # 0 픽셀 보존 (depth 측정 무효 표시) 위해 마스킹
        valid = depth_m > 0.001
        smoothed = cv2.bilateralFilter(depth_m, d=5,
                                        sigmaColor=0.05, sigmaSpace=5)
        depth_m = np.where(valid, smoothed, depth_m)
    return depth_m


class PairedVideoSource(InputAdapter):
    """RGB.mp4 + Depth.mp4 쌍을 읽어서 동기화된 Frame 을 공급."""

    def __init__(
        self,
        rgb_path: str | Path,
        depth_path: str | Path,
        camera_cfg: Optional[CameraConfig] = None,
        loop: bool = False,
        max_range_m: float = 5.0,
        depth_denoise: bool = False,
    ):
        self.rgb_path = str(rgb_path)
        self.depth_path = str(depth_path)
        self.loop = loop
        self.max_range_m = max_range_m
        self.depth_denoise = depth_denoise
        self.cfg = camera_cfg or CameraConfig()
        self.cap_rgb: Optional[cv2.VideoCapture] = None
        self.cap_depth: Optional[cv2.VideoCapture] = None
        self._total = 0
        self._idx = 0

    def start(self) -> bool:
        self.cap_rgb = cv2.VideoCapture(self.rgb_path)
        self.cap_depth = cv2.VideoCapture(self.depth_path)
        if not self.cap_rgb.isOpened() or not self.cap_depth.isOpened():
            logger.error("Failed to open video(s): %s / %s", self.rgb_path, self.depth_path)
            return False
        self._total = int(self.cap_rgb.get(cv2.CAP_PROP_FRAME_COUNT))
        # 해상도가 config 와 다르면 K 를 비례 조정
        w = int(self.cap_rgb.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap_rgb.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # 비디오 파일의 native fps 를 우선 사용 (타임스탬프·viz writer 일관성)
        native_fps = float(self.cap_rgb.get(cv2.CAP_PROP_FPS) or 0.0)
        new_fps = native_fps if native_fps > 0 else self.cfg.fps
        if w != self.cfg.width or h != self.cfg.height or new_fps != self.cfg.fps:
            if w != self.cfg.width or h != self.cfg.height:
                sx, sy = w / self.cfg.width, h / self.cfg.height
                new_fx, new_fy = self.cfg.fx * sx, self.cfg.fy * sy
                new_cx, new_cy = self.cfg.cx * sx, self.cfg.cy * sy
            else:
                new_fx, new_fy = self.cfg.fx, self.cfg.fy
                new_cx, new_cy = self.cfg.cx, self.cfg.cy
            self.cfg = CameraConfig(
                fx=new_fx, fy=new_fy, cx=new_cx, cy=new_cy,
                width=w, height=h, fps=new_fps,
                depth_scale=self.cfg.depth_scale,
                depth_min=self.cfg.depth_min, depth_max=self.cfg.depth_max,
            )
        logger.info("PairedVideoSource: %dx%d @ %.1f fps, %d frames",
                    w, h, self.cfg.fps, self._total)
        return True

    def read(self) -> Optional[Frame]:
        if self.cap_rgb is None:
            return None
        ok1, rgb = self.cap_rgb.read()
        ok2, depth_raw = self.cap_depth.read()
        if not (ok1 and ok2):
            if self.loop:
                self.cap_rgb.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.cap_depth.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self._idx = 0
                ok1, rgb = self.cap_rgb.read()
                ok2, depth_raw = self.cap_depth.read()
                if not (ok1 and ok2):
                    return None
            else:
                return None
        depth_m = decode_depth_bgr(depth_raw, self.max_range_m,
                                    denoise=self.depth_denoise)
        # timestamp: wall-clock 대신 **비디오 시간** (frame_idx / fps) 사용.
        # 그래야 KF 의 dt 가 실제 프레임 간격을 반영 → 속도/각속도 값이 현실적.
        ts = self._idx / float(self.cfg.fps if self.cfg.fps > 0 else 30.0)
        fr = Frame(rgb=rgb, depth=depth_m, K=self.get_K(),
                   timestamp=ts, frame_idx=self._idx)
        self._idx += 1
        return fr

    def stop(self) -> None:
        if self.cap_rgb: self.cap_rgb.release()
        if self.cap_depth: self.cap_depth.release()
        self.cap_rgb = self.cap_depth = None

    def get_K(self) -> np.ndarray:
        return np.array([
            [self.cfg.fx, 0, self.cfg.cx],
            [0, self.cfg.fy, self.cfg.cy],
            [0, 0, 1]
        ], dtype=np.float64)

    @property
    def total_frames(self) -> int:
        return self._total


class RealSenseSource(InputAdapter):
    """Intel RealSense D455 실시간 input adapter (aligned RGB + Depth).

    `pyrealsense2` 가 없거나 카메라 미연결 시 `start()` 가 False.
    """

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30,
                 enable_filters: bool = True):
        self.width = width; self.height = height; self.fps = fps
        self.pipeline = None
        self.align = None
        self._K: Optional[np.ndarray] = None
        self._idx = 0
        try:
            import pyrealsense2 as rs    # noqa
            self._rs = rs
        except ImportError:
            logger.error("pyrealsense2 not available in current env")
            self._rs = None
        # Rank 1 — librealsense 후처리 (depth jitter 완화)
        self._filters = None
        if enable_filters and self._rs is not None:
            from .realsense_filters import RealSenseDepthFilters
            self._filters = RealSenseDepthFilters(enable=True)

    def start(self) -> bool:
        if self._rs is None:
            return False
        rs = self._rs
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, self.width, self.height,
                                 rs.format.bgr8, self.fps)
            config.enable_stream(rs.stream.depth, self.width, self.height,
                                 rs.format.z16, self.fps)
            prof = self.pipeline.start(config)
            # depth → color 정렬
            self.align = rs.align(rs.stream.color)
            color_stream = prof.get_stream(rs.stream.color).as_video_stream_profile()
            intr = color_stream.get_intrinsics()
            self._K = np.array([[intr.fx, 0, intr.ppx],
                                [0, intr.fy, intr.ppy],
                                [0, 0, 1]], dtype=np.float64)
            self.depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()
            logger.info("RealSense started: %dx%d @ %dfps, K=fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                        self.width, self.height, self.fps,
                        intr.fx, intr.fy, intr.ppx, intr.ppy)
            return True
        except Exception as e:
            logger.error("RealSense start failed: %s", e)
            return False

    def read(self) -> Optional[Frame]:
        if self.pipeline is None:
            return None
        frames = self.pipeline.wait_for_frames(2000)
        frames = self.align.process(frames)
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            return None
        # Rank 1: librealsense 후처리 (spatial+temporal+hole_filling)
        if self._filters is not None:
            depth = self._filters.apply(depth)
        rgb = np.asanyarray(color.get_data())
        depth_raw = np.asanyarray(depth.get_data()).astype(np.float32)
        depth_m = depth_raw * self.depth_scale
        fr = Frame(rgb=rgb, depth=depth_m, K=self._K,
                   timestamp=self._idx / float(self.fps), frame_idx=self._idx)
        self._idx += 1
        return fr

    def stop(self) -> None:
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.pipeline = None

    def get_K(self) -> np.ndarray:
        return self._K if self._K is not None else np.array([
            [383.883, 0, 320.499], [0, 383.883, 237.913], [0, 0, 1]
        ], dtype=np.float64)

    @property
    def total_frames(self) -> int:
        return -1       # 실시간 — unbounded


def discover_video_pairs(video_dir: str | Path) -> list[tuple[str, Path, Path]]:
    """
    /video/ 폴더에서 `*_rgb_*.mp4` + `*_depth_*.mp4` 쌍을 자동 발견.

    Returns:
        List of (tag, rgb_path, depth_path). tag 는 "test1" 같은 공통 식별자.
    """
    video_dir = Path(video_dir)
    rgb_files = sorted(video_dir.glob("*rgb*.mp4"))
    pairs = []
    for rgb in rgb_files:
        depth = Path(str(rgb).replace("rgb", "depth"))
        if not depth.exists():
            continue
        # tag: 파일명에서 _rgb_ 이전까지, 맨 앞 output_video_ 제거
        stem = rgb.stem.replace("_rgb_30fps", "").replace("_rgb", "")
        tag = stem.replace("output_video_", "").strip("_")
        pairs.append((tag, rgb, depth))
    return pairs
