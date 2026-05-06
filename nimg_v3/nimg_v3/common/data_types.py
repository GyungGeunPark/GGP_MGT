"""
공통 데이터 타입: Frame / BBox / RecognizedObject / PoseHypothesis / MeasurementResult.

260420_mesh_sam_foundationpose_architecture_update.md §3.3 와 1:1 대응.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class BBox:
    x1: int
    y1: int
    x2: int
    y2: int
    conf: float = 1.0

    @property
    def cx(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def cy(self) -> float:
        return 0.5 * (self.y1 + self.y2)

    @property
    def w(self) -> int:
        return max(0, self.x2 - self.x1)

    @property
    def h(self) -> int:
        return max(0, self.y2 - self.y1)

    @property
    def area(self) -> int:
        return self.w * self.h

    def iou(self, other: "BBox") -> float:
        ix1, iy1 = max(self.x1, other.x1), max(self.y1, other.y1)
        ix2, iy2 = min(self.x2, other.x2), min(self.y2, other.y2)
        iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
        inter = iw * ih
        u = self.area + other.area - inter
        return inter / u if u > 0 else 0.0


@dataclass
class Frame:
    rgb: np.ndarray               # (H, W, 3) uint8 BGR
    depth: np.ndarray             # (H, W)    float32 meter
    K: np.ndarray                 # (3, 3)
    timestamp: float = field(default_factory=time.time)
    frame_idx: int = 0


@dataclass
class RecognizedObject:
    object_uid: int
    class_id: int
    bbox: BBox
    mask: Optional[np.ndarray] = None   # (H, W) bool, may be None when recognizer couldn't produce one
    ref_view_idx: int = -1              # DINOv3 top-1 뷰 index (coarse pose 힌트)
    match_score: float = 0.0            # DINOv3 cosine similarity
    source: str = "unknown"             # "noctis" | "cnos" | "yolo" | "depth_bbox"


@dataclass
class PoseHypothesis:
    T: np.ndarray                 # (4, 4) 카메라 좌표계
    score: float = 0.0
    source: str = "init"          # "init" | "track" | "reinit"
    timestamp: float = 0.0
    velocity: Optional[np.ndarray] = None          # (3,) m/s   world/camera frame
    angular_velocity: Optional[np.ndarray] = None  # (3,) rad/s


@dataclass
class MeasurementResult:
    frame_idx: int
    timestamp: float
    class_id: int
    object_uid: int
    T: np.ndarray                # (4, 4)
    score: float
    relative_T: np.ndarray       # (4, 4) baseline-relative
    relative_yaw_deg: float
    relative_pitch_deg: float
    relative_roll_deg: float
    signal: int                  # -2..+2, 99 for 'none'
    linear_velocity_ms: float = 0.0
    angular_velocity_rads: float = 0.0
    latency_A_ms: float = 0.0
    latency_B_ms: float = 0.0
    latency_C_ms: float = 0.0
    latency_D_ms: float = 0.0
    source: str = "track"        # "init" | "track" | "reinit" | "lost"
