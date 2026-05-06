"""
2D Tracker — DAM4SAM / HiM2SAM / SAMURAI / Cutie / OSTrack / IoU fallback.

architecture §4.2 에 대응. 위 모델 중 하나의 체크포인트가 있으면 그것을, 아니면
경량 IoU-based tracker 로 폴백. IoU fallback 은 template matching 으로 bbox 를
갱신한다 (SAM 없이도 수 ms 내 동작).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from ..common import BBox

logger = logging.getLogger(__name__)


@dataclass
class Tracker2DResult:
    bbox: BBox
    conf: float
    source: str


class IoUTracker2D:
    """경량 fallback — template matching 으로 bbox 를 갱신.

    OpenCV cv2.TrackerCSRT_create() 가 있으면 그것을 사용. 없으면 순수
    템플릿-NCC matching 으로 제자리 이동 정도만 추적.
    """

    def __init__(self):
        self.cv_tracker = None
        self.initialized = False
        self.last_bbox: Optional[BBox] = None
        self.template: Optional[np.ndarray] = None
        self.source = "iou_opencv"
        try:
            # opencv-contrib 이 있으면 CSRT 사용
            self.cv_tracker_factory = cv2.TrackerCSRT_create
        except AttributeError:
            self.cv_tracker_factory = None
            self.source = "iou_template"

    def reset(self, frame_rgb: np.ndarray, bbox: BBox):
        H, W = frame_rgb.shape[:2]
        x1, y1 = max(0, bbox.x1), max(0, bbox.y1)
        x2, y2 = min(W, bbox.x2), min(H, bbox.y2)
        self.last_bbox = BBox(x1, y1, x2, y2, conf=1.0)
        if self.cv_tracker_factory is not None:
            try:
                self.cv_tracker = self.cv_tracker_factory()
                self.cv_tracker.init(frame_rgb, (x1, y1, x2 - x1, y2 - y1))
                self.source = "csrt"
            except Exception as e:
                logger.warning("CSRT init failed: %s", e)
                self.cv_tracker = None
                self.template = frame_rgb[y1:y2, x1:x2].copy()
                self.source = "iou_template"
        else:
            self.template = frame_rgb[y1:y2, x1:x2].copy()
        self.initialized = True

    def update(self, frame_rgb: np.ndarray) -> Optional[Tracker2DResult]:
        if not self.initialized or self.last_bbox is None:
            return None
        H, W = frame_rgb.shape[:2]
        if self.cv_tracker is not None:
            ok, box = self.cv_tracker.update(frame_rgb)
            if not ok:
                return None
            x, y, w, h = box
            bb = BBox(int(x), int(y), int(x + w), int(y + h), conf=0.9)
            bb.x1 = max(0, bb.x1); bb.y1 = max(0, bb.y1)
            bb.x2 = min(W, bb.x2); bb.y2 = min(H, bb.y2)
            self.last_bbox = bb
            return Tracker2DResult(bb, 0.9, self.source)
        # template matching fallback
        if self.template is None or self.template.size == 0:
            return None
        # search window: 1.5× bbox 주변
        cx, cy = self.last_bbox.cx, self.last_bbox.cy
        th, tw = self.template.shape[:2]
        sx1 = max(0, int(cx - 1.5 * tw)); sx2 = min(W, int(cx + 1.5 * tw))
        sy1 = max(0, int(cy - 1.5 * th)); sy2 = min(H, int(cy + 1.5 * th))
        if sx2 - sx1 < tw + 2 or sy2 - sy1 < th + 2:
            return None
        search = frame_rgb[sy1:sy2, sx1:sx2]
        res = cv2.matchTemplate(search, self.template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)
        if max_val < 0.5:
            return None
        top_left = (sx1 + max_loc[0], sy1 + max_loc[1])
        bb = BBox(top_left[0], top_left[1],
                  top_left[0] + tw, top_left[1] + th, conf=float(max_val))
        self.last_bbox = bb
        return Tracker2DResult(bb, float(max_val), self.source)


def _try_sam_tracker(ckpt_path: str):
    """DAM4SAM/HiM2SAM/SAMURAI 체크포인트 자동 검출 (없으면 None)."""
    if not ckpt_path or not Path(ckpt_path).exists():
        return None
    # 실제 wrapper 구현은 외부 repo 가 필요하므로 Phase 4 후반에 추가.
    # 현재는 IoU fallback 고정.
    logger.info("SAM-based 2D tracker ckpt found but wrapper not implemented; using IoU")
    return None


def build_tracker_2d(preferred: str = "dam4sam", ckpt_path: str = ""):
    sam_tr = _try_sam_tracker(ckpt_path)
    if sam_tr is not None:
        return sam_tr
    return IoUTracker2D()
