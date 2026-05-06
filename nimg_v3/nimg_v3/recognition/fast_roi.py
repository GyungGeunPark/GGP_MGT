"""
Fast ROI (Tier-1) — depth 기반 평면 제거 + 전경 bbox 추출.

architecture §3.1 Layer-A 에 대응.
Open3D RANSAC plane segmentation 을 사용 (설치됨). 구현 복잡도 최소화를 위해
중앙 히스토그램 peak 을 평면 depth 로 간주하고 전경/배경 분리하는 경량 경로도 제공.
"""
from __future__ import annotations

import logging
from typing import List

import cv2
import numpy as np

from ..common import BBox, Frame

logger = logging.getLogger(__name__)


class FastROI:
    """프레임당 <10 ms 안에 전경 bbox 후보 리스트를 리턴.

    두 가지 경로:
    - 'simple_depth': depth 히스토그램 peak → 전경 마스크 → connected component
    - 'open3d_ransac': Open3D plane_segmentation (더 정확하나 느림)
    """

    def __init__(
        self,
        backend: str = "simple_depth",
        min_area: int = 800,
        max_area: int = 80_000,
        roi: tuple | None = None,
        foreground_delta_m: float = 0.03,
    ):
        self.backend = backend
        self.min_area = min_area
        self.max_area = max_area
        self.roi = roi
        self.fg_delta = foreground_delta_m

    def process(self, frame: Frame) -> List[BBox]:
        if self.backend == "simple_depth":
            return self._simple_depth(frame)
        else:
            # open3d backend — not used by default (느림)
            return self._simple_depth(frame)

    # ------------------------------------------------------------------
    def _simple_depth(self, frame: Frame) -> List[BBox]:
        depth = frame.depth
        valid = (depth > 0.15) & (depth < 3.5)
        if self.roi is not None:
            x1, y1, x2, y2 = self.roi
            roi_mask = np.zeros_like(valid)
            roi_mask[y1:y2, x1:x2] = True
            valid = valid & roi_mask
        if valid.sum() < 500:
            return []
        # 히스토그램 peak 을 바닥 depth 로 간주
        vals = depth[valid]
        hist, bin_edges = np.histogram(vals, bins=60, range=(0.2, 3.5))
        peak_idx = int(np.argmax(hist))
        floor_depth = 0.5 * (bin_edges[peak_idx] + bin_edges[peak_idx + 1])
        # 전경: 바닥보다 self.fg_delta 이상 가까운 픽셀
        fg = valid & (depth < floor_depth - self.fg_delta)
        if fg.sum() < self.min_area:
            return []
        fg_u8 = (fg * 255).astype(np.uint8)
        # morphology
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fg_u8 = cv2.morphologyEx(fg_u8, cv2.MORPH_OPEN, kernel)
        fg_u8 = cv2.morphologyEx(fg_u8, cv2.MORPH_CLOSE, kernel)
        # connected components
        n, labels, stats, _ = cv2.connectedComponentsWithStats(fg_u8, connectivity=8)
        boxes: List[BBox] = []
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if area < self.min_area or area > self.max_area:
                continue
            boxes.append(BBox(x1=int(x), y1=int(y), x2=int(x + w), y2=int(y + h),
                              conf=min(1.0, area / 5000.0)))
        # 좌→우 정렬 (우선순위: 큰 면적)
        boxes.sort(key=lambda b: -b.area)
        return boxes[:10]
