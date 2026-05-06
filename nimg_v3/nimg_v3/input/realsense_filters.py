"""
Rank 1 — librealsense 후처리 체인.

설계: research/260420_fp_top5_implementation_design.md §2.3.

`RealSenseSource.read()` 안에서 align 후 raw depth_frame 에 적용.
없는 환경에서도 동작 — pyrealsense2 import 실패 시 no-op.
"""
from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


class RealSenseDepthFilters:
    """librealsense 의 표준 후처리 체인.

    chain: [decimation? → spatial → temporal → hole_filling]
    Intel 권장: 30fps live 에서 spatial(2pass) + temporal + hole_filling 이 sweet spot.
    """

    def __init__(self,
                 enable: bool = True,
                 spatial_iter: int = 2,
                 spatial_alpha: float = 0.5,
                 spatial_delta: int = 20,
                 temporal_alpha: float = 0.4,
                 temporal_delta: int = 20,
                 hole_filling_mode: int = 1,    # 1 = farest_from_around
                 disable_decimation: bool = True):
        self.enabled = enable
        try:
            import pyrealsense2 as rs
            self._rs = rs
        except ImportError:
            self._rs = None
            self.enabled = False
            return
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, float(spatial_iter))
        self.spatial.set_option(rs.option.filter_smooth_alpha, float(spatial_alpha))
        self.spatial.set_option(rs.option.filter_smooth_delta, float(spatial_delta))

        self.temporal = rs.temporal_filter()
        self.temporal.set_option(rs.option.filter_smooth_alpha, float(temporal_alpha))
        self.temporal.set_option(rs.option.filter_smooth_delta, float(temporal_delta))

        self.hole = rs.hole_filling_filter()
        self.hole.set_option(rs.option.holes_fill, float(hole_filling_mode))

        if not disable_decimation:
            self.decim = rs.decimation_filter()
        else:
            self.decim = None

    def apply(self, depth_frame):
        """rs.depth_frame → 필터 적용된 rs.depth_frame.

        실패 시 원본을 반환.
        """
        if not self.enabled:
            return depth_frame
        try:
            f = depth_frame
            if self.decim is not None:
                f = self.decim.process(f)
            f = self.spatial.process(f)
            f = self.temporal.process(f)
            f = self.hole.process(f)
            return f
        except Exception as e:
            logger.debug("RealSense filters failed (%s); returning raw frame", e)
            return depth_frame
