"""
input 모듈 - 데이터 입력 처리

RealSense 카메라 및 오프라인 데이터 로드를 지원합니다.
"""

from .data_loader import DataLoader, FrameData
from .adapters import (
    InputAdapter, PairedVideoSource, RealSenseSource,
    discover_video_pairs, decode_depth_bgr,
)
from .depth_io import RawDepthReader
from .realsense_filters import RealSenseDepthFilters

__all__ = [
    'DataLoader', 'FrameData',
    'InputAdapter', 'PairedVideoSource', 'RealSenseSource',
    'RawDepthReader', 'RealSenseDepthFilters',
    'discover_video_pairs', 'decode_depth_bgr',
]
