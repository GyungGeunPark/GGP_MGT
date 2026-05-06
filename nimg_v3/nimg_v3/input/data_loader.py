"""
data_loader.py - 데이터 로더

RGB, Depth, IMU 데이터를 통합 로드합니다.
nimg_v2의 DataLoader를 기반으로 확장하였습니다.

Version: 1.0
Author: FurSys AI Team
"""

import os
import cv2
import numpy as np
import pandas as pd
from dataclasses import dataclass
from typing import Optional, Tuple, List, Iterator
from pathlib import Path
import logging
import re

logger = logging.getLogger(__name__)


@dataclass
class FrameData:
    """단일 프레임의 모든 데이터"""
    frame_idx: int
    rgb: np.ndarray
    depth: np.ndarray
    timestamp: float
    accel: Optional[np.ndarray] = None
    gyro: Optional[np.ndarray] = None
    rgb_path: Optional[str] = None
    depth_path: Optional[str] = None

    @property
    def has_imu(self) -> bool:
        return self.accel is not None and self.gyro is not None

    @property
    def depth_valid_ratio(self) -> float:
        if self.depth is None:
            return 0.0
        valid = (self.depth > 0.1) & (self.depth < 10.0)
        return np.sum(valid) / self.depth.size


class DataLoader:
    """
    RGB, Depth, IMU 데이터 통합 로더

    다양한 폴더 구조를 지원합니다:
    1. 기존 nimg_v2 구조 (color_*, depth_*)
    2. 참조 이미지 구조 (rgb/, depth/)

    Example:
        >>> loader = DataLoader("./20251208_155531_output")
        >>> for frame in loader:
        ...     process(frame)
    """

    def __init__(
        self,
        data_dir: str,
        depth_scale: float = 0.001,
        fps: float = 30.0,
        depth_min: float = 0.1,
        depth_max: float = 10.0
    ):
        self.data_dir = Path(data_dir)
        self.depth_scale = depth_scale
        self.fps = fps
        self.depth_min = depth_min
        self.depth_max = depth_max

        if not self.data_dir.exists():
            raise FileNotFoundError(f"Data directory not found: {data_dir}")

        # 폴더 구조 감지 및 파일 목록 로드
        self._detect_structure()
        self._load_file_lists()
        self._load_imu_data()

        logger.info(f"DataLoader: {self.num_frames} frames, structure={self._structure}")

    def _detect_structure(self):
        """폴더 구조 감지"""
        # 구조 1: rgb/ 및 depth/ 하위 폴더
        if (self.data_dir / 'rgb').exists() and (self.data_dir / 'depth').exists():
            self._structure = 'subfolder'
            self._rgb_dir = self.data_dir / 'rgb'
            self._depth_dir = self.data_dir / 'depth'
        # 구조 2: 같은 폴더에 color_*, depth_*
        else:
            self._structure = 'flat'
            self._rgb_dir = self.data_dir
            self._depth_dir = self.data_dir

    def _load_file_lists(self):
        """파일 목록 로드"""
        if self._structure == 'subfolder':
            self.color_files = sorted([
                f.name for f in self._rgb_dir.iterdir()
                if f.suffix.lower() in ['.png', '.jpg', '.jpeg']
            ])
            self.depth_files = sorted([
                f.name for f in self._depth_dir.iterdir()
                if f.suffix.lower() == '.png'
            ])
        else:
            all_files = list(self.data_dir.iterdir())
            self.color_files = sorted([
                f.name for f in all_files
                if f.name.startswith('color_') and f.suffix.lower() in ['.png', '.jpg', '.jpeg']
            ])
            self.depth_files = sorted([
                f.name for f in all_files
                if f.name.startswith('depth_') and f.suffix.lower() == '.png'
            ])

        if len(self.color_files) == 0:
            raise ValueError(f"No color images found in {self._rgb_dir}")
        if len(self.depth_files) == 0:
            raise ValueError(f"No depth images found in {self._depth_dir}")

        self.num_frames = min(len(self.color_files), len(self.depth_files))

    def _load_imu_data(self):
        """IMU 데이터 로드"""
        imu_path = self.data_dir / 'imu_data.csv'

        if imu_path.exists():
            try:
                self.imu_df = pd.read_csv(imu_path)
                self.has_imu = True
                self.accel_df = self.imu_df[self.imu_df['type'] == 'accel'].copy()
                self.gyro_df = self.imu_df[self.imu_df['type'] == 'gyro'].copy()
            except Exception as e:
                logger.warning(f"Failed to load IMU: {e}")
                self.imu_df = None
                self.has_imu = False
        else:
            self.imu_df = None
            self.has_imu = False

    def __len__(self) -> int:
        return self.num_frames

    def __getitem__(self, idx: int) -> FrameData:
        return self.load_frame(idx)

    def __iter__(self) -> Iterator[FrameData]:
        for idx in range(self.num_frames):
            yield self.load_frame(idx)

    def load_frame(self, idx: int) -> FrameData:
        """프레임 로드"""
        if idx < 0 or idx >= self.num_frames:
            raise IndexError(f"Frame index {idx} out of range")

        rgb_path = self._rgb_dir / self.color_files[idx]
        rgb = cv2.imread(str(rgb_path))
        if rgb is None:
            raise IOError(f"Failed to load RGB: {rgb_path}")

        depth_path = self._depth_dir / self.depth_files[idx]
        depth_raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if depth_raw is None:
            raise IOError(f"Failed to load depth: {depth_path}")

        depth = depth_raw.astype(np.float32) * self.depth_scale
        depth[(depth < self.depth_min) | (depth > self.depth_max)] = 0.0

        timestamp = idx / self.fps

        accel, gyro = None, None
        if self.has_imu:
            accel, gyro = self._get_imu_for_frame(idx)

        return FrameData(
            frame_idx=idx,
            rgb=rgb,
            depth=depth,
            timestamp=timestamp,
            accel=accel,
            gyro=gyro,
            rgb_path=str(rgb_path),
            depth_path=str(depth_path)
        )

    def _get_imu_for_frame(self, idx: int) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """프레임에 해당하는 IMU 데이터"""
        if not self.has_imu:
            return None, None

        t_start = idx / self.fps
        t_end = (idx + 1) / self.fps

        # 가속도 데이터
        if 'rel_time' in self.accel_df.columns:
            accel_mask = (self.accel_df['rel_time'] >= t_start) & (self.accel_df['rel_time'] < t_end)
        else:
            accel_mask = pd.Series([True] * len(self.accel_df))

        accel_slice = self.accel_df[accel_mask]
        if len(accel_slice) > 0:
            accel = np.array([accel_slice['x'].mean(), accel_slice['y'].mean(), accel_slice['z'].mean()])
        else:
            accel = np.array([self.accel_df['x'].mean(), self.accel_df['y'].mean(), self.accel_df['z'].mean()])

        # 자이로 데이터
        if 'rel_time' in self.gyro_df.columns:
            gyro_mask = (self.gyro_df['rel_time'] >= t_start) & (self.gyro_df['rel_time'] < t_end)
        else:
            gyro_mask = pd.Series([True] * len(self.gyro_df))

        gyro_slice = self.gyro_df[gyro_mask]
        if len(gyro_slice) > 0:
            gyro = np.array([gyro_slice['x'].mean(), gyro_slice['y'].mean(), gyro_slice['z'].mean()])
        else:
            gyro = np.array([self.gyro_df['x'].mean(), self.gyro_df['y'].mean(), self.gyro_df['z'].mean()])

        return accel, gyro

    def get_image_size(self) -> Tuple[int, int]:
        """이미지 크기 (height, width)"""
        rgb_path = self._rgb_dir / self.color_files[0]
        rgb = cv2.imread(str(rgb_path))
        return rgb.shape[:2]
