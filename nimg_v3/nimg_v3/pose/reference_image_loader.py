"""
reference_image_loader.py - 참조 이미지 로더

Model-Free FoundationPose를 위한 참조 이미지 로딩 및 관리 모듈.

참조 이미지 폴더 구조:
    /root/fursys_img_251229/extraction/
    ├── 20251229_093820_front/     # 정면
    │   ├── rgb/
    │   ├── depth/
    │   └── depth_csv/ (optional)
    ├── 20251229_094115_45/        # 45도
    ├── 20251229_094410_90/        # 90도
    └── ...

각도별 폴더에서 대표 이미지를 추출하여
Neural Object Field 학습에 사용합니다.

Version: 1.0
Author: FurSys AI Team
"""

import os
import cv2
import numpy as np
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict, Any
import logging
import re
from enum import Enum

logger = logging.getLogger(__name__)


class ViewAngle(Enum):
    """카메라 시점 각도"""
    FRONT = 0
    DEG_45 = 45
    DEG_90 = 90
    DEG_135 = 135
    DEG_180 = 180
    DEG_225 = 225
    DEG_270 = 270
    DEG_315 = 315
    TOP_FRONT = "topfront"
    TOP_90 = "top90"
    TOP_270 = "top270"
    BOTTOM_FRONT = "bottomfront"
    BOTTOM_90 = "bottom90"
    BOTTOM_270 = "bottom270"


@dataclass
class ReferenceImage:
    """
    단일 참조 이미지 데이터

    Attributes:
        rgb: RGB 이미지 [H, W, 3]
        depth: Depth 이미지 [H, W] (미터 단위)
        mask: 객체 마스크 [H, W] (binary)
        view_angle: 촬영 각도
        folder_name: 원본 폴더명
        timestamp: 이미지 타임스탬프
    """
    rgb: np.ndarray
    depth: np.ndarray
    mask: Optional[np.ndarray] = None
    view_angle: Optional[str] = None
    folder_name: str = ""
    timestamp: float = 0.0

    @property
    def has_mask(self) -> bool:
        return self.mask is not None

    @property
    def image_size(self) -> Tuple[int, int]:
        """(height, width)"""
        return self.rgb.shape[:2]


@dataclass
class ReferenceImageSet:
    """
    참조 이미지 세트 (Neural Object Field 학습용)

    여러 시점에서 촬영한 참조 이미지들을 관리합니다.
    """
    images: List[ReferenceImage] = field(default_factory=list)
    object_name: str = ""
    camera_intrinsics: Optional[Dict[str, float]] = None

    def __len__(self) -> int:
        return len(self.images)

    def __getitem__(self, idx: int) -> ReferenceImage:
        return self.images[idx]

    def __iter__(self):
        return iter(self.images)

    @property
    def num_views(self) -> int:
        return len(self.images)

    @property
    def view_angles(self) -> List[str]:
        return [img.view_angle for img in self.images if img.view_angle]

    def get_rgb_stack(self) -> np.ndarray:
        """모든 RGB 이미지를 스택 [N, H, W, 3]"""
        return np.stack([img.rgb for img in self.images])

    def get_depth_stack(self) -> np.ndarray:
        """모든 Depth 이미지를 스택 [N, H, W]"""
        return np.stack([img.depth for img in self.images])

    def get_mask_stack(self) -> Optional[np.ndarray]:
        """모든 마스크를 스택 [N, H, W]"""
        if all(img.has_mask for img in self.images):
            return np.stack([img.mask for img in self.images])
        return None


class ReferenceImageLoader:
    """
    참조 이미지 로더

    /root/fursys_img_251229/extraction 폴더 구조에서
    참조 이미지를 로드합니다.

    Example:
        >>> loader = ReferenceImageLoader("/root/fursys_img_251229/extraction")
        >>> ref_set = loader.load_all_views()
        >>> print(f"Loaded {len(ref_set)} reference images")
    """

    # 폴더명에서 각도 추출을 위한 패턴
    ANGLE_PATTERNS = {
        'front': 0,
        '45': 45,
        '90': 90,
        '135': 135,
        '180': 180,
        '225': 225,
        '270': 270,
        '315': 315,
        'topfront': 'topfront',
        'top90': 'top90',
        'top270': 'top270',
        'bottomfront': 'bottomfront',
        'bottom90': 'bottom90',
        'bottom270': 'bottom270',
    }

    def __init__(
        self,
        base_dir: str,
        depth_scale: float = 0.001,
        exclude_patterns: List[str] = None,
        camera_intrinsics: Optional[Dict[str, float]] = None
    ):
        """
        Args:
            base_dir: 참조 이미지 기본 디렉토리
            depth_scale: depth 값을 미터로 변환하는 스케일
            exclude_patterns: 제외할 폴더 패턴 (예: ['_test'])
            camera_intrinsics: 카메라 내부 파라미터
        """
        self.base_dir = Path(base_dir)
        self.depth_scale = depth_scale
        self.exclude_patterns = exclude_patterns or ['_test']
        self.camera_intrinsics = camera_intrinsics or {
            'fx': 383.883, 'fy': 383.883,
            'cx': 320.499, 'cy': 237.913
        }

        if not self.base_dir.exists():
            raise FileNotFoundError(f"Base directory not found: {base_dir}")

        # 사용 가능한 뷰 폴더 스캔
        self.view_folders = self._scan_view_folders()

        logger.info(f"ReferenceImageLoader initialized: {len(self.view_folders)} view folders found")

    def _scan_view_folders(self) -> List[Path]:
        """참조 이미지 폴더 스캔"""
        folders = []

        for item in sorted(self.base_dir.iterdir()):
            if not item.is_dir():
                continue

            # 제외 패턴 체크
            if any(pattern in item.name for pattern in self.exclude_patterns):
                logger.debug(f"Excluding folder: {item.name}")
                continue

            # rgb 폴더 존재 확인
            if (item / 'rgb').exists():
                folders.append(item)
                logger.debug(f"Found view folder: {item.name}")

        return folders

    def _extract_angle_from_folder(self, folder_name: str) -> Optional[str]:
        """폴더명에서 각도 정보 추출"""
        folder_lower = folder_name.lower()

        # 정확한 매칭을 위해 언더스코어로 구분된 부분 추출
        parts = folder_lower.split('_')

        # 마지막 부분이 각도 정보일 가능성이 높음
        if parts:
            last_part = parts[-1]

            # 정확한 패턴 매칭 (더 긴 패턴부터 확인)
            # 숫자 각도 먼저 확인 (315, 270, 225, 180, 135, 90, 45)
            sorted_patterns = sorted(
                [(k, v) for k, v in self.ANGLE_PATTERNS.items()],
                key=lambda x: -len(str(x[0])) if isinstance(x[0], str) else -len(str(x[1]))
            )

            for pattern, angle in sorted_patterns:
                pattern_str = str(pattern)
                if last_part == pattern_str or (pattern_str in last_part and len(pattern_str) >= 2):
                    return str(angle)

        # 폴백: 기존 방식
        for pattern, angle in self.ANGLE_PATTERNS.items():
            if str(pattern) in folder_lower:
                return str(angle)

        return None

    def _load_single_image(
        self,
        folder: Path,
        image_index: int = 0
    ) -> Optional[ReferenceImage]:
        """
        단일 폴더에서 대표 이미지 로드

        Args:
            folder: 뷰 폴더 경로
            image_index: 로드할 이미지 인덱스 (0=첫 번째)
        """
        rgb_dir = folder / 'rgb'
        depth_dir = folder / 'depth'

        if not rgb_dir.exists() or not depth_dir.exists():
            logger.warning(f"Missing rgb or depth folder in {folder}")
            return None

        # RGB 이미지 목록
        rgb_files = sorted([
            f for f in rgb_dir.iterdir()
            if f.suffix.lower() in ['.png', '.jpg', '.jpeg']
        ])

        if not rgb_files:
            logger.warning(f"No RGB images found in {rgb_dir}")
            return None

        # 인덱스 범위 체크
        if image_index >= len(rgb_files):
            image_index = len(rgb_files) // 2  # 중간 이미지 사용

        rgb_path = rgb_files[image_index]

        # 대응하는 depth 파일 찾기
        # 파일명에서 타임스탬프 추출
        rgb_name = rgb_path.stem
        timestamp = self._extract_timestamp(rgb_name)

        depth_files = sorted([
            f for f in depth_dir.iterdir()
            if f.suffix.lower() == '.png'
        ])

        # 타임스탬프로 매칭 또는 인덱스 기반
        depth_path = None
        if timestamp:
            for df in depth_files:
                if str(timestamp) in df.stem:
                    depth_path = df
                    break

        if depth_path is None and image_index < len(depth_files):
            depth_path = depth_files[image_index]

        if depth_path is None:
            logger.warning(f"No matching depth image for {rgb_path}")
            return None

        # 이미지 로드
        rgb = cv2.imread(str(rgb_path))
        if rgb is None:
            logger.error(f"Failed to load RGB: {rgb_path}")
            return None

        depth_raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if depth_raw is None:
            logger.error(f"Failed to load depth: {depth_path}")
            return None

        # Depth 이미지 처리
        # 3채널인 경우 grayscale로 변환
        if depth_raw.ndim == 3:
            depth_raw = cv2.cvtColor(depth_raw, cv2.COLOR_BGR2GRAY)

        # Depth 변환 (미터 단위)
        depth = depth_raw.astype(np.float32) * self.depth_scale

        # 각도 추출
        view_angle = self._extract_angle_from_folder(folder.name)

        return ReferenceImage(
            rgb=rgb,
            depth=depth,
            mask=None,  # 마스크는 별도 생성 필요
            view_angle=view_angle,
            folder_name=folder.name,
            timestamp=timestamp or 0.0
        )

    def _extract_timestamp(self, filename: str) -> Optional[float]:
        """파일명에서 타임스탬프 추출"""
        # 패턴: rgb_Color_1766968700277.18750000000000
        match = re.search(r'(\d+\.\d+)', filename)
        if match:
            try:
                return float(match.group(1))
            except ValueError:
                pass
        return None

    def load_all_views(
        self,
        images_per_view: int = 1,
        sample_strategy: str = 'middle'
    ) -> ReferenceImageSet:
        """
        모든 뷰에서 참조 이미지 로드

        Args:
            images_per_view: 각 뷰당 로드할 이미지 수
            sample_strategy: 샘플링 전략 ('first', 'middle', 'random')

        Returns:
            ReferenceImageSet: 로드된 참조 이미지 세트
        """
        images = []

        for folder in self.view_folders:
            rgb_dir = folder / 'rgb'
            if not rgb_dir.exists():
                continue

            rgb_files = sorted([
                f for f in rgb_dir.iterdir()
                if f.suffix.lower() in ['.png', '.jpg', '.jpeg']
            ])

            if not rgb_files:
                continue

            # 샘플링 인덱스 결정
            if sample_strategy == 'first':
                indices = list(range(min(images_per_view, len(rgb_files))))
            elif sample_strategy == 'middle':
                mid = len(rgb_files) // 2
                half = images_per_view // 2
                indices = list(range(max(0, mid - half), min(len(rgb_files), mid + half + 1)))[:images_per_view]
            elif sample_strategy == 'random':
                import random
                indices = random.sample(range(len(rgb_files)), min(images_per_view, len(rgb_files)))
            else:
                indices = [0]

            for idx in indices:
                ref_img = self._load_single_image(folder, idx)
                if ref_img is not None:
                    images.append(ref_img)

        logger.info(f"Loaded {len(images)} reference images from {len(self.view_folders)} views")

        return ReferenceImageSet(
            images=images,
            object_name="painting_object",
            camera_intrinsics=self.camera_intrinsics
        )

    def load_specific_views(
        self,
        view_angles: List[str]
    ) -> ReferenceImageSet:
        """
        특정 각도의 뷰만 로드

        Args:
            view_angles: 로드할 각도 리스트 (예: ['front', '45', '90'])
        """
        images = []

        for folder in self.view_folders:
            folder_angle = self._extract_angle_from_folder(folder.name)

            if folder_angle in view_angles or folder_angle in [str(a) for a in view_angles]:
                ref_img = self._load_single_image(folder)
                if ref_img is not None:
                    images.append(ref_img)

        return ReferenceImageSet(
            images=images,
            object_name="painting_object",
            camera_intrinsics=self.camera_intrinsics
        )

    def generate_masks_from_depth(
        self,
        ref_set: ReferenceImageSet,
        depth_threshold: float = 0.1,
        min_depth: float = 0.3,
        max_depth: float = 2.0
    ) -> ReferenceImageSet:
        """
        Depth 기반 마스크 자동 생성

        객체가 배경보다 가까이 있다고 가정하고
        depth 값 기반으로 마스크를 생성합니다.

        Args:
            ref_set: 참조 이미지 세트
            depth_threshold: depth 차이 임계값
            min_depth: 최소 유효 depth
            max_depth: 최대 유효 depth
        """
        for img in ref_set.images:
            # 유효 depth 영역
            valid_mask = (img.depth > min_depth) & (img.depth < max_depth)

            if np.sum(valid_mask) == 0:
                continue

            # depth 히스토그램 분석으로 객체/배경 분리
            valid_depths = img.depth[valid_mask]

            # 전경 객체는 보통 더 가까이 있음
            # 중앙값 기준 분리
            median_depth = np.median(valid_depths)
            foreground_mask = (img.depth > 0) & (img.depth < median_depth + depth_threshold)

            # 형태학적 연산으로 노이즈 제거
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            foreground_mask = foreground_mask.astype(np.uint8) * 255
            foreground_mask = cv2.morphologyEx(foreground_mask, cv2.MORPH_CLOSE, kernel)
            foreground_mask = cv2.morphologyEx(foreground_mask, cv2.MORPH_OPEN, kernel)

            # 가장 큰 연결 영역만 유지
            contours, _ = cv2.findContours(foreground_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                mask = np.zeros_like(foreground_mask)
                cv2.drawContours(mask, [largest_contour], -1, 255, -1)
                img.mask = mask
            else:
                img.mask = foreground_mask

        return ref_set

    def get_available_angles(self) -> List[str]:
        """사용 가능한 각도 목록 반환"""
        angles = []
        for folder in self.view_folders:
            angle = self._extract_angle_from_folder(folder.name)
            if angle:
                angles.append(angle)
        return angles

    def get_statistics(self) -> Dict[str, Any]:
        """로더 통계 정보"""
        return {
            'base_dir': str(self.base_dir),
            'num_view_folders': len(self.view_folders),
            'available_angles': self.get_available_angles(),
            'camera_intrinsics': self.camera_intrinsics
        }
