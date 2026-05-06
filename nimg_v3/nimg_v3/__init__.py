"""
nimg_v3 - FoundationPose 기반 6DoF 자세 추정 시스템

주요 특징:
- FoundationPose 기반 통합 6DoF 자세 추정
- Model-Free 지원 (CAD 없이 참조 이미지로 동작)
- Zero-Shot 일반화 (Domain Shift 해결)
- 하이브리드 회전 표현 (쿼터니언 + 오일러)

Version: 3.0
Author: FurSys AI Team
"""

__version__ = "3.0.0"
__author__ = "FurSys AI Team"

from .measurement.pose_converter import (
    PoseConverter,
    EulerAngles,
    Quaternion,
    PoseComponents,
    RotationOrder,
    pose_to_euler,
    pose_to_quaternion,
    compute_angle_change
)

from .measurement.pose_kalman_filter import (
    PoseKalmanFilter,
    PoseKalmanState,
    FilterMode
)

__all__ = [
    # Pose Converter
    'PoseConverter',
    'EulerAngles',
    'Quaternion',
    'PoseComponents',
    'RotationOrder',
    'pose_to_euler',
    'pose_to_quaternion',
    'compute_angle_change',
    # Kalman Filter
    'PoseKalmanFilter',
    'PoseKalmanState',
    'FilterMode',
]
