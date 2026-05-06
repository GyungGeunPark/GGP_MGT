"""
pose 모듈 - FoundationPose 기반 6DoF 자세 추정

Model-Free 및 Model-Based 방식을 모두 지원하는
FoundationPose 통합 모듈입니다.

주요 기능:
- FoundationPose 래퍼
- Neural Object Field 학습 및 로드
- 참조 이미지 관리
- 자세 추정 및 추적
"""

from .foundationpose_estimator import (
    FoundationPoseEstimator,
    PoseMode,
    TrackingState,
    PoseResult
)

from .neural_object_field import (
    NeuralObjectField,
    ReferenceImageSet
)

from .reference_image_loader import (
    ReferenceImageLoader,
    ReferenceImage
)

__all__ = [
    'FoundationPoseEstimator',
    'PoseMode',
    'TrackingState',
    'PoseResult',
    'NeuralObjectField',
    'ReferenceImageSet',
    'ReferenceImageLoader',
    'ReferenceImage',
]
