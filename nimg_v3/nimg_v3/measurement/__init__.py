"""
measurement 모듈 - 자세 변환 및 속도 추정

6DoF 자세 추정 결과를 다양한 형식으로 변환하고,
Kalman Filter를 통한 속도/각속도 추정을 제공합니다.

주요 기능:
- 회전 행렬 <-> 오일러 각도 <-> 쿼터니언 상호 변환
- 하이브리드 방식 (내부: 쿼터니언, 출력: 오일러)
- 12/13-상태 Kalman Filter (속도/각속도 추정)
- 짐벌 락 감지 및 경고
"""

from .pose_converter import (
    PoseConverter,
    EulerAngles,
    Quaternion,
    PoseComponents,
    RotationOrder,
    pose_to_euler,
    pose_to_quaternion,
    compute_angle_change
)

from .pose_kalman_filter import (
    PoseKalmanFilter,
    PoseKalmanState,
    FilterMode
)

__all__ = [
    'PoseConverter',
    'EulerAngles',
    'Quaternion',
    'PoseComponents',
    'RotationOrder',
    'pose_to_euler',
    'pose_to_quaternion',
    'compute_angle_change',
    'PoseKalmanFilter',
    'PoseKalmanState',
    'FilterMode',
]
