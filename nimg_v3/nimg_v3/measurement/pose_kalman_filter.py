"""
pose_kalman_filter.py - 6DoF 자세를 위한 Kalman Filter

두 가지 모드 지원:
1. 오일러 기반 (12-상태): 기존 호환
2. 쿼터니언 기반 (13-상태): 권장, 짐벌 락 안전

상태 벡터:
- EULER: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
- QUATERNION: [x, y, z, vx, vy, vz, qx, qy, qz, qw, wx, wy, wz]

설계 원칙 (euler_vs_quaternion_rotation_analysis.md 기반):
- 내부 처리: 쿼터니언 권장 (수치 안정성)
- 출력: 오일러 각도 제공 (사용자 친화성)

Version: 1.0
Author: FurSys AI Team
"""

import numpy as np
from filterpy.kalman import KalmanFilter
from typing import Optional, Union
from dataclasses import dataclass
from enum import Enum
import logging

from .pose_converter import (
    EulerAngles,
    Quaternion,
    PoseConverter
)

logger = logging.getLogger(__name__)


class FilterMode(Enum):
    """Kalman Filter 모드"""
    EULER = "euler"           # 12-상태 (기존 호환)
    QUATERNION = "quaternion" # 13-상태 (권장)


@dataclass
class PoseKalmanState:
    """
    Kalman Filter 상태

    Attributes:
        position: [x, y, z] 미터
        velocity: [vx, vy, vz] m/s
        orientation_euler: Roll, Pitch, Yaw (도)
        orientation_quat: 쿼터니언
        angular_velocity: [wx, wy, wz] 도/s
        speed: 선속도 크기 m/s
    """
    position: np.ndarray
    velocity: np.ndarray
    orientation_euler: EulerAngles
    orientation_quat: Quaternion
    angular_velocity: np.ndarray
    speed: float

    def to_dict(self) -> dict:
        """딕셔너리로 변환"""
        return {
            'position': self.position.tolist(),
            'velocity': self.velocity.tolist(),
            'orientation': {
                'euler': {
                    'roll': self.orientation_euler.roll,
                    'pitch': self.orientation_euler.pitch,
                    'yaw': self.orientation_euler.yaw
                },
                'quaternion': {
                    'x': self.orientation_quat.x,
                    'y': self.orientation_quat.y,
                    'z': self.orientation_quat.z,
                    'w': self.orientation_quat.w
                }
            },
            'angular_velocity': self.angular_velocity.tolist(),
            'speed': self.speed
        }


class PoseKalmanFilter:
    """
    6DoF 자세 추정을 위한 Kalman Filter

    두 가지 모드 지원:
    - EULER: 12-상태 [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
    - QUATERNION: 13-상태 [x, y, z, vx, vy, vz, qx, qy, qz, qw, wx, wy, wz]

    쿼터니언 모드가 권장됩니다:
    - 짐벌 락 문제 없음
    - 수치적으로 안정적
    - 각도 래핑 문제 없음

    Example:
        >>> kf = PoseKalmanFilter(dt=1/30.0, mode=FilterMode.QUATERNION)
        >>> state = kf.predict_and_update(position, quaternion)
        >>> print(f"Speed: {state.speed:.3f} m/s")
    """

    def __init__(
        self,
        dt: float = 1/30.0,
        mode: FilterMode = FilterMode.QUATERNION,
        process_noise_pos: float = 0.01,
        process_noise_vel: float = 0.1,
        process_noise_orient: float = 0.1,
        process_noise_angular_vel: float = 1.0,
        measurement_noise_pos: float = 0.005,
        measurement_noise_orient: float = 0.5,
        adaptive_noise: bool = True
    ):
        """
        Args:
            dt: 프레임 간 시간 간격 (초)
            mode: 필터 모드 (EULER 또는 QUATERNION)
            process_noise_*: 프로세스 노이즈 파라미터
            measurement_noise_*: 측정 노이즈 파라미터
            adaptive_noise: 거리 기반 적응형 노이즈 활성화
        """
        self.dt = dt
        self.mode = mode
        self.adaptive_noise = adaptive_noise
        self._converter = PoseConverter()

        # 노이즈 파라미터 저장
        self._pn_pos = process_noise_pos
        self._pn_vel = process_noise_vel
        self._pn_orient = process_noise_orient
        self._pn_angvel = process_noise_angular_vel
        self._mn_pos = measurement_noise_pos
        self._mn_orient = measurement_noise_orient

        # 필터 초기화
        if mode == FilterMode.EULER:
            self._init_euler_filter()
        else:
            self._init_quaternion_filter()

        self._initialized = False
        self._last_state = None

        logger.info(f"PoseKalmanFilter initialized: mode={mode.value}, dt={dt}")

    def _init_euler_filter(self):
        """
        오일러 기반 12-상태 필터

        상태: [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
        측정: [x, y, z, roll, pitch, yaw]
        """
        self.dim_x = 12
        self.dim_z = 6

        self.kf = KalmanFilter(dim_x=self.dim_x, dim_z=self.dim_z)

        # 상태 전이 행렬 F (등속 모델)
        self.kf.F = np.eye(12)
        # 위치 += 속도 * dt
        self.kf.F[0, 3] = self.dt  # x += vx * dt
        self.kf.F[1, 4] = self.dt  # y += vy * dt
        self.kf.F[2, 5] = self.dt  # z += vz * dt
        # 각도 += 각속도 * dt
        self.kf.F[6, 9] = self.dt   # roll += wx * dt
        self.kf.F[7, 10] = self.dt  # pitch += wy * dt
        self.kf.F[8, 11] = self.dt  # yaw += wz * dt

        # 측정 행렬 H
        self.kf.H = np.zeros((6, 12))
        self.kf.H[0, 0] = 1  # x
        self.kf.H[1, 1] = 1  # y
        self.kf.H[2, 2] = 1  # z
        self.kf.H[3, 6] = 1  # roll
        self.kf.H[4, 7] = 1  # pitch
        self.kf.H[5, 8] = 1  # yaw

        # 프로세스 노이즈 Q
        self.kf.Q = np.diag([
            self._pn_pos, self._pn_pos, self._pn_pos,        # 위치
            self._pn_vel, self._pn_vel, self._pn_vel,        # 속도
            self._pn_orient, self._pn_orient, self._pn_orient,  # 각도
            self._pn_angvel, self._pn_angvel, self._pn_angvel   # 각속도
        ])

        # 측정 노이즈 R
        self.kf.R = np.diag([
            self._mn_pos, self._mn_pos, self._mn_pos,       # 위치
            self._mn_orient, self._mn_orient, self._mn_orient  # 각도
        ])

        # 초기 공분산
        self.kf.P *= 1.0

    def _init_quaternion_filter(self):
        """
        쿼터니언 기반 13-상태 필터

        상태: [x, y, z, vx, vy, vz, qx, qy, qz, qw, wx, wy, wz]
        측정: [x, y, z, qx, qy, qz, qw]

        Note: 쿼터니언 업데이트는 비선형이지만,
              여기서는 선형 근사를 사용합니다.
        """
        self.dim_x = 13
        self.dim_z = 7

        self.kf = KalmanFilter(dim_x=self.dim_x, dim_z=self.dim_z)

        # 상태 전이 행렬 F
        self.kf.F = np.eye(13)
        # 위치 += 속도 * dt
        self.kf.F[0, 3] = self.dt
        self.kf.F[1, 4] = self.dt
        self.kf.F[2, 5] = self.dt
        # 쿼터니언은 predict()에서 별도 처리

        # 측정 행렬 H
        self.kf.H = np.zeros((7, 13))
        self.kf.H[0, 0] = 1  # x
        self.kf.H[1, 1] = 1  # y
        self.kf.H[2, 2] = 1  # z
        self.kf.H[3, 6] = 1  # qx
        self.kf.H[4, 7] = 1  # qy
        self.kf.H[5, 8] = 1  # qz
        self.kf.H[6, 9] = 1  # qw

        # 프로세스 노이즈 Q
        self.kf.Q = np.diag([
            self._pn_pos, self._pn_pos, self._pn_pos,     # 위치
            self._pn_vel, self._pn_vel, self._pn_vel,     # 속도
            self._pn_orient, self._pn_orient, self._pn_orient, self._pn_orient,  # 쿼터니언
            self._pn_angvel, self._pn_angvel, self._pn_angvel  # 각속도
        ])

        # 측정 노이즈 R
        self.kf.R = np.diag([
            self._mn_pos, self._mn_pos, self._mn_pos,           # 위치
            self._mn_orient, self._mn_orient, self._mn_orient, self._mn_orient  # 쿼터니언
        ])

        # 초기 공분산
        self.kf.P *= 1.0

    def initialize(
        self,
        position: np.ndarray,
        orientation: Union[EulerAngles, Quaternion]
    ):
        """
        필터 초기화

        Args:
            position: [x, y, z] 초기 위치
            orientation: EulerAngles 또는 Quaternion
        """
        if self.mode == FilterMode.EULER:
            if isinstance(orientation, Quaternion):
                orientation = self._converter.quaternion_to_euler(orientation)

            self.kf.x = np.array([
                position[0], position[1], position[2],  # 위치
                0, 0, 0,                                 # 속도
                orientation.roll, orientation.pitch, orientation.yaw,  # 각도
                0, 0, 0                                  # 각속도
            ], dtype=np.float64)
        else:
            if isinstance(orientation, EulerAngles):
                orientation = self._converter.euler_to_quaternion(orientation)

            orientation = orientation.normalize()

            self.kf.x = np.array([
                position[0], position[1], position[2],  # 위치
                0, 0, 0,                                 # 속도
                orientation.x, orientation.y, orientation.z, orientation.w,  # 쿼터니언
                0, 0, 0                                  # 각속도
            ], dtype=np.float64)

        self._initialized = True
        logger.debug(f"Filter initialized at position={position}")

    def predict(self) -> PoseKalmanState:
        """
        예측 단계

        Returns:
            예측된 상태
        """
        if not self._initialized:
            raise RuntimeError("Filter not initialized. Call initialize() first.")

        self.kf.predict()

        # 쿼터니언 정규화 (QUATERNION 모드)
        if self.mode == FilterMode.QUATERNION:
            self._normalize_quaternion()

        return self.get_state()

    def update(
        self,
        position: np.ndarray,
        orientation: Union[EulerAngles, Quaternion]
    ) -> PoseKalmanState:
        """
        업데이트 단계

        Args:
            position: 측정된 위치 [x, y, z]
            orientation: 측정된 자세 (EulerAngles 또는 Quaternion)

        Returns:
            업데이트된 상태
        """
        if not self._initialized:
            self.initialize(position, orientation)
            return self.get_state()

        # 측정 벡터 구성
        if self.mode == FilterMode.EULER:
            if isinstance(orientation, Quaternion):
                orientation = self._converter.quaternion_to_euler(orientation)

            # 각도 래핑 처리 (현재 상태와의 차이 최소화)
            euler_arr = self._unwrap_angles(orientation)

            measurement = np.concatenate([
                position,
                euler_arr
            ])
        else:
            if isinstance(orientation, EulerAngles):
                orientation = self._converter.euler_to_quaternion(orientation)

            orientation = orientation.normalize()

            # 쿼터니언 부호 일관성 유지
            orientation = self._ensure_quaternion_consistency(orientation)

            measurement = np.concatenate([
                position,
                [orientation.x, orientation.y, orientation.z, orientation.w]
            ])

        self.kf.update(measurement)

        # 쿼터니언 정규화
        if self.mode == FilterMode.QUATERNION:
            self._normalize_quaternion()

        state = self.get_state()
        self._last_state = state

        return state

    def _unwrap_angles(self, euler: EulerAngles) -> np.ndarray:
        """
        각도 래핑 처리 (연속성 유지)

        현재 추정 상태와 측정값 사이의 차이가 180도를 넘지 않도록 조정
        """
        euler_arr = np.array([euler.roll, euler.pitch, euler.yaw])

        # 현재 상태의 각도
        curr_angles = self.kf.x[6:9]

        # 차이 계산 및 래핑
        for i in range(3):
            diff = euler_arr[i] - curr_angles[i]
            if diff > 180:
                euler_arr[i] -= 360
            elif diff < -180:
                euler_arr[i] += 360

        return euler_arr

    def _ensure_quaternion_consistency(self, quat: Quaternion) -> Quaternion:
        """
        쿼터니언 부호 일관성 유지

        q와 -q는 같은 회전을 나타내므로,
        현재 상태와의 내적이 양수가 되도록 조정
        """
        curr_quat = self.kf.x[6:10]
        meas_quat = quat.to_array()

        if np.dot(curr_quat, meas_quat) < 0:
            return Quaternion(x=-quat.x, y=-quat.y, z=-quat.z, w=-quat.w)

        return quat

    def _normalize_quaternion(self):
        """상태 벡터의 쿼터니언 정규화"""
        quat = self.kf.x[6:10]
        norm = np.linalg.norm(quat)
        if norm > 1e-10:
            self.kf.x[6:10] = quat / norm

    def predict_and_update(
        self,
        position: np.ndarray,
        orientation: Union[EulerAngles, Quaternion]
    ) -> PoseKalmanState:
        """
        예측 + 업데이트 한번에

        Args:
            position: 측정된 위치
            orientation: 측정된 자세

        Returns:
            필터링된 상태
        """
        self.predict()
        return self.update(position, orientation)

    def get_state(self) -> PoseKalmanState:
        """현재 상태 반환"""
        state = self.kf.x.flatten()

        if self.mode == FilterMode.EULER:
            euler = EulerAngles(
                roll=float(state[6]),
                pitch=float(state[7]),
                yaw=float(state[8])
            )
            quat = self._converter.euler_to_quaternion(euler)
            angular_vel = state[9:12]
        else:
            quat = Quaternion(
                x=float(state[6]),
                y=float(state[7]),
                z=float(state[8]),
                w=float(state[9])
            )
            euler = self._converter.quaternion_to_euler(quat)
            angular_vel = state[10:13]

        return PoseKalmanState(
            position=state[0:3].copy(),
            velocity=state[3:6].copy(),
            orientation_euler=euler,
            orientation_quat=quat,
            angular_velocity=angular_vel.copy(),
            speed=float(np.linalg.norm(state[3:6]))
        )

    def update_adaptive_noise(self, depth_distance: float):
        """
        거리 기반 적응형 측정 노이즈 업데이트

        RealSense D455 depth 오차 모델:
        - 오차는 거리²에 비례
        - 약 1m에서 약 5mm 오차

        Args:
            depth_distance: 평균 깊이 거리 (미터)
        """
        if not self.adaptive_noise:
            return

        # D455 depth 오차 모델
        base_error = 0.005  # 1m에서 5mm
        pos_error = base_error * (depth_distance ** 2)
        pos_error = np.clip(pos_error, 0.002, 0.1)

        # 측정 노이즈 업데이트 (위치만)
        self.kf.R[0, 0] = pos_error ** 2
        self.kf.R[1, 1] = pos_error ** 2
        self.kf.R[2, 2] = pos_error ** 2

        logger.debug(f"Adaptive noise updated: depth={depth_distance:.2f}m, error={pos_error*1000:.1f}mm")

    def set_dt(self, dt: float):
        """
        시간 간격 업데이트

        가변 프레임률 지원
        """
        self.dt = dt

        if self.mode == FilterMode.EULER:
            self.kf.F[0, 3] = dt
            self.kf.F[1, 4] = dt
            self.kf.F[2, 5] = dt
            self.kf.F[6, 9] = dt
            self.kf.F[7, 10] = dt
            self.kf.F[8, 11] = dt
        else:
            self.kf.F[0, 3] = dt
            self.kf.F[1, 4] = dt
            self.kf.F[2, 5] = dt

    def reset(self):
        """필터 리셋"""
        self.kf.x = np.zeros(self.dim_x)
        self.kf.P = np.eye(self.dim_x) * 1.0

        # 쿼터니언 모드에서 단위 쿼터니언으로 초기화
        if self.mode == FilterMode.QUATERNION:
            self.kf.x[9] = 1.0  # qw = 1

        self._initialized = False
        self._last_state = None
        logger.debug("Filter reset")

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def covariance(self) -> np.ndarray:
        """현재 공분산 행렬"""
        return self.kf.P.copy()

    @property
    def position_uncertainty(self) -> np.ndarray:
        """위치 불확실성 (표준편차)"""
        return np.sqrt(np.diag(self.kf.P[:3, :3]))

    @property
    def orientation_uncertainty(self) -> np.ndarray:
        """자세 불확실성 (표준편차)"""
        if self.mode == FilterMode.EULER:
            return np.sqrt(np.diag(self.kf.P[6:9, 6:9]))
        else:
            return np.sqrt(np.diag(self.kf.P[6:10, 6:10]))


class AdaptivePoseKalmanFilter(PoseKalmanFilter):
    """
    적응형 Kalman Filter

    이동/정지 상태 감지 및 노이즈 자동 조정
    """

    def __init__(
        self,
        dt: float = 1/30.0,
        mode: FilterMode = FilterMode.QUATERNION,
        motion_threshold: float = 0.01,  # m/s
        rotation_threshold: float = 1.0,  # deg/s
        **kwargs
    ):
        super().__init__(dt=dt, mode=mode, **kwargs)

        self.motion_threshold = motion_threshold
        self.rotation_threshold = rotation_threshold

        # 상태별 노이즈 팩터
        self._stationary_factor = 0.1  # 정지 시 노이즈 감소
        self._moving_factor = 1.0      # 이동 시 기본 노이즈

    def update(
        self,
        position: np.ndarray,
        orientation: Union[EulerAngles, Quaternion]
    ) -> PoseKalmanState:
        """적응형 업데이트"""
        # 기본 업데이트
        state = super().update(position, orientation)

        # 상태 감지 및 노이즈 조정
        is_moving = (
            state.speed > self.motion_threshold or
            np.linalg.norm(state.angular_velocity) > self.rotation_threshold
        )

        factor = self._moving_factor if is_moving else self._stationary_factor
        self._adjust_noise(factor)

        return state

    def _adjust_noise(self, factor: float):
        """노이즈 조정"""
        # 프로세스 노이즈 조정
        base_q = np.diag([
            self._pn_pos, self._pn_pos, self._pn_pos,
            self._pn_vel, self._pn_vel, self._pn_vel,
            *([self._pn_orient] * (4 if self.mode == FilterMode.QUATERNION else 3)),
            self._pn_angvel, self._pn_angvel, self._pn_angvel
        ])

        self.kf.Q = base_q * factor
