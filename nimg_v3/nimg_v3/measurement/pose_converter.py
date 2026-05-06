"""
pose_converter.py - 통합 자세 변환 모듈

6DoF 자세 행렬에서 다양한 회전 표현으로 변환 지원:
- 오일러 각도 (Roll, Pitch, Yaw) - 사용자 표시용
- 쿼터니언 (x, y, z, w) - 내부 계산/ROS 통신용
- 회전 행렬 (3x3) - 직접 적용용

설계 원칙 (euler_vs_quaternion_rotation_analysis.md 기반):
1. 내부 처리: 쿼터니언 (수치 안정성, 연산 효율)
2. 외부 인터페이스: 오일러 (직관적 이해)
3. ROS 통신: 쿼터니언 (geometry_msgs/Pose 표준)
4. 저장/로깅: 둘 다 (호환성 + 디버깅)

Version: 1.0
Author: FurSys AI Team
Reference: euler_vs_quaternion_rotation_analysis.md
"""

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from typing import Tuple, Dict, Any, Optional, Union
from dataclasses import dataclass, field
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class RotationOrder(Enum):
    """오일러 각도 회전 순서"""
    XYZ = 'xyz'  # Roll-Pitch-Yaw (항공/로봇공학 표준)
    ZYX = 'zyx'  # Yaw-Pitch-Roll
    ZXY = 'zxy'
    YXZ = 'yxz'
    XZY = 'xzy'
    YZX = 'yzx'

    # 내재적(intrinsic) 회전 순서 (대문자)
    INTRINSIC_XYZ = 'XYZ'
    INTRINSIC_ZYX = 'ZYX'


@dataclass
class EulerAngles:
    """
    오일러 각도 (degrees by default)

    Attributes:
        roll: X축 회전 (좌우 기울기)
        pitch: Y축 회전 (앞뒤 기울기)
        yaw: Z축 회전 (방향 전환)
    """
    roll: float   # X축 회전
    pitch: float  # Y축 회전
    yaw: float    # Z축 회전

    def to_radians(self) -> 'EulerAngles':
        """라디안으로 변환"""
        return EulerAngles(
            roll=np.deg2rad(self.roll),
            pitch=np.deg2rad(self.pitch),
            yaw=np.deg2rad(self.yaw)
        )

    def to_degrees(self) -> 'EulerAngles':
        """도 단위로 변환 (이미 도 단위면 그대로)"""
        return self

    def to_array(self) -> np.ndarray:
        """numpy 배열로 변환 [roll, pitch, yaw]"""
        return np.array([self.roll, self.pitch, self.yaw])

    def normalize(self) -> 'EulerAngles':
        """각도를 -180 ~ 180 범위로 정규화"""
        def norm_angle(a: float) -> float:
            while a > 180:
                a -= 360
            while a < -180:
                a += 360
            return a

        return EulerAngles(
            roll=norm_angle(self.roll),
            pitch=norm_angle(self.pitch),
            yaw=norm_angle(self.yaw)
        )

    def __sub__(self, other: 'EulerAngles') -> 'EulerAngles':
        """두 오일러 각도의 차이"""
        diff = EulerAngles(
            roll=self.roll - other.roll,
            pitch=self.pitch - other.pitch,
            yaw=self.yaw - other.yaw
        )
        return diff.normalize()

    def __repr__(self) -> str:
        return f"EulerAngles(R={self.roll:.2f}, P={self.pitch:.2f}, Y={self.yaw:.2f})"


@dataclass
class Quaternion:
    """
    쿼터니언 (x, y, z, w) - scipy/ROS 형식

    표현: q = w + xi + yj + zk
    단위 쿼터니언 조건: |q| = sqrt(x² + y² + z² + w²) = 1
    """
    x: float
    y: float
    z: float
    w: float

    def to_array(self) -> np.ndarray:
        """[x, y, z, w] 형식 (scipy 표준)"""
        return np.array([self.x, self.y, self.z, self.w])

    def to_array_wxyz(self) -> np.ndarray:
        """[w, x, y, z] 형식 (일부 라이브러리용)"""
        return np.array([self.w, self.x, self.y, self.z])

    def normalize(self) -> 'Quaternion':
        """단위 쿼터니언으로 정규화"""
        arr = self.to_array()
        norm = np.linalg.norm(arr)
        if norm < 1e-10:
            return Quaternion(x=0, y=0, z=0, w=1)
        arr = arr / norm
        return Quaternion(x=arr[0], y=arr[1], z=arr[2], w=arr[3])

    def conjugate(self) -> 'Quaternion':
        """켤레 쿼터니언 (회전의 역)"""
        return Quaternion(x=-self.x, y=-self.y, z=-self.z, w=self.w)

    def inverse(self) -> 'Quaternion':
        """역 쿼터니언"""
        return self.conjugate().normalize()

    @property
    def norm(self) -> float:
        """쿼터니언 크기"""
        return np.linalg.norm(self.to_array())

    @property
    def is_unit(self) -> bool:
        """단위 쿼터니언 여부"""
        return abs(self.norm - 1.0) < 1e-6

    def __mul__(self, other: 'Quaternion') -> 'Quaternion':
        """해밀턴 곱 (회전 합성)"""
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z

        return Quaternion(
            x=w1*x2 + x1*w2 + y1*z2 - z1*y2,
            y=w1*y2 - x1*z2 + y1*w2 + z1*x2,
            z=w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w=w1*w2 - x1*x2 - y1*y2 - z1*z2
        )

    def dot(self, other: 'Quaternion') -> float:
        """내적"""
        return np.dot(self.to_array(), other.to_array())

    def angle_to(self, other: 'Quaternion') -> float:
        """다른 쿼터니언까지의 각도 (도)"""
        dot = abs(self.dot(other))
        dot = np.clip(dot, -1.0, 1.0)
        return np.rad2deg(2 * np.arccos(dot))

    def __repr__(self) -> str:
        return f"Quaternion(x={self.x:.4f}, y={self.y:.4f}, z={self.z:.4f}, w={self.w:.4f})"

    @classmethod
    def identity(cls) -> 'Quaternion':
        """단위 쿼터니언 (회전 없음)"""
        return cls(x=0, y=0, z=0, w=1)

    @classmethod
    def from_axis_angle(cls, axis: np.ndarray, angle_deg: float) -> 'Quaternion':
        """축-각도에서 생성"""
        axis = axis / np.linalg.norm(axis)
        angle_rad = np.deg2rad(angle_deg) / 2
        sin_a = np.sin(angle_rad)
        cos_a = np.cos(angle_rad)
        return cls(
            x=axis[0] * sin_a,
            y=axis[1] * sin_a,
            z=axis[2] * sin_a,
            w=cos_a
        )


@dataclass
class PoseComponents:
    """
    자세 구성요소 통합

    6DoF 자세를 다양한 표현으로 동시에 제공합니다.
    """
    translation: np.ndarray      # [x, y, z] 미터
    euler: EulerAngles          # Roll, Pitch, Yaw (도)
    quaternion: Quaternion      # (x, y, z, w)
    rotation_matrix: np.ndarray # 3x3
    gimbal_lock_warning: bool   # 짐벌 락 경고

    def to_dict(self) -> Dict[str, Any]:
        """딕셔너리로 변환"""
        return {
            'translation': self.translation.tolist(),
            'euler': {
                'roll': self.euler.roll,
                'pitch': self.euler.pitch,
                'yaw': self.euler.yaw
            },
            'quaternion': {
                'x': self.quaternion.x,
                'y': self.quaternion.y,
                'z': self.quaternion.z,
                'w': self.quaternion.w
            },
            'rotation_matrix': self.rotation_matrix.tolist(),
            'gimbal_lock_warning': self.gimbal_lock_warning
        }


class PoseConverter:
    """
    통합 자세 변환 클래스

    FoundationPose 출력(4x4 변환 행렬)을
    다양한 회전 표현으로 변환합니다.

    설계 원칙:
    - 내부 연산: 쿼터니언 (수치 안정성)
    - 사용자 출력: 오일러 (직관성)
    - ROS 통신: 쿼터니언 (표준 준수)

    Example:
        >>> converter = PoseConverter()
        >>> components = converter.pose_matrix_to_components(pose_4x4)
        >>> print(f"Yaw: {components.euler.yaw:.2f}")
    """

    GIMBAL_LOCK_THRESHOLD = 85.0  # 도 (±90°에서 ±5° 이내)

    def __init__(
        self,
        rotation_order: RotationOrder = RotationOrder.XYZ,
        warn_gimbal_lock: bool = True,
        use_degrees: bool = True
    ):
        """
        Args:
            rotation_order: 오일러 각도 회전 순서
            warn_gimbal_lock: 짐벌 락 경고 활성화
            use_degrees: 오일러 각도를 도 단위로 출력
        """
        self.rotation_order = rotation_order
        self.warn_gimbal_lock = warn_gimbal_lock
        self.use_degrees = use_degrees

    def pose_matrix_to_components(
        self,
        pose_matrix: np.ndarray
    ) -> PoseComponents:
        """
        4x4 자세 행렬에서 모든 구성요소 추출

        Args:
            pose_matrix: 4x4 변환 행렬 [R|t; 0 1]

        Returns:
            PoseComponents: 이동, 오일러, 쿼터니언, 회전 행렬
        """
        if pose_matrix.shape != (4, 4):
            raise ValueError(f"Expected 4x4 matrix, got {pose_matrix.shape}")

        # 이동 추출
        translation = pose_matrix[:3, 3].copy()

        # 회전 행렬 추출
        rotation_matrix = pose_matrix[:3, :3].copy()

        # scipy Rotation 객체 생성
        rot = Rotation.from_matrix(rotation_matrix)

        # 오일러 각도 추출
        euler_array = rot.as_euler(self.rotation_order.value, degrees=self.use_degrees)
        euler = EulerAngles(
            roll=float(euler_array[0]),
            pitch=float(euler_array[1]),
            yaw=float(euler_array[2])
        )

        # 쿼터니언 추출 [x, y, z, w]
        quat_array = rot.as_quat()
        quaternion = Quaternion(
            x=float(quat_array[0]),
            y=float(quat_array[1]),
            z=float(quat_array[2]),
            w=float(quat_array[3])
        )

        # 짐벌 락 감지
        gimbal_lock = self._check_gimbal_lock(euler.pitch)

        return PoseComponents(
            translation=translation,
            euler=euler,
            quaternion=quaternion,
            rotation_matrix=rotation_matrix,
            gimbal_lock_warning=gimbal_lock
        )

    def pose_to_euler(
        self,
        pose_matrix: np.ndarray
    ) -> Tuple[np.ndarray, EulerAngles]:
        """
        기존 API 호환: 4x4 행렬에서 이동과 오일러 각도 추출

        Args:
            pose_matrix: 4x4 변환 행렬

        Returns:
            (translation, euler_angles)
        """
        components = self.pose_matrix_to_components(pose_matrix)
        return components.translation, components.euler

    def pose_to_quaternion(
        self,
        pose_matrix: np.ndarray
    ) -> Tuple[np.ndarray, Quaternion]:
        """
        4x4 행렬에서 이동과 쿼터니언 추출

        Args:
            pose_matrix: 4x4 변환 행렬

        Returns:
            (translation, quaternion)
        """
        components = self.pose_matrix_to_components(pose_matrix)
        return components.translation, components.quaternion

    def quaternion_to_euler(
        self,
        quat: Quaternion
    ) -> EulerAngles:
        """
        쿼터니언에서 오일러 각도로 변환
        """
        rot = Rotation.from_quat(quat.to_array())
        euler_array = rot.as_euler(self.rotation_order.value, degrees=self.use_degrees)
        return EulerAngles(
            roll=float(euler_array[0]),
            pitch=float(euler_array[1]),
            yaw=float(euler_array[2])
        )

    def euler_to_quaternion(
        self,
        euler: EulerAngles
    ) -> Quaternion:
        """
        오일러 각도에서 쿼터니언으로 변환
        """
        euler_array = euler.to_array()
        rot = Rotation.from_euler(
            self.rotation_order.value,
            euler_array,
            degrees=self.use_degrees
        )
        quat_array = rot.as_quat()
        return Quaternion(
            x=float(quat_array[0]),
            y=float(quat_array[1]),
            z=float(quat_array[2]),
            w=float(quat_array[3])
        )

    def rotation_matrix_to_quaternion(
        self,
        R: np.ndarray
    ) -> Quaternion:
        """회전 행렬에서 쿼터니언으로 변환"""
        rot = Rotation.from_matrix(R)
        quat_array = rot.as_quat()
        return Quaternion(
            x=float(quat_array[0]),
            y=float(quat_array[1]),
            z=float(quat_array[2]),
            w=float(quat_array[3])
        )

    def quaternion_to_rotation_matrix(
        self,
        quat: Quaternion
    ) -> np.ndarray:
        """쿼터니언에서 회전 행렬로 변환"""
        rot = Rotation.from_quat(quat.to_array())
        return rot.as_matrix()

    def euler_to_rotation_matrix(
        self,
        euler: EulerAngles
    ) -> np.ndarray:
        """오일러 각도에서 회전 행렬로 변환"""
        rot = Rotation.from_euler(
            self.rotation_order.value,
            euler.to_array(),
            degrees=self.use_degrees
        )
        return rot.as_matrix()

    def rotation_matrix_to_euler(
        self,
        R: np.ndarray
    ) -> EulerAngles:
        """회전 행렬에서 오일러 각도로 변환"""
        rot = Rotation.from_matrix(R)
        euler_array = rot.as_euler(self.rotation_order.value, degrees=self.use_degrees)
        return EulerAngles(
            roll=float(euler_array[0]),
            pitch=float(euler_array[1]),
            yaw=float(euler_array[2])
        )

    def _check_gimbal_lock(self, pitch: float) -> bool:
        """
        짐벌 락 근접 여부 확인

        짐벌 락은 Pitch가 ±90°에 근접할 때 발생합니다.
        이 경우 Roll과 Yaw의 구분이 불가능해집니다.
        """
        if not self.warn_gimbal_lock:
            return False

        # 도 단위 기준
        pitch_deg = pitch if self.use_degrees else np.rad2deg(pitch)

        if abs(abs(pitch_deg) - 90) < (90 - self.GIMBAL_LOCK_THRESHOLD):
            logger.warning(f"Approaching gimbal lock (pitch={pitch_deg:.1f})")
            return True

        return False

    @staticmethod
    def slerp(
        q0: Quaternion,
        q1: Quaternion,
        t: float
    ) -> Quaternion:
        """
        두 쿼터니언 사이 구면 선형 보간 (SLERP)

        SLERP는 두 회전 사이를 일정한 각속도로 보간합니다.
        오일러 각도의 선형 보간과 달리 자연스러운 회전을 생성합니다.

        Args:
            q0: 시작 쿼터니언
            q1: 끝 쿼터니언
            t: 보간 파라미터 [0, 1]

        Returns:
            보간된 쿼터니언
        """
        key_times = [0, 1]
        key_rots = Rotation.from_quat([q0.to_array(), q1.to_array()])
        slerp = Slerp(key_times, key_rots)

        result = slerp(t).as_quat()
        return Quaternion(
            x=float(result[0]),
            y=float(result[1]),
            z=float(result[2]),
            w=float(result[3])
        )

    @staticmethod
    def slerp_multi(
        quaternions: list,
        times: list,
        query_time: float
    ) -> Quaternion:
        """
        여러 쿼터니언 사이 구면 선형 보간

        Args:
            quaternions: 쿼터니언 리스트
            times: 각 쿼터니언에 대응하는 시간
            query_time: 보간할 시간

        Returns:
            보간된 쿼터니언
        """
        quats = [q.to_array() for q in quaternions]
        key_rots = Rotation.from_quat(quats)
        slerp = Slerp(times, key_rots)

        result = slerp(query_time).as_quat()
        return Quaternion(
            x=float(result[0]),
            y=float(result[1]),
            z=float(result[2]),
            w=float(result[3])
        )

    @staticmethod
    def compute_rotation_difference(
        q1: Quaternion,
        q2: Quaternion
    ) -> Tuple[float, np.ndarray]:
        """
        두 쿼터니언 사이의 회전 차이 계산

        Returns:
            angle: 회전 각도 (도)
            axis: 회전 축 (단위 벡터)
        """
        rot1 = Rotation.from_quat(q1.to_array())
        rot2 = Rotation.from_quat(q2.to_array())

        diff = rot2 * rot1.inv()
        rotvec = diff.as_rotvec()

        angle = np.linalg.norm(rotvec)
        axis = rotvec / angle if angle > 1e-6 else np.array([0, 0, 1])

        return np.rad2deg(angle), axis

    def compute_relative_pose(
        self,
        pose_ref: np.ndarray,
        pose_curr: np.ndarray
    ) -> PoseComponents:
        """
        기준 자세 대비 상대 자세 계산

        Args:
            pose_ref: 기준 자세 (4x4)
            pose_curr: 현재 자세 (4x4)

        Returns:
            상대 자세 구성요소
        """
        # 상대 변환: T_rel = T_ref^(-1) * T_curr
        pose_ref_inv = np.linalg.inv(pose_ref)
        pose_rel = pose_ref_inv @ pose_curr

        return self.pose_matrix_to_components(pose_rel)


# 편의 함수들 (기존 API 호환)
def pose_to_euler(pose_matrix: np.ndarray) -> Tuple[np.ndarray, EulerAngles]:
    """
    기존 API 호환 함수: 4x4 자세 행렬에서 이동과 오일러 각도 추출
    """
    converter = PoseConverter()
    return converter.pose_to_euler(pose_matrix)


def pose_to_quaternion(pose_matrix: np.ndarray) -> Tuple[np.ndarray, Quaternion]:
    """
    4x4 자세 행렬에서 이동과 쿼터니언 추출
    """
    converter = PoseConverter()
    return converter.pose_to_quaternion(pose_matrix)


def compute_angle_change(
    prev_pose: np.ndarray,
    curr_pose: np.ndarray,
    use_quaternion: bool = True
) -> np.ndarray:
    """
    두 자세 간의 각도 변화 계산

    쿼터니언을 사용한 계산이 더 정확합니다 (짐벌 락 영향 없음).

    Args:
        prev_pose: 이전 자세 (4x4)
        curr_pose: 현재 자세 (4x4)
        use_quaternion: True면 쿼터니언으로 정확히 계산 후 오일러 변환

    Returns:
        delta_angles: [roll, pitch, yaw] 도 단위
    """
    converter = PoseConverter()

    if use_quaternion:
        # 쿼터니언 기반 정확한 계산
        _, q_prev = converter.pose_to_quaternion(prev_pose)
        _, q_curr = converter.pose_to_quaternion(curr_pose)

        # 회전 차이 계산: R_diff = R_curr * R_prev^(-1)
        rot_prev = Rotation.from_quat(q_prev.to_array())
        rot_curr = Rotation.from_quat(q_curr.to_array())

        rot_diff = rot_curr * rot_prev.inv()
        delta_euler = rot_diff.as_euler('xyz', degrees=True)
    else:
        # 오일러 기반 직접 계산 (짐벌 락 근처에서 부정확)
        _, prev_euler = converter.pose_to_euler(prev_pose)
        _, curr_euler = converter.pose_to_euler(curr_pose)

        delta_euler = np.array([
            curr_euler.roll - prev_euler.roll,
            curr_euler.pitch - prev_euler.pitch,
            curr_euler.yaw - prev_euler.yaw
        ])

    # -180 ~ 180 범위로 정규화
    delta_euler = np.where(delta_euler > 180, delta_euler - 360, delta_euler)
    delta_euler = np.where(delta_euler < -180, delta_euler + 360, delta_euler)

    return delta_euler


def rotation_matrix_to_6d(R: np.ndarray) -> np.ndarray:
    """
    회전 행렬에서 6D 연속 표현 추출 (신경망 학습용)

    Zhou et al. (CVPR 2019):
    "On the Continuity of Rotation Representations in Neural Networks"

    6D 표현은 신경망 학습에서 불연속성 문제가 없습니다.
    첫 두 열 벡터를 연결: [col1(3), col2(3)] = 6개 값
    """
    # Column-major order로 flatten하여 첫 두 열 벡터를 연결
    col1 = R[:, 0]  # 첫 번째 열
    col2 = R[:, 1]  # 두 번째 열
    return np.concatenate([col1, col2])  # [r1x, r1y, r1z, r2x, r2y, r2z]


def sixd_to_rotation_matrix(sixd: np.ndarray) -> np.ndarray:
    """
    6D 표현에서 회전 행렬 복원 (Gram-Schmidt 직교화)

    입력: [r1x, r1y, r1z, r2x, r2y, r2z] - 첫 두 열 벡터
    출력: 3x3 회전 행렬
    """
    a1 = sixd[:3]  # 첫 번째 열 벡터
    a2 = sixd[3:6]  # 두 번째 열 벡터

    # 정규화 (첫 번째 열)
    b1 = a1 / np.linalg.norm(a1)

    # Gram-Schmidt 직교화 (두 번째 열)
    b2 = a2 - np.dot(b1, a2) * b1
    b2 = b2 / np.linalg.norm(b2)

    # 세 번째 열 = 외적 (오른손 좌표계)
    b3 = np.cross(b1, b2)

    # 열 벡터로 회전 행렬 구성
    R = np.column_stack([b1, b2, b3])
    return R
