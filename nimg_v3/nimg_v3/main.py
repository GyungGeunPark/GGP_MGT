"""
main.py - nimg_v3 통합 측정 시스템

YOLO + FoundationPose + Kalman Filter 파이프라인을 통합하여
6DoF 자세 추정 및 속도 측정을 수행합니다.

파이프라인:
1. YOLO로 객체 탐지 및 마스크 생성
2. FoundationPose로 6DoF 자세 추정/추적
3. Kalman Filter로 속도 추정
4. 기준 프레임 대비 변화량 계산

Version: 1.0
Author: FurSys AI Team
Reference: nimg_v3_foundationpose_comprehensive_design_guide.md
"""

import numpy as np
from typing import Dict, Optional, Any, List
from dataclasses import dataclass, asdict
import time
import logging
from pathlib import Path

from .config.system_config import SystemConfig, load_config
from .measurement.pose_converter import (
    PoseConverter, EulerAngles, Quaternion, compute_angle_change
)
from .measurement.pose_kalman_filter import (
    PoseKalmanFilter, PoseKalmanState, FilterMode
)
from .pose.foundationpose_estimator import (
    FoundationPoseEstimator, PoseMode, TrackingState, PoseResult
)

logger = logging.getLogger(__name__)


@dataclass
class MeasurementResult:
    """
    측정 결과

    단일 프레임에서 추출한 모든 측정 정보를 담습니다.
    """
    # 탐지 정보
    detection_bbox: tuple          # (x1, y1, x2, y2)
    detection_confidence: float
    class_id: int
    class_name: str

    # 자세 정보
    pose_matrix: np.ndarray        # 4x4
    translation: np.ndarray        # [x, y, z] 미터
    euler_angles: EulerAngles      # roll, pitch, yaw (도)
    quaternion: Quaternion         # 쿼터니언
    pose_confidence: float
    tracking_state: str

    # Kalman Filter 상태
    filtered_position: np.ndarray  # [x, y, z]
    velocity: np.ndarray           # [vx, vy, vz] m/s
    speed: float                   # m/s
    angular_velocity: np.ndarray   # [wx, wy, wz] deg/s

    # 기준 대비 변화량
    position_change: Optional[np.ndarray]  # [dx, dy, dz]
    angle_change: Optional[np.ndarray]     # [droll, dpitch, dyaw]

    # 메타데이터
    frame_idx: int
    timestamp: float
    processing_time_ms: float

    def to_dict(self) -> Dict[str, Any]:
        """딕셔너리로 변환"""
        return {
            'detection': {
                'bbox': self.detection_bbox,
                'confidence': self.detection_confidence,
                'class_id': self.class_id,
                'class_name': self.class_name
            },
            'pose': {
                'translation': self.translation.tolist() if isinstance(self.translation, np.ndarray) else self.translation,
                'euler': {
                    'roll': self.euler_angles.roll,
                    'pitch': self.euler_angles.pitch,
                    'yaw': self.euler_angles.yaw
                },
                'quaternion': {
                    'x': self.quaternion.x,
                    'y': self.quaternion.y,
                    'z': self.quaternion.z,
                    'w': self.quaternion.w
                },
                'confidence': self.pose_confidence,
                'tracking_state': self.tracking_state
            },
            'kalman': {
                'position': self.filtered_position.tolist() if isinstance(self.filtered_position, np.ndarray) else self.filtered_position,
                'velocity': self.velocity.tolist() if isinstance(self.velocity, np.ndarray) else self.velocity,
                'speed': self.speed,
                'angular_velocity': self.angular_velocity.tolist() if isinstance(self.angular_velocity, np.ndarray) else self.angular_velocity
            },
            'change': {
                'position': self.position_change.tolist() if self.position_change is not None else None,
                'angle': self.angle_change.tolist() if self.angle_change is not None else None
            },
            'meta': {
                'frame_idx': self.frame_idx,
                'timestamp': self.timestamp,
                'processing_time_ms': self.processing_time_ms
            }
        }


class IntegratedMeasurementSystem:
    """
    nimg_v3 통합 측정 시스템

    YOLO + FoundationPose + Kalman Filter를 통합하여
    6DoF 자세 추정 및 속도 측정을 수행합니다.

    Example:
        >>> config = load_config("config/system_config.yaml")
        >>> system = IntegratedMeasurementSystem.from_config(config)
        >>> result = system.process_frame(rgb, depth)
        >>> print(f"Position: {result.translation}")
        >>> print(f"Yaw: {result.euler_angles.yaw:.2f}")
    """

    def __init__(
        self,
        yolo_model_path: str,
        foundationpose_model_dir: str,
        pose_mode: PoseMode = PoseMode.MODEL_FREE,
        mesh_path: Optional[str] = None,
        neural_field_dir: Optional[str] = None,
        intrinsics: Optional[Dict[str, float]] = None,
        reference_frame_idx: int = 0,
        fps: float = 30.0,
        device: str = 'cuda:0',
        kalman_mode: FilterMode = FilterMode.QUATERNION
    ):
        """
        Args:
            yolo_model_path: YOLO 모델 경로
            foundationpose_model_dir: FoundationPose 모델 디렉토리
            pose_mode: Model-Based 또는 Model-Free
            mesh_path: CAD 모델 경로 (Model-Based)
            neural_field_dir: Neural Field 경로 (Model-Free)
            intrinsics: 카메라 내부 파라미터
            reference_frame_idx: 기준 프레임 인덱스
            fps: 프레임 레이트
            device: 추론 디바이스
            kalman_mode: Kalman Filter 모드
        """
        self.device = device
        self.fps = fps
        self.reference_frame_idx = reference_frame_idx

        # 카메라 파라미터
        self.intrinsics = intrinsics or {
            'fx': 383.883, 'fy': 383.883,
            'cx': 320.499, 'cy': 237.913
        }

        # 컴포넌트 초기화
        self._detector = None
        self._pose_estimator = None
        self._kalman_filter = None
        self._pose_converter = None

        # YOLO 탐지기 (지연 로딩)
        self._yolo_model_path = yolo_model_path

        # FoundationPose 추정기 (지연 로딩)
        self._foundationpose_config = {
            'model_dir': foundationpose_model_dir,
            'mode': pose_mode,
            'mesh_path': mesh_path,
            'neural_field_dir': neural_field_dir,
            'device': device
        }

        # Kalman Filter
        self._kalman_filter = PoseKalmanFilter(
            dt=1.0 / fps,
            mode=kalman_mode
        )

        # Pose Converter
        self._pose_converter = PoseConverter()

        # 기준 프레임 관리
        self._reference_pose = None
        self._reference_euler = None
        self._reference_set = False

        # 프레임 카운터
        self._frame_count = 0

        logger.info("IntegratedMeasurementSystem initialized")

    def _init_detector(self):
        """YOLO 탐지기 지연 초기화"""
        if self._detector is not None:
            return

        try:
            # nimg_v2의 YOLODetector 사용 시도
            import sys
            sys.path.insert(0, str(Path(__file__).parents[2] / 'nimg_v2' / 'nimg_v2'))
            from detection.yolo_detector import YOLODetector

            self._detector = YOLODetector(
                model_path=self._yolo_model_path,
                conf_threshold=0.5,
                device=self.device
            )
            logger.info("YOLODetector initialized")
        except ImportError as e:
            logger.warning(f"YOLODetector import failed: {e}")
            logger.info("Using placeholder detector")
            self._detector = None

    def _init_pose_estimator(self):
        """FoundationPose 추정기 지연 초기화"""
        if self._pose_estimator is not None:
            return

        try:
            self._pose_estimator = FoundationPoseEstimator(**self._foundationpose_config)
            logger.info("FoundationPoseEstimator initialized")
        except Exception as e:
            logger.error(f"Failed to initialize pose estimator: {e}")
            raise

    def process_frame(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        timestamp: Optional[float] = None,
        detection: Optional[Any] = None
    ) -> Optional[MeasurementResult]:
        """
        단일 프레임 처리

        Args:
            rgb: RGB 이미지 [H, W, 3]
            depth: Depth 이미지 [H, W] (미터 단위)
            timestamp: 프레임 타임스탬프
            detection: 사전 탐지된 객체 (None이면 YOLO 사용)

        Returns:
            MeasurementResult: 측정 결과 (탐지 실패 시 None)
        """
        start_time = time.time()
        frame_idx = self._frame_count
        self._frame_count += 1

        if timestamp is None:
            timestamp = frame_idx / self.fps

        # 1. 객체 탐지 (또는 사전 제공된 탐지 사용)
        if detection is None:
            detection = self._detect_object(rgb)

        if detection is None:
            logger.debug(f"Frame {frame_idx}: No detection")
            return None

        # 탐지 정보 추출
        bbox = (detection.x, detection.y, detection.x2, detection.y2)

        # 2. 마스크 생성
        mask = self._create_mask(rgb.shape[:2], bbox)

        # 3. 자세 추정 초기화 (필요시)
        if self._pose_estimator is None:
            self._init_pose_estimator()

        # 4. FoundationPose 자세 추정/추적
        force_estimate = (
            frame_idx == self.reference_frame_idx or
            not self._reference_set or
            not self._pose_estimator.is_tracking
        )

        pose_result = self._pose_estimator.process(
            rgb=rgb,
            depth=depth,
            mask=mask if force_estimate else None,
            intrinsics=self.intrinsics,
            force_estimate=force_estimate
        )

        # 5. 자세 변환 (오일러 + 쿼터니언)
        components = self._pose_converter.pose_matrix_to_components(pose_result.pose_matrix)

        # 6. Kalman Filter 업데이트
        kalman_state = self._kalman_filter.predict_and_update(
            position=components.translation,
            orientation=components.quaternion
        )

        # 거리 기반 적응형 노이즈 업데이트
        depth_distance = float(np.mean(depth[mask > 0]) if np.any(mask > 0) else 1.0)
        self._kalman_filter.update_adaptive_noise(depth_distance)

        # 7. 기준 프레임 설정 또는 변화량 계산
        position_change = None
        angle_change = None

        if frame_idx == self.reference_frame_idx:
            self._set_reference(pose_result.pose_matrix, components.euler)
        elif self._reference_set:
            position_change = components.translation - self._reference_pose[:3, 3]
            angle_change = compute_angle_change(
                self._reference_pose,
                pose_result.pose_matrix,
                use_quaternion=True
            )

        processing_time = (time.time() - start_time) * 1000

        return MeasurementResult(
            # 탐지 정보
            detection_bbox=bbox,
            detection_confidence=detection.confidence,
            class_id=detection.class_id,
            class_name=detection.class_name,

            # 자세 정보
            pose_matrix=pose_result.pose_matrix,
            translation=components.translation,
            euler_angles=components.euler,
            quaternion=components.quaternion,
            pose_confidence=pose_result.confidence,
            tracking_state=pose_result.tracking_state.value,

            # Kalman 상태
            filtered_position=kalman_state.position,
            velocity=kalman_state.velocity,
            speed=kalman_state.speed,
            angular_velocity=kalman_state.angular_velocity,

            # 변화량
            position_change=position_change,
            angle_change=angle_change,

            # 메타
            frame_idx=frame_idx,
            timestamp=timestamp,
            processing_time_ms=processing_time
        )

    def _detect_object(self, rgb: np.ndarray) -> Optional[Any]:
        """객체 탐지"""
        if self._detector is None:
            self._init_detector()

        if self._detector is None:
            # 더미 탐지 (전체 이미지)
            from dataclasses import dataclass

            @dataclass
            class DummyDetection:
                x: int = 100
                y: int = 100
                x2: int = 540
                y2: int = 380
                confidence: float = 0.9
                class_id: int = 0
                class_name: str = "object"

            return DummyDetection()

        detections = self._detector.detect(rgb)
        if not detections:
            return None

        # 가장 신뢰도 높은 탐지 반환
        return max(detections, key=lambda d: d.confidence)

    def _create_mask(
        self,
        image_shape: tuple,
        bbox: tuple
    ) -> np.ndarray:
        """바운딩 박스에서 마스크 생성"""
        mask = np.zeros(image_shape, dtype=np.uint8)
        x1, y1, x2, y2 = bbox
        mask[y1:y2, x1:x2] = 255
        return mask

    def _set_reference(self, pose_matrix: np.ndarray, euler_angles: EulerAngles):
        """기준 프레임 설정"""
        self._reference_pose = pose_matrix.copy()
        self._reference_euler = euler_angles
        self._reference_set = True
        logger.info(f"Reference frame set:")
        logger.info(f"  Position: [{pose_matrix[0,3]:.3f}, {pose_matrix[1,3]:.3f}, {pose_matrix[2,3]:.3f}]")
        logger.info(f"  Orientation: R={euler_angles.roll:.2f}, P={euler_angles.pitch:.2f}, Y={euler_angles.yaw:.2f}")

    def process_sequence(
        self,
        frames: List[Dict[str, np.ndarray]]
    ) -> List[Optional[MeasurementResult]]:
        """
        프레임 시퀀스 처리

        Args:
            frames: [{'rgb': array, 'depth': array, 'timestamp': float}, ...]

        Returns:
            측정 결과 리스트
        """
        results = []

        for frame in frames:
            result = self.process_frame(
                rgb=frame['rgb'],
                depth=frame['depth'],
                timestamp=frame.get('timestamp')
            )
            results.append(result)

        return results

    def reset(self):
        """시스템 리셋"""
        if self._pose_estimator is not None:
            self._pose_estimator.reset()

        self._kalman_filter.reset()
        self._reference_pose = None
        self._reference_euler = None
        self._reference_set = False
        self._frame_count = 0

        logger.info("System reset")

    @property
    def is_reference_set(self) -> bool:
        return self._reference_set

    @property
    def is_tracking(self) -> bool:
        if self._pose_estimator is None:
            return False
        return self._pose_estimator.is_tracking

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @classmethod
    def from_config(cls, config: SystemConfig) -> 'IntegratedMeasurementSystem':
        """
        설정에서 시스템 생성

        Args:
            config: SystemConfig

        Returns:
            IntegratedMeasurementSystem
        """
        pose_mode = (
            PoseMode.MODEL_BASED
            if config.pose_estimation.mode == "model_based"
            else PoseMode.MODEL_FREE
        )

        kalman_mode = (
            FilterMode.QUATERNION
            if config.kalman_filter.mode == "quaternion"
            else FilterMode.EULER
        )

        return cls(
            yolo_model_path=config.detection.model_path,
            foundationpose_model_dir=config.pose_estimation.model_dir,
            pose_mode=pose_mode,
            mesh_path=config.pose_estimation.mesh_path,
            neural_field_dir=config.pose_estimation.neural_field_dir,
            intrinsics=config.camera.to_intrinsics_dict(),
            reference_frame_idx=config.reference_frame_idx,
            fps=config.camera.fps,
            device=config.device,
            kalman_mode=kalman_mode
        )


def run_measurement(
    rgb: np.ndarray,
    depth: np.ndarray,
    config: Optional[SystemConfig] = None
) -> Optional[MeasurementResult]:
    """
    단일 프레임 측정 (편의 함수)

    Args:
        rgb: RGB 이미지
        depth: Depth 이미지
        config: 시스템 설정

    Returns:
        측정 결과
    """
    if config is None:
        config = SystemConfig()

    system = IntegratedMeasurementSystem.from_config(config)
    return system.process_frame(rgb, depth)
