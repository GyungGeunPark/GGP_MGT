"""
system_config.py - 시스템 설정 관리

nimg_v3 시스템의 모든 설정을 통합 관리합니다.

Version: 1.0
Author: FurSys AI Team
"""

import yaml
from pathlib import Path
from dataclasses import dataclass, field
from typing import Dict, Any, Optional
import logging

logger = logging.getLogger(__name__)


@dataclass
class CameraConfig:
    """카메라 설정"""
    # RealSense D455 기본 파라미터
    fx: float = 383.883
    fy: float = 383.883
    cx: float = 320.499
    cy: float = 237.913

    # 이미지 크기
    width: int = 640
    height: int = 480

    # Depth 설정
    depth_scale: float = 0.001  # 미터 변환
    depth_min: float = 0.1     # 최소 거리 (m)
    depth_max: float = 10.0    # 최대 거리 (m)

    # 프레임레이트
    fps: float = 30.0

    def to_intrinsics_dict(self) -> Dict[str, float]:
        """카메라 내부 파라미터 딕셔너리"""
        return {
            'fx': self.fx,
            'fy': self.fy,
            'cx': self.cx,
            'cy': self.cy
        }


@dataclass
class DetectionConfig:
    """객체 탐지 설정"""
    model_path: str = "models/yolo/class187_image85286_v12x_250epochs.pt"
    conf_threshold: float = 0.5
    iou_threshold: float = 0.45
    max_detections: int = 100
    img_size: int = 640
    half_precision: bool = True


@dataclass
class PoseEstimationConfig:
    """자세 추정 설정"""
    # FoundationPose 모델
    model_dir: str = "models/foundationpose"
    mode: str = "model_free"  # "model_based" or "model_free"

    # Model-Based
    mesh_path: Optional[str] = None

    # Model-Free
    neural_field_dir: Optional[str] = "models/neural_fields/painting_object"
    reference_images_dir: Optional[str] = None

    # 추정 설정
    use_tensorrt: bool = True
    refine_iterations: int = 5

    # 추적 설정
    tracking_recovery_threshold: float = 0.3
    max_lost_frames: int = 5


@dataclass
class KalmanFilterConfig:
    """Kalman Filter 설정"""
    mode: str = "quaternion"  # "euler" or "quaternion"

    # 프로세스 노이즈
    process_noise_pos: float = 0.01
    process_noise_vel: float = 0.1
    process_noise_orient: float = 0.1
    process_noise_angular_vel: float = 1.0

    # 측정 노이즈
    measurement_noise_pos: float = 0.005
    measurement_noise_orient: float = 0.5

    # 적응형 노이즈
    adaptive_noise: bool = True


@dataclass
class OutputConfig:
    """출력 설정"""
    # 저장 옵션
    save_results: bool = True
    output_dir: str = "output"
    output_format: str = "csv"  # "csv" or "json"

    # 시각화
    visualize: bool = True
    save_visualization: bool = False

    # 로깅
    log_level: str = "INFO"
    log_to_file: bool = False


@dataclass
class SystemConfig:
    """nimg_v3 시스템 전체 설정"""
    camera: CameraConfig = field(default_factory=CameraConfig)
    detection: DetectionConfig = field(default_factory=DetectionConfig)
    pose_estimation: PoseEstimationConfig = field(default_factory=PoseEstimationConfig)
    kalman_filter: KalmanFilterConfig = field(default_factory=KalmanFilterConfig)
    output: OutputConfig = field(default_factory=OutputConfig)

    # 추가 설정
    device: str = "cuda:0"
    reference_frame_idx: int = 0

    def save(self, filepath: str):
        """설정을 YAML 파일로 저장"""
        config_dict = {
            'camera': self.camera.__dict__,
            'detection': self.detection.__dict__,
            'pose_estimation': self.pose_estimation.__dict__,
            'kalman_filter': self.kalman_filter.__dict__,
            'output': self.output.__dict__,
            'device': self.device,
            'reference_frame_idx': self.reference_frame_idx
        }

        with open(filepath, 'w') as f:
            yaml.dump(config_dict, f, default_flow_style=False)

        logger.info(f"Config saved to {filepath}")

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> 'SystemConfig':
        """딕셔너리에서 설정 생성"""
        return cls(
            camera=CameraConfig(**d.get('camera', {})),
            detection=DetectionConfig(**d.get('detection', {})),
            pose_estimation=PoseEstimationConfig(**d.get('pose_estimation', {})),
            kalman_filter=KalmanFilterConfig(**d.get('kalman_filter', {})),
            output=OutputConfig(**d.get('output', {})),
            device=d.get('device', 'cuda:0'),
            reference_frame_idx=d.get('reference_frame_idx', 0)
        )


def load_config(filepath: str) -> SystemConfig:
    """
    YAML 파일에서 설정 로드

    Args:
        filepath: 설정 파일 경로

    Returns:
        SystemConfig: 로드된 설정
    """
    path = Path(filepath)

    if not path.exists():
        logger.warning(f"Config file not found: {filepath}, using defaults")
        return SystemConfig()

    with open(path, 'r') as f:
        config_dict = yaml.safe_load(f)

    if config_dict is None:
        return SystemConfig()

    return SystemConfig.from_dict(config_dict)


def create_default_config(save_path: Optional[str] = None) -> SystemConfig:
    """
    기본 설정 생성

    Args:
        save_path: 저장 경로 (None이면 저장 안함)

    Returns:
        SystemConfig: 기본 설정
    """
    config = SystemConfig()

    if save_path:
        config.save(save_path)

    return config
