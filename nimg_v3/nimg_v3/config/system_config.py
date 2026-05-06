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
class RecognitionConfig:
    """SAM + DINOv3 + NOCTIS 인식 파이프라인 설정 (architecture §8)."""
    backend: str = "noctis"                    # "noctis" | "cnos" | "yolo" | "depth_bbox" | "disabled"
    sam_version: str = "sam3.1"                # "sam3.1" | "sam2.1" | "disabled"
    sam_cfg: str = "configs/sam3.1/sam3.1_hiera_l.yaml"
    sam_ckpt: str = ""                         # empty → auto fallback chain
    sam_points_per_side: int = 16
    sam_use_concept_prompt: bool = True
    concept_prompts: Dict[int, str] = field(default_factory=lambda: {
        0: "metallic housing on conveyor",
        1: "wiring tray on conveyor",
    })
    encoder: str = "dinov3-vitl16"             # "dinov3-vitl16" | "dinov3-vitb16" |
                                               # "dinov2-large" | "hsvhist" (fallback)
    template_db_root: str = "src/nimg_v3/models/neural_fields"
    template_subdir: str = "templates"
    match_threshold: float = 0.35
    cyclic_threshold_step: float = 0.05        # NOCTIS 전용
    nms_iou: float = 0.5
    roi: Optional[tuple] = None                # (x1, y1, x2, y2) or None
    # YOLO fallback (Tier-1)
    yolo_fallback_path: str = "src/nimg_v3/models/yolo/yolo26_2class_seg_best_260324.pt"
    yolo_conf: float = 0.5


@dataclass
class V6OptimizationFlags:
    """v6 (Top-5 최적화) 토글 — 모두 False = v5.2 baseline.

    설계 문서: research/260420_fp_top5_implementation_design.md §1
    """
    # Rank 1: depth fidelity
    use_raw_depth: bool = False              # RawDepthReader 사용 (16-bit PNG/NPZ)
    depth_denoise_bilateral: bool = False    # PairedVideoSource 의 best-effort 보강
    use_realsense_filters: bool = True       # librealsense 후처리 (live mode default ON)

    # Rank 2: mesh
    use_textured_mesh: bool = False          # Part_02_textured.obj 우선 로드
    use_symmetry_tfs: bool = False           # reference_config.yaml 의 symmetry 적용
    require_watertight: bool = False         # 메쉬가 watertight 아니면 warn (true 면 raise)

    # Rank 3: KF — 현재 KF 가 normalize+sign 적용 → 우선순위 낮음
    use_mekf: bool = False                   # 미구현 placeholder (deferred)

    # Rank 4: hierarchical refine
    use_hierarchical_refine: bool = False
    hierarchical_n_hyp: int = 2
    hierarchical_sigma_rot_deg: float = 15.0

    # Rank 5: small wins
    use_per_device_K: bool = True            # RealSense 자동 (이미 적용)
    track_refine_iter_v6: int = 5            # 3 → 5
    render_resolution: int = 160             # 160 → 224 시도


@dataclass
class TrackerConfig:
    """FoundationPose + FP++ 추적 설정 (architecture §8)."""
    backend: str = "fp_plus_plus"              # "fp_plus_plus" | "fp" | "pca"
    tracker_2d: str = "iou"                    # "dam4sam" | "him2sam" | "samurai" |
                                               # "cutie" | "ostrack" | "iou"
    tracker_2d_ckpt: str = ""                  # empty → auto-detect
    track_refine_iter: int = 3
    est_refine_iter: int = 8
    kf_measurement_noise_scale: float = 0.05
    lost_score_threshold: float = 0.4
    lost_score_frames: int = 3
    periodic_reinit_frames: int = 150
    use_fast_foundation_stereo: bool = False
    rgb_only_fallback: str = "disabled"        # "disabled" | "gotrack" | "rgbtrack" | "conceptpose"
    fp_weights_root: str = "src/nimg_v3/models/foundationpose"


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
    recognition: RecognitionConfig = field(default_factory=RecognitionConfig)
    tracker: TrackerConfig = field(default_factory=TrackerConfig)
    optim: V6OptimizationFlags = field(default_factory=V6OptimizationFlags)
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
