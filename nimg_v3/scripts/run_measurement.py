#!/usr/bin/env python3
"""
run_measurement.py - nimg_v3 측정 실행 스크립트

오프라인 데이터 또는 실시간 카메라로부터 6DoF 자세를 측정합니다.

사용법:
    # 오프라인 데이터
    python scripts/run_measurement.py \
        --data_dir /path/to/data \
        --output_dir output

    # 참조 이미지 폴더에서 테스트
    python scripts/run_measurement.py \
        --data_dir /root/fursys_img_251229/extraction/20251229_093820_front

Author: FurSys AI Team
"""

import argparse
import logging
import sys
from pathlib import Path
import cv2
import numpy as np

# nimg_v3 모듈 경로 추가
sys.path.insert(0, str(Path(__file__).parents[1]))

from nimg_v3.main import IntegratedMeasurementSystem
from nimg_v3.config.system_config import SystemConfig, load_config
from nimg_v3.input.data_loader import DataLoader
from nimg_v3.output.result_exporter import ResultExporter
from nimg_v3.pose.foundationpose_estimator import PoseMode
from nimg_v3.measurement.pose_kalman_filter import FilterMode

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description='Run nimg_v3 Measurement')

    parser.add_argument(
        '--data_dir',
        type=str,
        required=True,
        help='데이터 디렉토리'
    )
    parser.add_argument(
        '--config',
        type=str,
        default=None,
        help='설정 파일 경로'
    )
    parser.add_argument(
        '--output_dir',
        type=str,
        default='output',
        help='출력 디렉토리'
    )
    parser.add_argument(
        '--visualize',
        action='store_true',
        help='결과 시각화'
    )
    parser.add_argument(
        '--max_frames',
        type=int,
        default=None,
        help='최대 처리 프레임 수'
    )

    args = parser.parse_args()

    # 설정 로드
    if args.config:
        config = load_config(args.config)
    else:
        config = SystemConfig()

    # 데이터 로더
    logger.info(f"Loading data from {args.data_dir}")
    data_loader = DataLoader(args.data_dir)
    logger.info(f"Found {len(data_loader)} frames")

    # 측정 시스템 초기화
    logger.info("Initializing measurement system...")
    system = IntegratedMeasurementSystem(
        yolo_model_path=config.detection.model_path,
        foundationpose_model_dir=config.pose_estimation.model_dir,
        pose_mode=PoseMode.MODEL_FREE,
        neural_field_dir=config.pose_estimation.neural_field_dir,
        intrinsics=config.camera.to_intrinsics_dict(),
        fps=config.camera.fps,
        device=config.device
    )

    # 결과 내보내기
    exporter = ResultExporter(args.output_dir)

    # 처리
    max_frames = args.max_frames or len(data_loader)
    max_frames = min(max_frames, len(data_loader))

    logger.info(f"Processing {max_frames} frames...")

    for i in range(max_frames):
        frame = data_loader.load_frame(i)

        result = system.process_frame(
            rgb=frame.rgb,
            depth=frame.depth,
            timestamp=frame.timestamp
        )

        if result is not None:
            exporter.add_result(result)

            # 로그 출력
            if i % 10 == 0 or i == max_frames - 1:
                logger.info(
                    f"Frame {i}: "
                    f"pos=[{result.translation[0]:.3f}, {result.translation[1]:.3f}, {result.translation[2]:.3f}], "
                    f"euler=[{result.euler_angles.roll:.1f}, {result.euler_angles.pitch:.1f}, {result.euler_angles.yaw:.1f}], "
                    f"speed={result.speed:.3f} m/s"
                )

            # 시각화
            if args.visualize:
                vis = frame.rgb.copy()
                x1, y1, x2, y2 = result.detection_bbox
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

                text = f"R:{result.euler_angles.roll:.1f} P:{result.euler_angles.pitch:.1f} Y:{result.euler_angles.yaw:.1f}"
                cv2.putText(vis, text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                cv2.imshow('nimg_v3', vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    # 저장
    filepath = exporter.save()
    logger.info(f"Results saved to {filepath}")

    # 요약
    summary = exporter.get_summary()
    logger.info(f"Summary: {summary}")

    if args.visualize:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
