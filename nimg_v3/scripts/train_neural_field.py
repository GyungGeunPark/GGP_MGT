#!/usr/bin/env python3
"""
train_neural_field.py - Neural Object Field 학습 스크립트

참조 이미지로부터 Neural Object Field를 학습합니다.
Model-Free FoundationPose의 핵심 전처리 단계입니다.

사용법:
    python scripts/train_neural_field.py \
        --ref_dir /root/fursys_img_251229/extraction \
        --output_dir models/neural_fields/painting_object

Author: FurSys AI Team
"""

import argparse
import logging
import sys
from pathlib import Path

# nimg_v3 모듈 경로 추가
sys.path.insert(0, str(Path(__file__).parents[1]))

from nimg_v3.pose.reference_image_loader import ReferenceImageLoader
from nimg_v3.pose.neural_object_field import NeuralObjectField, NeuralFieldConfig

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def main():
    parser = argparse.ArgumentParser(description='Train Neural Object Field')

    parser.add_argument(
        '--ref_dir',
        type=str,
        required=True,
        help='참조 이미지 디렉토리'
    )
    parser.add_argument(
        '--output_dir',
        type=str,
        required=True,
        help='출력 디렉토리'
    )
    parser.add_argument(
        '--num_iterations',
        type=int,
        default=1000,
        help='학습 반복 횟수'
    )
    parser.add_argument(
        '--device',
        type=str,
        default='cuda:0',
        help='학습 디바이스'
    )
    parser.add_argument(
        '--generate_masks',
        action='store_true',
        help='Depth 기반 마스크 자동 생성'
    )

    args = parser.parse_args()

    # 1. 참조 이미지 로드
    logger.info(f"Loading reference images from {args.ref_dir}")
    loader = ReferenceImageLoader(
        base_dir=args.ref_dir,
        exclude_patterns=['_test']
    )

    ref_set = loader.load_all_views(images_per_view=1, sample_strategy='middle')
    logger.info(f"Loaded {len(ref_set)} reference images")
    logger.info(f"Available angles: {ref_set.view_angles}")

    # 2. 마스크 생성 (선택)
    if args.generate_masks:
        logger.info("Generating masks from depth...")
        ref_set = loader.generate_masks_from_depth(ref_set)

    # 3. Neural Field 학습
    logger.info("Training Neural Object Field...")
    config = NeuralFieldConfig(num_iterations=args.num_iterations)
    nof = NeuralObjectField(config=config, device=args.device)

    stats = nof.train(ref_set, verbose=True)

    logger.info(f"Training completed: {stats}")

    # 4. 메시 추출
    logger.info("Extracting mesh...")
    try:
        mesh = nof.extract_mesh()
        if mesh is not None:
            logger.info(f"Mesh extracted: {len(mesh.vertices)} vertices")
    except Exception as e:
        logger.warning(f"Mesh extraction failed: {e}")

    # 5. 저장
    logger.info(f"Saving to {args.output_dir}")
    nof.save(args.output_dir)

    logger.info("Done!")


if __name__ == '__main__':
    main()
