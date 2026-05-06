#!/usr/bin/env python3
"""
nimg_v3 Pipeline Test Script

테스트 이미지에서 YOLO 검출 및 FoundationPose 자세 추정 파이프라인 테스트
결과를 시각화하여 test_result 폴더에 저장
"""

import os
import sys
import cv2
import numpy as np
import json
import logging
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional, Tuple
import time

# 상위 경로 추가
sys.path.insert(0, str(Path(__file__).parent.parent))

from ultralytics import YOLO

# nimg_v3 모듈
from nimg_v3.measurement.pose_converter import PoseConverter, EulerAngles, Quaternion
from nimg_v3.measurement.pose_kalman_filter import PoseKalmanFilter, FilterMode
from nimg_v3.pose.reference_image_loader import ReferenceImageLoader

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class PipelineTestRunner:
    """nimg_v3 파이프라인 테스트 러너"""

    def __init__(
        self,
        yolo_model_path: str,
        test_images_dir: str,
        reference_images_dir: str,
        output_dir: str,
        max_images: int = 100,
        confidence_threshold: float = 0.5
    ):
        self.yolo_model_path = Path(yolo_model_path)
        self.test_images_dir = Path(test_images_dir)
        self.reference_images_dir = Path(reference_images_dir)
        self.output_dir = Path(output_dir)
        self.max_images = max_images
        self.confidence_threshold = confidence_threshold

        # 출력 디렉토리 생성
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'detections').mkdir(exist_ok=True)
        (self.output_dir / 'poses').mkdir(exist_ok=True)
        (self.output_dir / 'visualizations').mkdir(exist_ok=True)

        # 결과 저장
        self.results = {
            'test_info': {},
            'detection_results': [],
            'pose_results': [],
            'statistics': {}
        }

        # 모델 로드
        self._load_models()

    def _load_models(self):
        """모델 로드"""
        logger.info(f"Loading YOLO model from {self.yolo_model_path}")

        if not self.yolo_model_path.exists():
            raise FileNotFoundError(f"YOLO model not found: {self.yolo_model_path}")

        self.yolo_model = YOLO(str(self.yolo_model_path))
        logger.info("YOLO model loaded successfully")

        # Kalman Filter 초기화
        self.kalman_filter = PoseKalmanFilter(dt=1/30.0, mode=FilterMode.QUATERNION)
        self.pose_converter = PoseConverter()

        logger.info("Pipeline components initialized")

    def _get_test_images(self) -> List[Path]:
        """테스트 이미지 목록 가져오기"""
        rgb_dir = self.test_images_dir / 'rgb'

        if not rgb_dir.exists():
            raise FileNotFoundError(f"RGB directory not found: {rgb_dir}")

        images = sorted([
            f for f in rgb_dir.iterdir()
            if f.suffix.lower() in ['.png', '.jpg', '.jpeg']
        ])

        # 균등 샘플링
        if len(images) > self.max_images:
            step = len(images) // self.max_images
            images = images[::step][:self.max_images]

        logger.info(f"Found {len(images)} test images")
        return images

    def _get_depth_image(self, rgb_path: Path) -> Optional[np.ndarray]:
        """RGB 이미지에 대응하는 depth 이미지 로드"""
        # rgb 파일명에서 타임스탬프 추출
        rgb_name = rgb_path.stem
        depth_dir = self.test_images_dir / 'depth'

        if not depth_dir.exists():
            return None

        # depth 파일 찾기
        for depth_file in depth_dir.iterdir():
            if depth_file.suffix.lower() == '.png':
                # 타임스탬프 매칭 시도
                depth_name = depth_file.stem
                # 같은 인덱스 사용
                depth_img = cv2.imread(str(depth_file), cv2.IMREAD_UNCHANGED)
                if depth_img is not None:
                    if depth_img.ndim == 3:
                        depth_img = cv2.cvtColor(depth_img, cv2.COLOR_BGR2GRAY)
                    return depth_img.astype(np.float32) * 0.001  # mm to meters

        return None

    def run_detection(self, image: np.ndarray) -> List[Dict]:
        """YOLO 검출 실행"""
        results = self.yolo_model(image, verbose=False)

        detections = []
        for result in results:
            if result.boxes is not None:
                boxes = result.boxes
                for i in range(len(boxes)):
                    box = boxes[i]
                    detection = {
                        'bbox': box.xyxy[0].cpu().numpy().tolist(),
                        'confidence': float(box.conf[0].cpu().numpy()),
                        'class_id': int(box.cls[0].cpu().numpy()),
                        'class_name': self.yolo_model.names[int(box.cls[0])]
                    }
                    if detection['confidence'] >= self.confidence_threshold:
                        detections.append(detection)

        return detections

    def visualize_detection(
        self,
        image: np.ndarray,
        detections: List[Dict],
        output_path: Path
    ):
        """검출 결과 시각화"""
        vis_image = image.copy()

        for det in detections:
            bbox = det['bbox']
            x1, y1, x2, y2 = map(int, bbox)
            conf = det['confidence']
            class_name = det['class_name']

            # 바운딩 박스 그리기
            color = (0, 255, 0)  # 녹색
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, 2)

            # 라벨 그리기
            label = f"{class_name}: {conf:.2f}"
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)

            # 라벨 배경
            cv2.rectangle(
                vis_image,
                (x1, y1 - label_size[1] - 10),
                (x1 + label_size[0], y1),
                color,
                -1
            )
            cv2.putText(
                vis_image,
                label,
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                2
            )

        cv2.imwrite(str(output_path), vis_image)

    def run_test(self):
        """전체 테스트 실행"""
        start_time = time.time()

        # 테스트 정보 저장
        self.results['test_info'] = {
            'timestamp': datetime.now().isoformat(),
            'yolo_model': str(self.yolo_model_path),
            'test_images_dir': str(self.test_images_dir),
            'max_images': self.max_images,
            'confidence_threshold': self.confidence_threshold
        }

        # 테스트 이미지 가져오기
        test_images = self._get_test_images()

        total_detections = 0
        detection_times = []
        class_counts = {}

        logger.info(f"Starting test with {len(test_images)} images...")

        for idx, image_path in enumerate(test_images):
            try:
                # 이미지 로드
                image = cv2.imread(str(image_path))
                if image is None:
                    logger.warning(f"Failed to load image: {image_path}")
                    continue

                # 검출 실행
                det_start = time.time()
                detections = self.run_detection(image)
                det_time = time.time() - det_start
                detection_times.append(det_time)

                total_detections += len(detections)

                # 클래스별 카운트
                for det in detections:
                    class_name = det['class_name']
                    class_counts[class_name] = class_counts.get(class_name, 0) + 1

                # 결과 저장
                result = {
                    'image_path': str(image_path),
                    'image_name': image_path.name,
                    'detections': detections,
                    'detection_time': det_time,
                    'num_detections': len(detections)
                }
                self.results['detection_results'].append(result)

                # 시각화 저장 (처음 50개만)
                if idx < 50:
                    vis_path = self.output_dir / 'visualizations' / f"det_{idx:04d}_{image_path.stem}.jpg"
                    self.visualize_detection(image, detections, vis_path)

                # 진행상황 로깅
                if (idx + 1) % 10 == 0:
                    logger.info(f"Processed {idx + 1}/{len(test_images)} images")

            except Exception as e:
                logger.error(f"Error processing {image_path}: {e}")
                continue

        # 통계 계산
        total_time = time.time() - start_time
        avg_det_time = np.mean(detection_times) if detection_times else 0

        self.results['statistics'] = {
            'total_images': len(test_images),
            'total_detections': total_detections,
            'avg_detections_per_image': total_detections / len(test_images) if test_images else 0,
            'avg_detection_time': avg_det_time,
            'fps': 1 / avg_det_time if avg_det_time > 0 else 0,
            'total_test_time': total_time,
            'class_distribution': class_counts
        }

        logger.info(f"Test completed in {total_time:.2f}s")
        logger.info(f"Total detections: {total_detections}")
        logger.info(f"Average FPS: {self.results['statistics']['fps']:.2f}")

        # 결과 저장
        self._save_results()

    def _save_results(self):
        """결과 저장"""
        # JSON 결과 저장
        results_path = self.output_dir / 'test_results.json'
        with open(results_path, 'w', encoding='utf-8') as f:
            json.dump(self.results, f, indent=2, ensure_ascii=False)
        logger.info(f"Results saved to {results_path}")

        # 로그 파일 저장
        log_path = self.output_dir / 'test_log.txt'
        with open(log_path, 'w', encoding='utf-8') as f:
            f.write("=" * 60 + "\n")
            f.write("nimg_v3 Pipeline Test Results\n")
            f.write("=" * 60 + "\n\n")

            f.write(f"Test Time: {self.results['test_info']['timestamp']}\n")
            f.write(f"YOLO Model: {self.results['test_info']['yolo_model']}\n")
            f.write(f"Test Images: {self.results['test_info']['test_images_dir']}\n\n")

            f.write("Statistics:\n")
            f.write("-" * 40 + "\n")
            stats = self.results['statistics']
            f.write(f"Total Images Processed: {stats['total_images']}\n")
            f.write(f"Total Detections: {stats['total_detections']}\n")
            f.write(f"Avg Detections/Image: {stats['avg_detections_per_image']:.2f}\n")
            f.write(f"Avg Detection Time: {stats['avg_detection_time']*1000:.2f} ms\n")
            f.write(f"FPS: {stats['fps']:.2f}\n")
            f.write(f"Total Test Time: {stats['total_test_time']:.2f} s\n\n")

            f.write("Class Distribution:\n")
            f.write("-" * 40 + "\n")
            for class_name, count in sorted(stats['class_distribution'].items(), key=lambda x: -x[1]):
                f.write(f"  {class_name}: {count}\n")

        logger.info(f"Log saved to {log_path}")

        # 요약 이미지 생성
        self._create_summary_image()

    def _create_summary_image(self):
        """요약 이미지 생성"""
        # 통계 시각화
        stats = self.results['statistics']

        # 간단한 텍스트 요약 이미지
        summary = np.ones((400, 600, 3), dtype=np.uint8) * 255

        lines = [
            "nimg_v3 Pipeline Test Summary",
            "",
            f"Total Images: {stats['total_images']}",
            f"Total Detections: {stats['total_detections']}",
            f"Avg Detections/Image: {stats['avg_detections_per_image']:.2f}",
            f"Average FPS: {stats['fps']:.2f}",
            f"Total Time: {stats['total_test_time']:.2f}s",
            "",
            "Top Classes:",
        ]

        # 상위 5개 클래스 추가
        sorted_classes = sorted(
            stats['class_distribution'].items(),
            key=lambda x: -x[1]
        )[:5]
        for class_name, count in sorted_classes:
            lines.append(f"  {class_name}: {count}")

        # 텍스트 그리기
        y = 30
        for line in lines:
            if line.startswith("nimg_v3"):
                cv2.putText(summary, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
            else:
                cv2.putText(summary, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
            y += 30

        summary_path = self.output_dir / 'test_summary.png'
        cv2.imwrite(str(summary_path), summary)
        logger.info(f"Summary image saved to {summary_path}")


def main():
    """메인 함수"""
    # 경로 설정
    base_dir = Path(__file__).parent.parent

    yolo_model_path = base_dir / "models" / "yolo" / "class187_image85286_v12x_250epochs.pt"
    test_images_dir = Path("/root/fursys_img_251229/extraction/20251229_154835_test")
    reference_images_dir = Path("/root/fursys_img_251229/extraction")
    output_dir = base_dir / "test_result"

    # 테스트 러너 생성 및 실행
    runner = PipelineTestRunner(
        yolo_model_path=str(yolo_model_path),
        test_images_dir=str(test_images_dir),
        reference_images_dir=str(reference_images_dir),
        output_dir=str(output_dir),
        max_images=100,  # 100개 이미지 샘플링
        confidence_threshold=0.5
    )

    runner.run_test()

    print("\n" + "=" * 60)
    print("Test completed! Results saved to:", output_dir)
    print("=" * 60)


if __name__ == "__main__":
    main()
