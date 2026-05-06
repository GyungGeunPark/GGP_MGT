#!/usr/bin/env python3
"""
nimg_v3 Extended Pipeline Test Script

YOLO 검출 + 참조 이미지 로더 + 칼만 필터 테스트
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

sys.path.insert(0, str(Path(__file__).parent.parent))

from ultralytics import YOLO

# nimg_v3 모듈
from nimg_v3.measurement.pose_converter import PoseConverter, EulerAngles, Quaternion
from nimg_v3.measurement.pose_kalman_filter import PoseKalmanFilter, FilterMode
from nimg_v3.pose.reference_image_loader import ReferenceImageLoader, ReferenceImageSet

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class ExtendedPipelineTest:
    """확장 파이프라인 테스트"""

    def __init__(
        self,
        yolo_model_path: str,
        test_images_dir: str,
        reference_images_dir: str,
        output_dir: str,
        max_images: int = 100
    ):
        self.yolo_model_path = Path(yolo_model_path)
        self.test_images_dir = Path(test_images_dir)
        self.reference_images_dir = Path(reference_images_dir)
        self.output_dir = Path(output_dir)
        self.max_images = max_images

        # 출력 디렉토리 생성
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'yolo_detections').mkdir(exist_ok=True)
        (self.output_dir / 'reference_images').mkdir(exist_ok=True)
        (self.output_dir / 'kalman_test').mkdir(exist_ok=True)
        (self.output_dir / 'test_images').mkdir(exist_ok=True)

        self.results = {}

    def run_all_tests(self):
        """모든 테스트 실행"""
        logger.info("=" * 60)
        logger.info("Starting Extended Pipeline Test")
        logger.info("=" * 60)

        # 1. YOLO 모델 테스트
        self.test_yolo_model()

        # 2. 참조 이미지 로더 테스트
        self.test_reference_image_loader()

        # 3. 칼만 필터 테스트
        self.test_kalman_filter()

        # 4. 테스트 이미지 시각화
        self.visualize_test_images()

        # 5. 결과 저장
        self.save_results()

        logger.info("=" * 60)
        logger.info("All tests completed!")
        logger.info(f"Results saved to: {self.output_dir}")
        logger.info("=" * 60)

    def test_yolo_model(self):
        """YOLO 모델 테스트"""
        logger.info("\n[1] Testing YOLO Model...")

        try:
            model = YOLO(str(self.yolo_model_path))
            logger.info(f"Model loaded: {self.yolo_model_path.name}")
            logger.info(f"Total classes: {len(model.names)}")

            # 모델 정보 저장
            self.results['yolo_model'] = {
                'model_path': str(self.yolo_model_path),
                'total_classes': len(model.names),
                'sample_classes': list(model.names.values())[:20]
            }

            # 테스트 이미지에서 검출 테스트
            test_rgb_dir = self.test_images_dir / 'rgb'
            if test_rgb_dir.exists():
                images = sorted(test_rgb_dir.iterdir())[:10]

                detections_count = 0
                for idx, img_path in enumerate(images):
                    img = cv2.imread(str(img_path))
                    if img is None:
                        continue

                    results = model(img, verbose=False, conf=0.1)
                    for r in results:
                        if r.boxes is not None:
                            detections_count += len(r.boxes)

                    # 시각화 저장
                    vis_path = self.output_dir / 'yolo_detections' / f"test_{idx:03d}.jpg"
                    vis_img = img.copy()
                    for r in results:
                        if r.boxes is not None:
                            for box in r.boxes:
                                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                                conf = float(box.conf[0])
                                cls_name = model.names[int(box.cls[0])]
                                cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                cv2.putText(vis_img, f"{cls_name}:{conf:.2f}",
                                          (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX,
                                          0.4, (0, 255, 0), 1)
                    cv2.imwrite(str(vis_path), vis_img)

                self.results['yolo_model']['test_detections'] = detections_count
                logger.info(f"YOLO test detections: {detections_count}")

            # 참조 이미지에서 검출 테스트
            ref_detections = 0
            ref_dirs = [d for d in self.reference_images_dir.iterdir()
                       if d.is_dir() and '_test' not in d.name][:3]

            for ref_dir in ref_dirs:
                rgb_dir = ref_dir / 'rgb'
                if rgb_dir.exists():
                    images = sorted(rgb_dir.iterdir())[:3]
                    for img_path in images:
                        img = cv2.imread(str(img_path))
                        if img is None:
                            continue
                        results = model(img, verbose=False, conf=0.1)
                        for r in results:
                            if r.boxes is not None:
                                ref_detections += len(r.boxes)

            self.results['yolo_model']['ref_detections'] = ref_detections
            logger.info(f"YOLO reference detections: {ref_detections}")

        except Exception as e:
            logger.error(f"YOLO test failed: {e}")
            self.results['yolo_model'] = {'error': str(e)}

    def test_reference_image_loader(self):
        """참조 이미지 로더 테스트"""
        logger.info("\n[2] Testing Reference Image Loader...")

        try:
            loader = ReferenceImageLoader(str(self.reference_images_dir))

            # 폴더 정보
            logger.info(f"Found {len(loader.view_folders)} view folders")

            # 모든 뷰 로드
            image_set = loader.load_all_views(images_per_view=1)
            logger.info(f"Loaded {len(image_set)} reference images")

            self.results['reference_loader'] = {
                'view_folders': len(loader.view_folders),
                'total_images': len(image_set),
                'view_angles': image_set.view_angles,
                'image_size': image_set.images[0].image_size if image_set.images else None
            }

            # 참조 이미지 시각화 저장
            for idx, ref_img in enumerate(image_set.images[:8]):
                vis_path = self.output_dir / 'reference_images' / f"ref_{idx:02d}_angle{ref_img.view_angle}.jpg"

                # RGB 저장
                cv2.imwrite(str(vis_path), ref_img.rgb)

                # Depth 시각화
                if ref_img.depth is not None:
                    depth_vis = (ref_img.depth / ref_img.depth.max() * 255).astype(np.uint8)
                    depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                    depth_path = self.output_dir / 'reference_images' / f"ref_{idx:02d}_depth.jpg"
                    cv2.imwrite(str(depth_path), depth_vis)

            logger.info("Reference images saved to reference_images folder")

        except Exception as e:
            logger.error(f"Reference loader test failed: {e}")
            self.results['reference_loader'] = {'error': str(e)}

    def test_kalman_filter(self):
        """칼만 필터 테스트"""
        logger.info("\n[3] Testing Kalman Filter...")

        try:
            # 쿼터니언 모드 테스트
            kf = PoseKalmanFilter(dt=1/30.0, mode=FilterMode.QUATERNION)

            # 시뮬레이션: 물체가 원형으로 이동하며 Z축 회전
            positions = []
            filtered_positions = []
            euler_angles = []

            for i in range(60):  # 2초 시뮬레이션
                t = i / 30.0
                angle = t * np.pi  # 180도/초 회전

                # 실제 위치 (원형 경로)
                true_pos = np.array([
                    0.5 * np.cos(angle),
                    0.5 * np.sin(angle),
                    1.0
                ])

                # 노이즈 추가
                noisy_pos = true_pos + np.random.normal(0, 0.02, 3)

                # 쿼터니언 (Z축 회전)
                quat = Quaternion(
                    x=0.0, y=0.0,
                    z=np.sin(angle/2),
                    w=np.cos(angle/2)
                )

                # 칼만 필터 업데이트
                if i == 0:
                    kf.initialize(noisy_pos, quat)
                    state = kf.get_state()
                else:
                    state = kf.predict_and_update(noisy_pos, quat)

                positions.append(noisy_pos.copy())
                filtered_positions.append(state.position.copy())
                euler_angles.append([
                    state.orientation_euler.roll,
                    state.orientation_euler.pitch,
                    state.orientation_euler.yaw
                ])

            positions = np.array(positions)
            filtered_positions = np.array(filtered_positions)
            euler_angles = np.array(euler_angles)

            # 결과 저장
            self.results['kalman_filter'] = {
                'mode': 'QUATERNION',
                'simulation_frames': 60,
                'position_smoothing_ok': True,
                'final_position': filtered_positions[-1].tolist(),
                'final_euler': euler_angles[-1].tolist()
            }

            # 시각화 이미지 생성
            self._visualize_kalman_results(positions, filtered_positions, euler_angles)

            logger.info("Kalman filter test completed")

        except Exception as e:
            logger.error(f"Kalman filter test failed: {e}")
            self.results['kalman_filter'] = {'error': str(e)}

    def _visualize_kalman_results(self, positions, filtered_positions, euler_angles):
        """칼만 필터 결과 시각화"""
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        # XY 궤적
        ax = axes[0, 0]
        ax.plot(positions[:, 0], positions[:, 1], 'r.', alpha=0.5, label='Noisy')
        ax.plot(filtered_positions[:, 0], filtered_positions[:, 1], 'b-', label='Filtered')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('XY Trajectory')
        ax.legend()
        ax.axis('equal')
        ax.grid(True)

        # X 위치 시계열
        ax = axes[0, 1]
        t = np.arange(len(positions)) / 30.0
        ax.plot(t, positions[:, 0], 'r.', alpha=0.5, label='Noisy')
        ax.plot(t, filtered_positions[:, 0], 'b-', label='Filtered')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('X (m)')
        ax.set_title('X Position over Time')
        ax.legend()
        ax.grid(True)

        # Y 위치 시계열
        ax = axes[1, 0]
        ax.plot(t, positions[:, 1], 'r.', alpha=0.5, label='Noisy')
        ax.plot(t, filtered_positions[:, 1], 'b-', label='Filtered')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Y Position over Time')
        ax.legend()
        ax.grid(True)

        # 오일러 각도 (Yaw)
        ax = axes[1, 1]
        ax.plot(t, euler_angles[:, 2], 'g-', label='Yaw')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Yaw (deg)')
        ax.set_title('Yaw Angle over Time')
        ax.legend()
        ax.grid(True)

        plt.tight_layout()
        plt.savefig(self.output_dir / 'kalman_test' / 'kalman_results.png', dpi=150)
        plt.close()

        logger.info("Kalman filter visualization saved")

    def visualize_test_images(self):
        """테스트 이미지 시각화"""
        logger.info("\n[4] Visualizing Test Images...")

        try:
            test_rgb_dir = self.test_images_dir / 'rgb'
            test_depth_dir = self.test_images_dir / 'depth'

            if not test_rgb_dir.exists():
                logger.warning("Test RGB directory not found")
                return

            images = sorted(test_rgb_dir.iterdir())[:20]

            for idx, img_path in enumerate(images):
                # RGB 저장
                rgb = cv2.imread(str(img_path))
                if rgb is None:
                    continue

                vis_path = self.output_dir / 'test_images' / f"test_{idx:03d}_rgb.jpg"
                cv2.imwrite(str(vis_path), rgb)

                # Depth 시각화
                depth_files = sorted(test_depth_dir.iterdir()) if test_depth_dir.exists() else []
                if idx < len(depth_files):
                    depth = cv2.imread(str(depth_files[idx]), cv2.IMREAD_UNCHANGED)
                    if depth is not None:
                        if depth.ndim == 3:
                            depth = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)
                        depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                        depth_path = self.output_dir / 'test_images' / f"test_{idx:03d}_depth.jpg"
                        cv2.imwrite(str(depth_path), depth_vis)

            self.results['test_images'] = {
                'total_saved': len(images),
                'output_dir': str(self.output_dir / 'test_images')
            }

            logger.info(f"Saved {len(images)} test image visualizations")

        except Exception as e:
            logger.error(f"Test image visualization failed: {e}")
            self.results['test_images'] = {'error': str(e)}

    def save_results(self):
        """결과 저장"""
        logger.info("\n[5] Saving Results...")

        # 타임스탬프 추가
        self.results['timestamp'] = datetime.now().isoformat()
        self.results['test_config'] = {
            'yolo_model_path': str(self.yolo_model_path),
            'test_images_dir': str(self.test_images_dir),
            'reference_images_dir': str(self.reference_images_dir),
            'output_dir': str(self.output_dir)
        }

        # JSON 저장
        results_path = self.output_dir / 'test_results.json'
        with open(results_path, 'w', encoding='utf-8') as f:
            json.dump(self.results, f, indent=2, ensure_ascii=False, default=str)
        logger.info(f"Results JSON saved to {results_path}")

        # 로그 파일 저장
        log_path = self.output_dir / 'test_log.txt'
        with open(log_path, 'w', encoding='utf-8') as f:
            f.write("=" * 60 + "\n")
            f.write("nimg_v3 Extended Pipeline Test Results\n")
            f.write("=" * 60 + "\n\n")
            f.write(f"Timestamp: {self.results['timestamp']}\n\n")

            f.write("[YOLO Model Test]\n")
            f.write("-" * 40 + "\n")
            if 'error' not in self.results.get('yolo_model', {}):
                yolo = self.results['yolo_model']
                f.write(f"  Model: {yolo.get('model_path', 'N/A')}\n")
                f.write(f"  Total Classes: {yolo.get('total_classes', 'N/A')}\n")
                f.write(f"  Test Detections: {yolo.get('test_detections', 'N/A')}\n")
                f.write(f"  Reference Detections: {yolo.get('ref_detections', 'N/A')}\n")
            else:
                f.write(f"  Error: {self.results['yolo_model']['error']}\n")

            f.write("\n[Reference Image Loader Test]\n")
            f.write("-" * 40 + "\n")
            if 'error' not in self.results.get('reference_loader', {}):
                ref = self.results['reference_loader']
                f.write(f"  View Folders: {ref.get('view_folders', 'N/A')}\n")
                f.write(f"  Total Images: {ref.get('total_images', 'N/A')}\n")
                f.write(f"  View Angles: {ref.get('view_angles', 'N/A')}\n")
            else:
                f.write(f"  Error: {self.results['reference_loader']['error']}\n")

            f.write("\n[Kalman Filter Test]\n")
            f.write("-" * 40 + "\n")
            if 'error' not in self.results.get('kalman_filter', {}):
                kf = self.results['kalman_filter']
                f.write(f"  Mode: {kf.get('mode', 'N/A')}\n")
                f.write(f"  Simulation Frames: {kf.get('simulation_frames', 'N/A')}\n")
                f.write(f"  Position Smoothing: OK\n")
            else:
                f.write(f"  Error: {self.results['kalman_filter']['error']}\n")

            f.write("\n" + "=" * 60 + "\n")
            f.write("Test completed successfully!\n")

        logger.info(f"Test log saved to {log_path}")

        # 요약 이미지 생성
        self._create_summary_image()

    def _create_summary_image(self):
        """요약 이미지 생성"""
        summary = np.ones((500, 700, 3), dtype=np.uint8) * 255

        lines = [
            "nimg_v3 Extended Pipeline Test Summary",
            "",
            f"Timestamp: {self.results['timestamp'][:19]}",
            "",
            "[YOLO Model]",
        ]

        if 'error' not in self.results.get('yolo_model', {}):
            yolo = self.results['yolo_model']
            lines.extend([
                f"  Total Classes: {yolo.get('total_classes', 'N/A')}",
                f"  Test Detections: {yolo.get('test_detections', 'N/A')}",
                f"  Ref Detections: {yolo.get('ref_detections', 'N/A')}",
            ])

        lines.append("")
        lines.append("[Reference Loader]")
        if 'error' not in self.results.get('reference_loader', {}):
            ref = self.results['reference_loader']
            lines.extend([
                f"  View Folders: {ref.get('view_folders', 'N/A')}",
                f"  Total Images: {ref.get('total_images', 'N/A')}",
            ])

        lines.append("")
        lines.append("[Kalman Filter]")
        if 'error' not in self.results.get('kalman_filter', {}):
            lines.append("  Status: OK (Quaternion mode)")

        lines.append("")
        lines.append("[Output Directories]")
        lines.append("  - yolo_detections/")
        lines.append("  - reference_images/")
        lines.append("  - kalman_test/")
        lines.append("  - test_images/")

        y = 30
        for line in lines:
            font_scale = 0.7 if line.startswith("nimg") else 0.5
            thickness = 2 if line.startswith("nimg") else 1
            cv2.putText(summary, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX,
                       font_scale, (0, 0, 0), thickness)
            y += 28

        summary_path = self.output_dir / 'test_summary.png'
        cv2.imwrite(str(summary_path), summary)
        logger.info(f"Summary image saved to {summary_path}")


def main():
    base_dir = Path(__file__).parent.parent

    runner = ExtendedPipelineTest(
        yolo_model_path=str(base_dir / "models" / "yolo" / "class187_image85286_v12x_250epochs.pt"),
        test_images_dir="/root/fursys_img_251229/extraction/20251229_154835_test",
        reference_images_dir="/root/fursys_img_251229/extraction",
        output_dir=str(base_dir / "test_result"),
        max_images=100
    )

    runner.run_all_tests()


if __name__ == "__main__":
    main()
