#!/usr/bin/env python3
"""
속도/각도 측정 테스트 (YOLO 없이)

참조 이미지 시퀀스에서 Depth 기반 중심점 추출 + 칼만 필터로 속도/각도 측정
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
import re

sys.path.insert(0, str(Path(__file__).parent.parent))

from nimg_v3.measurement.pose_converter import PoseConverter, EulerAngles, Quaternion
from nimg_v3.measurement.pose_kalman_filter import PoseKalmanFilter, FilterMode, PoseKalmanState

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class DepthBasedObjectTracker:
    """Depth 기반 객체 추적기 (YOLO 없이)"""

    def __init__(
        self,
        depth_threshold_min: float = 0.5,  # 50cm 이상
        depth_threshold_max: float = 3.5,   # 3.5m 이하 (실제 데이터 범위에 맞춤)
        min_object_area: int = 500,
        camera_intrinsics: Optional[Dict] = None
    ):
        self.depth_threshold_min = depth_threshold_min
        self.depth_threshold_max = depth_threshold_max
        self.min_object_area = min_object_area

        # D455 기본 내부 파라미터
        self.intrinsics = camera_intrinsics or {
            'fx': 383.883, 'fy': 383.883,
            'cx': 320.499, 'cy': 237.913
        }

    def segment_object(self, depth: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Depth 이미지에서 객체 세그멘테이션

        Returns:
            mask: 객체 마스크
            center_3d: [x, y, z] 3D 중심점 (카메라 좌표계, 미터)
        """
        # Depth 범위 마스킹
        valid_mask = (depth > self.depth_threshold_min) & (depth < self.depth_threshold_max)

        # 노이즈 제거
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        valid_mask = cv2.morphologyEx(valid_mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        valid_mask = cv2.morphologyEx(valid_mask, cv2.MORPH_CLOSE, kernel)

        # 연결 영역 분석
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(valid_mask)

        if num_labels < 2:  # 배경만 있음
            return None, None

        # 가장 큰 객체 찾기 (배경 제외)
        areas = stats[1:, cv2.CC_STAT_AREA]
        if len(areas) == 0 or max(areas) < self.min_object_area:
            return None, None

        largest_idx = np.argmax(areas) + 1
        object_mask = (labels == largest_idx).astype(np.uint8) * 255

        # 2D 중심점
        centroid_2d = centroids[largest_idx]  # [x, y]
        cx, cy = int(centroid_2d[0]), int(centroid_2d[1])

        # 중심점 주변 depth 평균
        roi_size = 10
        y1 = max(0, cy - roi_size)
        y2 = min(depth.shape[0], cy + roi_size)
        x1 = max(0, cx - roi_size)
        x2 = min(depth.shape[1], cx + roi_size)

        depth_roi = depth[y1:y2, x1:x2]
        mask_roi = object_mask[y1:y2, x1:x2]

        valid_depths = depth_roi[mask_roi > 0]
        if len(valid_depths) == 0:
            return object_mask, None

        z = np.median(valid_depths)

        # 2D → 3D 변환 (pinhole camera model)
        x_3d = (cx - self.intrinsics['cx']) * z / self.intrinsics['fx']
        y_3d = (cy - self.intrinsics['cy']) * z / self.intrinsics['fy']

        center_3d = np.array([x_3d, y_3d, z])

        return object_mask, center_3d

    def estimate_orientation(self, mask: np.ndarray) -> float:
        """
        마스크에서 객체 방향 추정 (PCA 기반)

        Returns:
            angle: 도 단위 회전 각도 (Yaw)
        """
        # 마스크 포인트 추출
        points = np.column_stack(np.where(mask > 0))
        if len(points) < 10:
            return 0.0

        # PCA로 주축 방향 추출
        mean = np.mean(points, axis=0)
        centered = points - mean

        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        # 주축 방향 (가장 큰 고유값에 대응)
        principal_axis = eigenvectors[:, np.argmax(eigenvalues)]

        # 각도 계산 (라디안 → 도)
        angle = np.degrees(np.arctan2(principal_axis[1], principal_axis[0]))

        return angle


class VelocityAngleMeasurementTest:
    """속도/각도 측정 테스트"""

    def __init__(
        self,
        reference_dir: str,
        output_dir: str,
        max_frames: int = 200
    ):
        self.reference_dir = Path(reference_dir)
        self.output_dir = Path(output_dir)
        self.max_frames = max_frames

        # 출력 디렉토리 설정
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'tracking_vis').mkdir(exist_ok=True)
        (self.output_dir / 'plots').mkdir(exist_ok=True)

        # 컴포넌트 초기화
        self.tracker = DepthBasedObjectTracker()
        self.kalman_filter = PoseKalmanFilter(dt=1/30.0, mode=FilterMode.QUATERNION)
        self.pose_converter = PoseConverter()

        self.results = {
            'test_info': {},
            'measurements': [],
            'statistics': {}
        }

    def _extract_timestamp(self, filename: str) -> float:
        """파일명에서 타임스탬프 추출"""
        match = re.search(r'(\d+\.\d+)', filename)
        if match:
            return float(match.group(1))
        return 0.0

    def _load_frame_pair(self, rgb_path: Path, depth_csv_files: List[Path], idx: int) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """RGB와 대응 Depth CSV 로드"""
        rgb = cv2.imread(str(rgb_path))
        if rgb is None:
            return None, None

        depth = None
        if idx < len(depth_csv_files):
            try:
                # CSV에서 depth 로드 (이미 미터 단위)
                depth = np.loadtxt(str(depth_csv_files[idx]), delimiter=',').astype(np.float32)
            except Exception as e:
                logger.warning(f"Failed to load depth CSV: {e}")

        return rgb, depth

    def run_test_on_sequence(self, sequence_name: str, rgb_dir: Path) -> Dict:
        """단일 시퀀스에서 측정 테스트"""
        logger.info(f"Processing sequence: {sequence_name}")

        rgb_files = sorted([f for f in rgb_dir.iterdir() if f.suffix == '.png'])
        if not rgb_files:
            return {'error': 'No RGB files found'}

        # Depth CSV 파일 목록 (PNG 대신 CSV 사용)
        depth_csv_dir = rgb_dir.parent / 'depth_csv'
        depth_csv_files = sorted([f for f in depth_csv_dir.iterdir() if f.suffix == '.csv']) if depth_csv_dir.exists() else []

        rgb_files = rgb_files[:self.max_frames]

        measurements = []
        positions_raw = []
        positions_filtered = []
        velocities = []
        angles = []
        timestamps = []

        filter_initialized = False
        prev_timestamp = None

        for idx, rgb_path in enumerate(rgb_files):
            rgb, depth = self._load_frame_pair(rgb_path, depth_csv_files, idx)
            if rgb is None or depth is None:
                continue

            timestamp = self._extract_timestamp(rgb_path.name)
            timestamps.append(timestamp)

            # Depth 기반 객체 추적
            mask, center_3d = self.tracker.segment_object(depth)

            if mask is None or center_3d is None:
                continue

            # 방향 추정
            yaw = self.tracker.estimate_orientation(mask)

            positions_raw.append(center_3d.copy())

            # 시간 간격 계산
            if prev_timestamp is not None:
                dt = (timestamp - prev_timestamp) / 1000.0  # ms to s
                if dt > 0:
                    self.kalman_filter.set_dt(dt)
            prev_timestamp = timestamp

            # 쿼터니언 생성 (Yaw만)
            yaw_rad = np.radians(yaw)
            quat = Quaternion(
                x=0.0, y=0.0,
                z=np.sin(yaw_rad / 2),
                w=np.cos(yaw_rad / 2)
            )

            # 칼만 필터 업데이트
            if not filter_initialized:
                self.kalman_filter.initialize(center_3d, quat)
                state = self.kalman_filter.get_state()
                filter_initialized = True
            else:
                state = self.kalman_filter.predict_and_update(center_3d, quat)

            positions_filtered.append(state.position.copy())
            velocities.append(state.velocity.copy())
            angles.append({
                'roll': state.orientation_euler.roll,
                'pitch': state.orientation_euler.pitch,
                'yaw': state.orientation_euler.yaw
            })

            measurement = {
                'frame': idx,
                'timestamp': timestamp,
                'position_raw': center_3d.tolist(),
                'position_filtered': state.position.tolist(),
                'velocity': state.velocity.tolist(),
                'speed': state.speed,
                'euler': angles[-1],
                'yaw_raw': yaw
            }
            measurements.append(measurement)

            # 시각화 저장 (처음 30개만)
            if idx < 30:
                self._save_tracking_visualization(
                    rgb, depth, mask, center_3d, state,
                    sequence_name, idx
                )

            if (idx + 1) % 50 == 0:
                logger.info(f"  Processed {idx + 1}/{len(rgb_files)} frames")

        # 통계 계산
        if measurements:
            speeds = [m['speed'] for m in measurements]
            yaws = [m['euler']['yaw'] for m in measurements]

            stats = {
                'total_frames': len(measurements),
                'avg_speed': float(np.mean(speeds)),
                'max_speed': float(np.max(speeds)),
                'min_speed': float(np.min(speeds)),
                'std_speed': float(np.std(speeds)),
                'avg_yaw': float(np.mean(yaws)),
                'yaw_range': float(np.max(yaws) - np.min(yaws)),
            }

            # 플롯 생성
            self._create_measurement_plots(
                positions_raw, positions_filtered, velocities,
                [m['euler']['yaw'] for m in measurements],
                timestamps[:len(measurements)],
                sequence_name
            )
        else:
            stats = {'error': 'No valid measurements'}

        return {
            'sequence': sequence_name,
            'measurements': measurements,
            'statistics': stats
        }

    def _save_tracking_visualization(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        mask: np.ndarray,
        center_3d: np.ndarray,
        state: PoseKalmanState,
        sequence_name: str,
        frame_idx: int
    ):
        """추적 시각화 저장"""
        vis = rgb.copy()

        # 마스크 오버레이
        mask_color = np.zeros_like(rgb)
        mask_color[:, :, 1] = mask  # 녹색
        vis = cv2.addWeighted(vis, 0.7, mask_color, 0.3, 0)

        # 중심점 표시
        cx = int(center_3d[0] * self.tracker.intrinsics['fx'] / center_3d[2] + self.tracker.intrinsics['cx'])
        cy = int(center_3d[1] * self.tracker.intrinsics['fy'] / center_3d[2] + self.tracker.intrinsics['cy'])
        cv2.circle(vis, (cx, cy), 10, (0, 0, 255), -1)

        # 정보 텍스트
        info_lines = [
            f"Frame: {frame_idx}",
            f"Pos: ({state.position[0]:.3f}, {state.position[1]:.3f}, {state.position[2]:.3f})m",
            f"Speed: {state.speed:.3f} m/s",
            f"Yaw: {state.orientation_euler.yaw:.1f} deg",
        ]

        y = 30
        for line in info_lines:
            cv2.putText(vis, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(vis, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
            y += 25

        # 저장
        vis_path = self.output_dir / 'tracking_vis' / f"{sequence_name}_frame{frame_idx:04d}.jpg"
        cv2.imwrite(str(vis_path), vis)

    def _create_measurement_plots(
        self,
        positions_raw: List[np.ndarray],
        positions_filtered: List[np.ndarray],
        velocities: List[np.ndarray],
        yaws: List[float],
        timestamps: List[float],
        sequence_name: str
    ):
        """측정 결과 플롯 생성"""
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        positions_raw = np.array(positions_raw)
        positions_filtered = np.array(positions_filtered)
        velocities = np.array(velocities)
        yaws = np.array(yaws)

        # 타임스탬프를 상대 시간으로 변환
        t = (np.array(timestamps) - timestamps[0]) / 1000.0  # 초 단위

        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        fig.suptitle(f'Velocity/Angle Measurement: {sequence_name}', fontsize=14)

        # X 위치
        ax = axes[0, 0]
        ax.plot(t, positions_raw[:, 0], 'r.', alpha=0.3, label='Raw', markersize=2)
        ax.plot(t, positions_filtered[:, 0], 'b-', label='Filtered', linewidth=1)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('X (m)')
        ax.set_title('X Position')
        ax.legend()
        ax.grid(True)

        # Y 위치
        ax = axes[0, 1]
        ax.plot(t, positions_raw[:, 1], 'r.', alpha=0.3, label='Raw', markersize=2)
        ax.plot(t, positions_filtered[:, 1], 'b-', label='Filtered', linewidth=1)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Y Position')
        ax.legend()
        ax.grid(True)

        # Z 위치 (Depth)
        ax = axes[0, 2]
        ax.plot(t, positions_raw[:, 2], 'r.', alpha=0.3, label='Raw', markersize=2)
        ax.plot(t, positions_filtered[:, 2], 'b-', label='Filtered', linewidth=1)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Z (m)')
        ax.set_title('Z Position (Depth)')
        ax.legend()
        ax.grid(True)

        # 속도
        ax = axes[1, 0]
        speeds = np.linalg.norm(velocities, axis=1)
        ax.plot(t, speeds, 'g-', linewidth=1)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (m/s)')
        ax.set_title(f'Speed (avg: {np.mean(speeds):.3f} m/s)')
        ax.grid(True)

        # Yaw 각도
        ax = axes[1, 1]
        ax.plot(t, yaws, 'm-', linewidth=1)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Yaw (deg)')
        ax.set_title('Yaw Angle')
        ax.grid(True)

        # XY 궤적
        ax = axes[1, 2]
        ax.plot(positions_raw[:, 0], positions_raw[:, 1], 'r.', alpha=0.3, markersize=2, label='Raw')
        ax.plot(positions_filtered[:, 0], positions_filtered[:, 1], 'b-', linewidth=1, label='Filtered')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('XY Trajectory')
        ax.legend()
        ax.axis('equal')
        ax.grid(True)

        plt.tight_layout()
        plt.savefig(self.output_dir / 'plots' / f'{sequence_name}_measurements.png', dpi=150)
        plt.close()

    def run_all_tests(self):
        """모든 시퀀스에서 테스트 실행"""
        logger.info("=" * 60)
        logger.info("Starting Velocity/Angle Measurement Test")
        logger.info("(YOLO-free, Depth-based tracking)")
        logger.info("=" * 60)

        start_time = time.time()

        self.results['test_info'] = {
            'timestamp': datetime.now().isoformat(),
            'reference_dir': str(self.reference_dir),
            'output_dir': str(self.output_dir),
            'method': 'Depth-based segmentation + Kalman Filter'
        }

        # 참조 이미지 폴더 탐색
        sequence_dirs = sorted([
            d for d in self.reference_dir.iterdir()
            if d.is_dir() and '_test' not in d.name and (d / 'rgb').exists()
        ])

        logger.info(f"Found {len(sequence_dirs)} sequences")

        all_measurements = []
        sequence_stats = []

        for seq_dir in sequence_dirs[:5]:  # 처음 5개 시퀀스만
            rgb_dir = seq_dir / 'rgb'
            result = self.run_test_on_sequence(seq_dir.name, rgb_dir)

            all_measurements.append(result)
            if 'statistics' in result and 'error' not in result['statistics']:
                sequence_stats.append({
                    'sequence': result['sequence'],
                    **result['statistics']
                })

            # 칼만 필터 리셋
            self.kalman_filter.reset()

        self.results['sequences'] = all_measurements
        self.results['sequence_statistics'] = sequence_stats

        # 전체 통계
        if sequence_stats:
            self.results['overall_statistics'] = {
                'total_sequences': len(sequence_stats),
                'total_frames': sum(s['total_frames'] for s in sequence_stats),
                'avg_speed_all': float(np.mean([s['avg_speed'] for s in sequence_stats])),
                'max_speed_all': float(max(s['max_speed'] for s in sequence_stats)),
            }

        total_time = time.time() - start_time
        self.results['test_info']['total_time'] = total_time

        # 결과 저장
        self._save_results()

        logger.info("=" * 60)
        logger.info(f"Test completed in {total_time:.2f}s")
        logger.info(f"Results saved to: {self.output_dir}")
        logger.info("=" * 60)

    def _save_results(self):
        """결과 저장"""
        # JSON
        results_path = self.output_dir / 'measurement_results.json'
        with open(results_path, 'w', encoding='utf-8') as f:
            # measurements 간소화 (너무 많은 데이터 제거)
            simplified_results = {
                'test_info': self.results['test_info'],
                'sequence_statistics': self.results.get('sequence_statistics', []),
                'overall_statistics': self.results.get('overall_statistics', {}),
            }
            json.dump(simplified_results, f, indent=2, ensure_ascii=False)

        # 로그 파일
        log_path = self.output_dir / 'measurement_log.txt'
        with open(log_path, 'w', encoding='utf-8') as f:
            f.write("=" * 60 + "\n")
            f.write("Velocity/Angle Measurement Test Results\n")
            f.write("(Depth-based tracking, YOLO-free)\n")
            f.write("=" * 60 + "\n\n")

            f.write(f"Timestamp: {self.results['test_info']['timestamp']}\n")
            f.write(f"Method: {self.results['test_info']['method']}\n\n")

            f.write("[Sequence Results]\n")
            f.write("-" * 40 + "\n")
            for stats in self.results.get('sequence_statistics', []):
                f.write(f"\n{stats['sequence']}:\n")
                f.write(f"  Frames: {stats['total_frames']}\n")
                f.write(f"  Avg Speed: {stats['avg_speed']:.4f} m/s\n")
                f.write(f"  Max Speed: {stats['max_speed']:.4f} m/s\n")
                f.write(f"  Yaw Range: {stats['yaw_range']:.2f} deg\n")

            f.write("\n\n[Overall Statistics]\n")
            f.write("-" * 40 + "\n")
            overall = self.results.get('overall_statistics', {})
            f.write(f"Total Sequences: {overall.get('total_sequences', 0)}\n")
            f.write(f"Total Frames: {overall.get('total_frames', 0)}\n")
            f.write(f"Avg Speed (all): {overall.get('avg_speed_all', 0):.4f} m/s\n")
            f.write(f"Max Speed (all): {overall.get('max_speed_all', 0):.4f} m/s\n")

        # 요약 이미지
        self._create_summary_image()

        logger.info(f"Results saved to {self.output_dir}")

    def _create_summary_image(self):
        """요약 이미지 생성"""
        summary = np.ones((550, 750, 3), dtype=np.uint8) * 255

        lines = [
            "Velocity/Angle Measurement Test Summary",
            "(Depth-based tracking, YOLO-free)",
            "",
            f"Timestamp: {self.results['test_info']['timestamp'][:19]}",
            "",
            "[Method]",
            "  - Depth-based object segmentation",
            "  - PCA-based orientation estimation",
            "  - Quaternion Kalman Filter",
            "",
            "[Sequence Results]",
        ]

        for stats in self.results.get('sequence_statistics', [])[:5]:
            lines.append(f"  {stats['sequence'][:25]}: {stats['total_frames']} frames, "
                        f"speed={stats['avg_speed']:.3f} m/s")

        lines.append("")
        lines.append("[Overall]")
        overall = self.results.get('overall_statistics', {})
        lines.append(f"  Total Sequences: {overall.get('total_sequences', 0)}")
        lines.append(f"  Total Frames: {overall.get('total_frames', 0)}")
        lines.append(f"  Avg Speed: {overall.get('avg_speed_all', 0):.4f} m/s")
        lines.append(f"  Max Speed: {overall.get('max_speed_all', 0):.4f} m/s")

        y = 30
        for line in lines:
            font_scale = 0.7 if line.startswith("Velocity") else 0.5
            thickness = 2 if line.startswith("Velocity") else 1
            cv2.putText(summary, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX,
                       font_scale, (0, 0, 0), thickness)
            y += 26

        summary_path = self.output_dir / 'measurement_summary.png'
        cv2.imwrite(str(summary_path), summary)


def main():
    base_dir = Path(__file__).parent.parent

    test = VelocityAngleMeasurementTest(
        reference_dir="/root/fursys_img_251229/extraction",
        output_dir=str(base_dir / "test_result" / "velocity_angle_measurement"),
        max_frames=200
    )

    test.run_all_tests()


if __name__ == "__main__":
    main()
