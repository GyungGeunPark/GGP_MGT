#!/usr/bin/env python3
"""
기준 이미지 대비 절대/상대 각도 및 속도 측정 테스트

테스트 데이터: /root/fursys_img_251229/extraction/20251229_154835_test
- 첫 번째 프레임을 기준(Reference)으로 설정
- 각 프레임에서 기준 대비 절대 위치/각도 계산
- 연속 프레임 간 상대 속도/각도 변화량 계산
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


class DepthBasedTracker:
    """Depth 기반 객체 추적"""

    def __init__(
        self,
        depth_min: float = 0.5,
        depth_max: float = 4.0,
        min_area: int = 500,
        intrinsics: Optional[Dict] = None
    ):
        self.depth_min = depth_min
        self.depth_max = depth_max
        self.min_area = min_area
        self.intrinsics = intrinsics or {
            'fx': 383.883, 'fy': 383.883,
            'cx': 320.499, 'cy': 237.913
        }

    def track(self, depth: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[float]]:
        """
        Depth에서 객체 추적

        Returns:
            mask: 객체 마스크
            center_3d: [x, y, z] 3D 중심점
            yaw: 추정 yaw 각도 (도)
        """
        # 유효 depth 마스크
        valid = (depth > self.depth_min) & (depth < self.depth_max)

        # 모폴로지 처리
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        valid = cv2.morphologyEx(valid.astype(np.uint8), cv2.MORPH_OPEN, kernel)
        valid = cv2.morphologyEx(valid, cv2.MORPH_CLOSE, kernel)

        # 연결 영역 분석
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(valid)

        if num_labels < 2:
            return None, None, None

        # 가장 큰 객체 (배경 제외)
        areas = stats[1:, cv2.CC_STAT_AREA]
        if len(areas) == 0 or max(areas) < self.min_area:
            return None, None, None

        largest_idx = np.argmax(areas) + 1
        mask = (labels == largest_idx).astype(np.uint8) * 255

        # 2D 중심
        cx, cy = centroids[largest_idx]
        cx, cy = int(cx), int(cy)

        # 중심 주변 depth
        roi = 15
        y1, y2 = max(0, cy-roi), min(depth.shape[0], cy+roi)
        x1, x2 = max(0, cx-roi), min(depth.shape[1], cx+roi)

        roi_depth = depth[y1:y2, x1:x2]
        roi_mask = mask[y1:y2, x1:x2]
        valid_depths = roi_depth[roi_mask > 0]

        if len(valid_depths) == 0:
            return mask, None, None

        z = np.median(valid_depths)

        # 3D 변환
        x_3d = (cx - self.intrinsics['cx']) * z / self.intrinsics['fx']
        y_3d = (cy - self.intrinsics['cy']) * z / self.intrinsics['fy']
        center_3d = np.array([x_3d, y_3d, z])

        # PCA로 Yaw 추정
        yaw = self._estimate_yaw(mask)

        return mask, center_3d, yaw

    def _estimate_yaw(self, mask: np.ndarray) -> float:
        """PCA로 Yaw 각도 추정"""
        points = np.column_stack(np.where(mask > 0))
        if len(points) < 10:
            return 0.0

        mean = np.mean(points, axis=0)
        centered = points - mean
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        principal = eigenvectors[:, np.argmax(eigenvalues)]

        return np.degrees(np.arctan2(principal[1], principal[0]))


class AbsoluteRelativeMeasurementTest:
    """기준 이미지 대비 절대/상대 측정 테스트"""

    def __init__(
        self,
        test_data_dir: str,
        output_dir: str,
        max_frames: int = 500,
        sample_interval: int = 1
    ):
        self.test_data_dir = Path(test_data_dir)
        self.output_dir = Path(output_dir)
        self.max_frames = max_frames
        self.sample_interval = sample_interval

        # 출력 디렉토리
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'visualizations').mkdir(exist_ok=True)
        (self.output_dir / 'plots').mkdir(exist_ok=True)

        # 컴포넌트
        self.tracker = DepthBasedTracker()
        self.kalman_filter = PoseKalmanFilter(dt=1/30.0, mode=FilterMode.QUATERNION)
        self.pose_converter = PoseConverter()

        # 기준 상태
        self.reference_position = None
        self.reference_yaw = None
        self.reference_timestamp = None

        self.results = {
            'test_info': {},
            'reference_frame': {},
            'measurements': [],
            'statistics': {}
        }

    def _extract_timestamp(self, filename: str) -> float:
        match = re.search(r'(\d+\.\d+)', filename)
        return float(match.group(1)) if match else 0.0

    def _load_depth_csv(self, csv_path: Path) -> Optional[np.ndarray]:
        try:
            return np.loadtxt(str(csv_path), delimiter=',').astype(np.float32)
        except:
            return None

    def run_test(self):
        """테스트 실행"""
        logger.info("=" * 60)
        logger.info("Absolute/Relative Measurement Test")
        logger.info("Test Data: 20251229_154835_test")
        logger.info("=" * 60)

        start_time = time.time()

        # 파일 목록
        rgb_dir = self.test_data_dir / 'rgb'
        depth_csv_dir = self.test_data_dir / 'depth_csv'

        rgb_files = sorted([f for f in rgb_dir.iterdir() if f.suffix == '.png'])
        depth_csv_files = sorted([f for f in depth_csv_dir.iterdir() if f.suffix == '.csv'])

        logger.info(f"Total RGB files: {len(rgb_files)}")
        logger.info(f"Total Depth CSV files: {len(depth_csv_files)}")

        # 샘플링
        rgb_files = rgb_files[::self.sample_interval][:self.max_frames]
        depth_csv_files = depth_csv_files[::self.sample_interval][:self.max_frames]

        logger.info(f"Processing {len(rgb_files)} frames (interval: {self.sample_interval})")

        # 결과 저장용
        measurements = []
        positions_raw = []
        positions_filtered = []
        velocities = []
        yaws_raw = []
        yaws_filtered = []
        timestamps = []

        # 절대 변화량
        absolute_positions = []  # 기준 대비 절대 위치
        absolute_yaws = []       # 기준 대비 절대 각도

        # 상대 변화량
        relative_positions = []  # 이전 프레임 대비
        relative_yaws = []

        filter_initialized = False
        prev_timestamp = None
        prev_position = None
        prev_yaw = None

        for idx in range(min(len(rgb_files), len(depth_csv_files))):
            rgb_path = rgb_files[idx]
            depth_csv_path = depth_csv_files[idx]

            # 로드
            rgb = cv2.imread(str(rgb_path))
            depth = self._load_depth_csv(depth_csv_path)

            if rgb is None or depth is None:
                continue

            timestamp = self._extract_timestamp(rgb_path.name)

            # 추적
            mask, center_3d, yaw_raw = self.tracker.track(depth)

            if center_3d is None:
                continue

            timestamps.append(timestamp)
            positions_raw.append(center_3d.copy())
            yaws_raw.append(yaw_raw)

            # 시간 간격
            if prev_timestamp is not None:
                dt = (timestamp - prev_timestamp) / 1000.0
                if dt > 0 and dt < 1.0:  # 합리적인 범위
                    self.kalman_filter.set_dt(dt)
            prev_timestamp = timestamp

            # 쿼터니언 생성
            yaw_rad = np.radians(yaw_raw)
            quat = Quaternion(x=0.0, y=0.0, z=np.sin(yaw_rad/2), w=np.cos(yaw_rad/2))

            # 칼만 필터
            if not filter_initialized:
                self.kalman_filter.initialize(center_3d, quat)
                state = self.kalman_filter.get_state()
                filter_initialized = True

                # 기준 프레임 설정
                self.reference_position = center_3d.copy()
                self.reference_yaw = yaw_raw
                self.reference_timestamp = timestamp

                self.results['reference_frame'] = {
                    'timestamp': timestamp,
                    'position': center_3d.tolist(),
                    'yaw': yaw_raw,
                    'frame_index': idx
                }
                logger.info(f"Reference frame set: pos={center_3d}, yaw={yaw_raw:.2f}°")
            else:
                state = self.kalman_filter.predict_and_update(center_3d, quat)

            positions_filtered.append(state.position.copy())
            velocities.append(state.velocity.copy())
            yaws_filtered.append(state.orientation_euler.yaw)

            # 기준 대비 절대 변화량
            abs_pos = state.position - self.reference_position
            abs_yaw = state.orientation_euler.yaw - self.reference_yaw
            # -180 ~ 180 정규화
            while abs_yaw > 180: abs_yaw -= 360
            while abs_yaw < -180: abs_yaw += 360

            absolute_positions.append(abs_pos.copy())
            absolute_yaws.append(abs_yaw)

            # 이전 프레임 대비 상대 변화량
            if prev_position is not None:
                rel_pos = state.position - prev_position
                rel_yaw = state.orientation_euler.yaw - prev_yaw
                while rel_yaw > 180: rel_yaw -= 360
                while rel_yaw < -180: rel_yaw += 360
            else:
                rel_pos = np.zeros(3)
                rel_yaw = 0.0

            relative_positions.append(rel_pos.copy() if isinstance(rel_pos, np.ndarray) else np.array(rel_pos))
            relative_yaws.append(rel_yaw)

            prev_position = state.position.copy()
            prev_yaw = state.orientation_euler.yaw

            # 측정 기록
            measurement = {
                'frame': idx,
                'timestamp': timestamp,
                'time_from_ref': (timestamp - self.reference_timestamp) / 1000.0,  # 초
                'position_raw': center_3d.tolist(),
                'position_filtered': state.position.tolist(),
                'velocity': state.velocity.tolist(),
                'speed': state.speed,
                'yaw_raw': yaw_raw,
                'yaw_filtered': state.orientation_euler.yaw,
                'absolute': {
                    'position': abs_pos.tolist(),
                    'distance': float(np.linalg.norm(abs_pos)),
                    'yaw': abs_yaw
                },
                'relative': {
                    'position': rel_pos.tolist() if isinstance(rel_pos, np.ndarray) else list(rel_pos),
                    'yaw': rel_yaw
                }
            }
            measurements.append(measurement)

            # 시각화 저장 (처음 50개 + 마지막 10개)
            if idx < 50 or idx >= len(rgb_files) - 10:
                self._save_visualization(
                    rgb, depth, mask, center_3d, state,
                    abs_pos, abs_yaw, idx
                )

            if (idx + 1) % 100 == 0:
                logger.info(f"  Processed {idx + 1}/{min(len(rgb_files), len(depth_csv_files))} frames")

        self.results['measurements'] = measurements

        # 통계 계산
        if measurements:
            self._calculate_statistics(
                measurements, positions_filtered, velocities,
                absolute_positions, absolute_yaws,
                relative_positions, relative_yaws
            )

            # 플롯 생성
            self._create_plots(
                timestamps, positions_raw, positions_filtered, velocities,
                yaws_raw, yaws_filtered,
                absolute_positions, absolute_yaws,
                relative_positions, relative_yaws
            )

        total_time = time.time() - start_time
        self.results['test_info'] = {
            'timestamp': datetime.now().isoformat(),
            'test_data_dir': str(self.test_data_dir),
            'total_frames': len(measurements),
            'sample_interval': self.sample_interval,
            'total_time': total_time
        }

        # 저장
        self._save_results()

        logger.info("=" * 60)
        logger.info(f"Test completed in {total_time:.2f}s")
        logger.info(f"Results saved to: {self.output_dir}")
        logger.info("=" * 60)

    def _calculate_statistics(self, measurements, positions_filtered, velocities,
                             absolute_positions, absolute_yaws,
                             relative_positions, relative_yaws):
        """통계 계산"""
        speeds = [m['speed'] for m in measurements]
        abs_distances = [m['absolute']['distance'] for m in measurements]

        self.results['statistics'] = {
            'total_frames': len(measurements),
            'duration_sec': measurements[-1]['time_from_ref'] if measurements else 0,

            'speed': {
                'mean': float(np.mean(speeds)),
                'max': float(np.max(speeds)),
                'min': float(np.min(speeds)),
                'std': float(np.std(speeds))
            },

            'absolute_position': {
                'max_distance': float(np.max(abs_distances)),
                'final_distance': float(abs_distances[-1]) if abs_distances else 0,
                'mean_x': float(np.mean([p[0] for p in absolute_positions])),
                'mean_y': float(np.mean([p[1] for p in absolute_positions])),
                'mean_z': float(np.mean([p[2] for p in absolute_positions])),
            },

            'absolute_yaw': {
                'max': float(np.max(absolute_yaws)),
                'min': float(np.min(absolute_yaws)),
                'range': float(np.max(absolute_yaws) - np.min(absolute_yaws)),
                'final': float(absolute_yaws[-1]) if absolute_yaws else 0
            },

            'relative_changes': {
                'avg_position_change': float(np.mean([np.linalg.norm(p) for p in relative_positions])),
                'avg_yaw_change': float(np.mean(np.abs(relative_yaws))),
                'max_yaw_change': float(np.max(np.abs(relative_yaws)))
            }
        }

    def _save_visualization(self, rgb, depth, mask, center_3d, state, abs_pos, abs_yaw, idx):
        """프레임 시각화 저장"""
        vis = rgb.copy()

        # 마스크 오버레이
        if mask is not None:
            mask_color = np.zeros_like(rgb)
            mask_color[:, :, 1] = mask
            vis = cv2.addWeighted(vis, 0.7, mask_color, 0.3, 0)

        # 중심점
        if center_3d is not None:
            cx = int(center_3d[0] * self.tracker.intrinsics['fx'] / center_3d[2] + self.tracker.intrinsics['cx'])
            cy = int(center_3d[1] * self.tracker.intrinsics['fy'] / center_3d[2] + self.tracker.intrinsics['cy'])
            cv2.circle(vis, (cx, cy), 8, (0, 0, 255), -1)

        # 정보 표시
        lines = [
            f"Frame: {idx}",
            f"Pos: ({state.position[0]:.3f}, {state.position[1]:.3f}, {state.position[2]:.3f})",
            f"Speed: {state.speed:.3f} m/s",
            f"Abs Dist: {np.linalg.norm(abs_pos):.3f} m",
            f"Abs Yaw: {abs_yaw:.1f} deg",
        ]

        y = 25
        for line in lines:
            cv2.putText(vis, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(vis, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            y += 22

        vis_path = self.output_dir / 'visualizations' / f"frame_{idx:05d}.jpg"
        cv2.imwrite(str(vis_path), vis)

    def _create_plots(self, timestamps, positions_raw, positions_filtered, velocities,
                     yaws_raw, yaws_filtered, absolute_positions, absolute_yaws,
                     relative_positions, relative_yaws):
        """측정 플롯 생성"""
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        # numpy 배열로 변환
        t = (np.array(timestamps) - timestamps[0]) / 1000.0  # 초
        pos_raw = np.array(positions_raw)
        pos_filt = np.array(positions_filtered)
        vel = np.array(velocities)
        abs_pos = np.array(absolute_positions)
        rel_pos = np.array(relative_positions)

        # === 플롯 1: 위치 및 속도 ===
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        fig.suptitle('Position & Velocity (Filtered)', fontsize=14)

        # X, Y, Z 위치
        for i, (ax, label) in enumerate(zip(axes[0], ['X', 'Y', 'Z'])):
            ax.plot(t, pos_filt[:, i], 'b-', linewidth=0.5)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'{label} (m)')
            ax.set_title(f'{label} Position')
            ax.grid(True)

        # 속도 성분
        for i, (ax, label) in enumerate(zip(axes[1], ['Vx', 'Vy', 'Vz'])):
            ax.plot(t, vel[:, i], 'g-', linewidth=0.5)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(f'{label} (m/s)')
            ax.set_title(f'{label} Velocity')
            ax.grid(True)

        plt.tight_layout()
        plt.savefig(self.output_dir / 'plots' / 'position_velocity.png', dpi=150)
        plt.close()

        # === 플롯 2: 절대 변화량 ===
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Absolute Changes (from Reference Frame)', fontsize=14)

        # 절대 위치 변화
        ax = axes[0, 0]
        ax.plot(t, abs_pos[:, 0], 'r-', label='dX', linewidth=0.5)
        ax.plot(t, abs_pos[:, 1], 'g-', label='dY', linewidth=0.5)
        ax.plot(t, abs_pos[:, 2], 'b-', label='dZ', linewidth=0.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (m)')
        ax.set_title('Absolute Position Change')
        ax.legend()
        ax.grid(True)

        # 절대 거리
        ax = axes[0, 1]
        abs_dist = np.linalg.norm(abs_pos, axis=1)
        ax.plot(t, abs_dist, 'm-', linewidth=0.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance (m)')
        ax.set_title('Absolute Distance from Reference')
        ax.grid(True)

        # 절대 Yaw
        ax = axes[1, 0]
        ax.plot(t, absolute_yaws, 'c-', linewidth=0.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Yaw (deg)')
        ax.set_title('Absolute Yaw Change')
        ax.grid(True)

        # XY 궤적
        ax = axes[1, 1]
        ax.plot(abs_pos[:, 0], abs_pos[:, 1], 'b-', linewidth=0.5)
        ax.plot(0, 0, 'ro', markersize=10, label='Reference')
        ax.set_xlabel('dX (m)')
        ax.set_ylabel('dY (m)')
        ax.set_title('XY Trajectory (from Reference)')
        ax.legend()
        ax.axis('equal')
        ax.grid(True)

        plt.tight_layout()
        plt.savefig(self.output_dir / 'plots' / 'absolute_changes.png', dpi=150)
        plt.close()

        # === 플롯 3: 상대 변화량 ===
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Relative Changes (Frame-to-Frame)', fontsize=14)

        # 상대 위치 변화
        ax = axes[0, 0]
        rel_dist = np.linalg.norm(rel_pos, axis=1)
        ax.plot(t, rel_dist, 'b-', linewidth=0.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance (m)')
        ax.set_title('Relative Position Change (per frame)')
        ax.grid(True)

        # 상대 Yaw 변화
        ax = axes[0, 1]
        ax.plot(t, relative_yaws, 'g-', linewidth=0.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Yaw (deg)')
        ax.set_title('Relative Yaw Change (per frame)')
        ax.grid(True)

        # 속도 크기
        ax = axes[1, 0]
        speeds = np.linalg.norm(vel, axis=1)
        ax.plot(t, speeds, 'r-', linewidth=0.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (m/s)')
        ax.set_title(f'Speed (avg: {np.mean(speeds):.4f} m/s)')
        ax.grid(True)

        # Yaw 비교
        ax = axes[1, 1]
        ax.plot(t, yaws_raw, 'r.', alpha=0.3, markersize=1, label='Raw')
        ax.plot(t, yaws_filtered, 'b-', linewidth=0.5, label='Filtered')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Yaw (deg)')
        ax.set_title('Yaw: Raw vs Filtered')
        ax.legend()
        ax.grid(True)

        plt.tight_layout()
        plt.savefig(self.output_dir / 'plots' / 'relative_changes.png', dpi=150)
        plt.close()

        logger.info("Plots saved")

    def _save_results(self):
        """결과 저장"""
        # JSON (간소화)
        summary = {
            'test_info': self.results['test_info'],
            'reference_frame': self.results['reference_frame'],
            'statistics': self.results['statistics'],
            'sample_measurements': self.results['measurements'][:10] + self.results['measurements'][-10:]
        }

        with open(self.output_dir / 'test_results.json', 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)

        # 전체 측정값 CSV
        with open(self.output_dir / 'measurements.csv', 'w') as f:
            f.write("frame,time_from_ref,pos_x,pos_y,pos_z,speed,yaw,abs_dist,abs_yaw,rel_dist,rel_yaw\n")
            for m in self.results['measurements']:
                f.write(f"{m['frame']},{m['time_from_ref']:.4f},"
                       f"{m['position_filtered'][0]:.6f},{m['position_filtered'][1]:.6f},{m['position_filtered'][2]:.6f},"
                       f"{m['speed']:.6f},{m['yaw_filtered']:.4f},"
                       f"{m['absolute']['distance']:.6f},{m['absolute']['yaw']:.4f},"
                       f"{np.linalg.norm(m['relative']['position']):.6f},{m['relative']['yaw']:.4f}\n")

        # 로그 파일
        with open(self.output_dir / 'test_log.txt', 'w', encoding='utf-8') as f:
            f.write("=" * 60 + "\n")
            f.write("Absolute/Relative Measurement Test Results\n")
            f.write("Test Data: 20251229_154835_test\n")
            f.write("=" * 60 + "\n\n")

            f.write(f"Timestamp: {self.results['test_info']['timestamp']}\n")
            f.write(f"Total Frames: {self.results['test_info']['total_frames']}\n")
            f.write(f"Sample Interval: {self.results['test_info']['sample_interval']}\n\n")

            f.write("[Reference Frame]\n")
            f.write("-" * 40 + "\n")
            ref = self.results['reference_frame']
            f.write(f"  Frame Index: {ref['frame_index']}\n")
            f.write(f"  Position: {ref['position']}\n")
            f.write(f"  Yaw: {ref['yaw']:.2f} deg\n\n")

            f.write("[Speed Statistics]\n")
            f.write("-" * 40 + "\n")
            spd = self.results['statistics']['speed']
            f.write(f"  Mean Speed: {spd['mean']:.4f} m/s\n")
            f.write(f"  Max Speed: {spd['max']:.4f} m/s\n")
            f.write(f"  Min Speed: {spd['min']:.4f} m/s\n")
            f.write(f"  Std Dev: {spd['std']:.4f} m/s\n\n")

            f.write("[Absolute Position (from Reference)]\n")
            f.write("-" * 40 + "\n")
            abspos = self.results['statistics']['absolute_position']
            f.write(f"  Max Distance: {abspos['max_distance']:.4f} m\n")
            f.write(f"  Final Distance: {abspos['final_distance']:.4f} m\n")
            f.write(f"  Mean dX: {abspos['mean_x']:.4f} m\n")
            f.write(f"  Mean dY: {abspos['mean_y']:.4f} m\n")
            f.write(f"  Mean dZ: {abspos['mean_z']:.4f} m\n\n")

            f.write("[Absolute Yaw (from Reference)]\n")
            f.write("-" * 40 + "\n")
            absyaw = self.results['statistics']['absolute_yaw']
            f.write(f"  Max: {absyaw['max']:.2f} deg\n")
            f.write(f"  Min: {absyaw['min']:.2f} deg\n")
            f.write(f"  Range: {absyaw['range']:.2f} deg\n")
            f.write(f"  Final: {absyaw['final']:.2f} deg\n\n")

            f.write("[Relative Changes (per frame)]\n")
            f.write("-" * 40 + "\n")
            rel = self.results['statistics']['relative_changes']
            f.write(f"  Avg Position Change: {rel['avg_position_change']:.6f} m\n")
            f.write(f"  Avg Yaw Change: {rel['avg_yaw_change']:.4f} deg\n")
            f.write(f"  Max Yaw Change: {rel['max_yaw_change']:.4f} deg\n")

        # 요약 이미지
        self._create_summary_image()

        logger.info(f"Results saved to {self.output_dir}")

    def _create_summary_image(self):
        """요약 이미지"""
        summary = np.ones((650, 800, 3), dtype=np.uint8) * 255

        lines = [
            "Absolute/Relative Measurement Test Summary",
            "Test Data: 20251229_154835_test",
            "",
            f"Total Frames: {self.results['test_info']['total_frames']}",
            f"Duration: {self.results['statistics'].get('duration_sec', 0):.2f} sec",
            "",
            "[Speed]",
            f"  Mean: {self.results['statistics']['speed']['mean']:.4f} m/s",
            f"  Max: {self.results['statistics']['speed']['max']:.4f} m/s",
            "",
            "[Absolute Position (from Reference)]",
            f"  Max Distance: {self.results['statistics']['absolute_position']['max_distance']:.4f} m",
            f"  Final Distance: {self.results['statistics']['absolute_position']['final_distance']:.4f} m",
            "",
            "[Absolute Yaw (from Reference)]",
            f"  Range: {self.results['statistics']['absolute_yaw']['range']:.2f} deg",
            f"  Final: {self.results['statistics']['absolute_yaw']['final']:.2f} deg",
            "",
            "[Relative Changes (per frame)]",
            f"  Avg Position: {self.results['statistics']['relative_changes']['avg_position_change']:.6f} m",
            f"  Avg Yaw: {self.results['statistics']['relative_changes']['avg_yaw_change']:.4f} deg",
        ]

        y = 30
        for line in lines:
            font_scale = 0.7 if line.startswith("Absolute/Relative") else 0.5
            thickness = 2 if line.startswith("Absolute/Relative") else 1
            cv2.putText(summary, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX,
                       font_scale, (0, 0, 0), thickness)
            y += 28

        cv2.imwrite(str(self.output_dir / 'test_summary.png'), summary)


def main():
    base_dir = Path(__file__).parent.parent

    test = AbsoluteRelativeMeasurementTest(
        test_data_dir="/root/fursys_img_251229/extraction/20251229_154835_test",
        output_dir=str(base_dir / "test_result" / "absolute_relative_measurement"),
        max_frames=1000,  # 최대 1000 프레임
        sample_interval=10  # 10프레임마다 샘플링 (약 3fps)
    )

    test.run_test()


if __name__ == "__main__":
    main()
