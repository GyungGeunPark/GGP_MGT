#!/usr/bin/env python3
"""
RealSense D455 Real-time 6DOF Pose Estimation Streaming with ROS2 Signal Publishing

Features:
- Real-time RGB + Depth capture from Intel RealSense D455
- GPU accelerated YOLO segmentation
- 6DOF pose estimation with coordinate axes visualization
- Kalman filter for smooth tracking
- YAW-based reference measurement (from test_result_22)
- Signal classification (-2, -1, 0, 1, 2, none) based on reference images
- ROI (Region of Interest) filtering for targeted detection
- Multi-object tracking with persistent ID assignment
- Object ordering: Right-to-left (rightmost object gets lowest ID)
- ROS2 Foxy signal publishing to 'p_s' topic
- OpenCV window display for real-time monitoring

Usage:
    python realsense_6dof_stream.py
    python realsense_6dof_stream.py --width 640 --height 480 --fps 30
    python realsense_6dof_stream.py --save output.mp4
    python realsense_6dof_stream.py --no-ros  # Disable ROS2 publishing
    python realsense_6dof_stream.py --no-roi  # Disable ROI filtering

Controls:
    'q' or ESC: Quit
    's': Save current frame as image
    'r': Reset baseline
    'p': Pause/Resume
    'f': Toggle fullscreen mode

Window:
    - Resizable: Drag window edges to resize
    - Maximize/Minimize: Use window title bar buttons
    - Fullscreen: Press 'f' key
"""

import sys
import os
import cv2
import numpy as np
import json
import time
import argparse
import re
import socket
import threading
from pathlib import Path
from collections import deque, OrderedDict
from datetime import datetime
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, field

# Add nimg_v3 to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from nimg_v3.measurement.pose_converter import Quaternion, PoseConverter
from nimg_v3.measurement.pose_kalman_filter import PoseKalmanFilter, FilterMode

# Check GPU availability
try:
    import cupy as cp
    HAS_CUPY = True
except ImportError:
    HAS_CUPY = False

import torch
HAS_CUDA = torch.cuda.is_available()

# RealSense
try:
    import pyrealsense2 as rs
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False
    print("[ERROR] pyrealsense2 not found. Please install it first.")
    sys.exit(1)

# ROS2 Foxy
HAS_ROS2 = False
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    HAS_ROS2 = True
except ImportError:
    print("[WARNING] ROS2 not found. Signal publishing will be disabled.")


# ========================== Configuration ==========================
# YOLOv26 2클래스 모델 설정
CONFIDENCE_THRESHOLD = 0.95
REFERENCE_IMAGE_FILE = "rgb_Color_20260113_000000_0_Color.png"
IOU_THRESHOLD = 0.3  # IoU threshold for object matching
MAX_LOST_FRAMES = 60  # Maximum frames an object can be lost before removal
MIN_DETECTIONS_FOR_VALID_OBJECT = 5  # Minimum detections to consider object valid
YAW_BASED_MEASUREMENT = True  # Enable yaw-based reference measurement
NONE_SIGNAL_ENABLED = True  # Enable 'none' signal for angles beyond -2 and +2 range
REFERENCE_DIR = "/root/fursys_imgprosessing_ws/src/nimg_v3/models/neural_fields"

# 클래스별 기준이미지 디렉토리 (YOLOv26 2클래스)
# housing_M: 0° 기준 상대각도 측정 (180° 동시 측정 비활성화됨)
# Wiring_tray: 0° 기준 상대각도 측정 (시그널 분류 비활성화됨)
STANDARD_DIRS = {
    0: "/root/fursys_imgprosessing_ws/src/yolo26_housingM_standard2",
    1: "/root/fursys_imgprosessing_ws/src/yolo26_tray_top_standard",
}
YOLO_MODEL_PATH = "/root/fursys_imgprosessing_ws/src/nimg_v3/models/yolo/yolo26_2class_seg_best_260324.pt"

# 클래스 이름 매핑 (YOLOv26 2클래스 모델)
CLASS_NAMES = {0: "housing_M", 1: "Wiring_tray"}
CLASS_COLORS_MAP = {0: (0, 255, 0), 1: (255, 128, 0)}  # housing_M: 녹색, Wiring_tray: 주황색

# ========================== Dual ROI Configuration ==========================
# Angle ROI - 각도/신호 측정용 (대폭 확대: 프레임 중앙~우측)
ANGLE_ROI_TOP_LEFT = (130, 80)       # (x1, y1)
ANGLE_ROI_BOTTOM_RIGHT = (260, 250)  # (x2, y2)
ANGLE_ROI_COLOR = (0, 255, 255)      # Yellow

# Speed ROI - 속도 측정용 (대폭 확대: 프레임 전체 커버)
# SPEED_ROI_TOP_LEFT = (0, 80)        # (x1, y1)
# SPEED_ROI_BOTTOM_RIGHT = (150, 250)  # (x2, y2)
SPEED_ROI_TOP_LEFT = (0, 0)        # (x1, y1)
SPEED_ROI_BOTTOM_RIGHT = (0, 0)  # (x2, y2)
SPEED_ROI_COLOR = (0, 165, 255)      # Orange

ROI_ENABLED = True
ROI_THICKNESS = 2

# ========================== TCP Configuration ==========================
TCP_HOST = '0.0.0.0'
TCP_PORT = 9999
# ===================================================================


def is_bbox_in_roi(bbox: List[int], roi_tl: Tuple[int, int], roi_br: Tuple[int, int],
                   min_overlap_ratio: float = 0.5) -> bool:
    """
    Check if bounding box is within ROI region.

    Args:
        bbox: [x1, y1, x2, y2] bounding box coordinates
        roi_tl: (x, y) top-left corner of ROI
        roi_br: (x, y) bottom-right corner of ROI
        min_overlap_ratio: Minimum overlap ratio required (0-1)

    Returns:
        True if bbox center is within ROI or sufficient overlap exists
    """
    x1, y1, x2, y2 = bbox
    roi_x1, roi_y1 = roi_tl
    roi_x2, roi_y2 = roi_br

    # Calculate bbox center
    center_x = (x1 + x2) / 2
    center_y = (y1 + y2) / 2

    # Check if center is within ROI
    if roi_x1 <= center_x <= roi_x2 and roi_y1 <= center_y <= roi_y2:
        return True

    # Calculate intersection area
    inter_x1 = max(x1, roi_x1)
    inter_y1 = max(y1, roi_y1)
    inter_x2 = min(x2, roi_x2)
    inter_y2 = min(y2, roi_y2)

    if inter_x2 <= inter_x1 or inter_y2 <= inter_y1:
        return False  # No intersection

    inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
    bbox_area = (x2 - x1) * (y2 - y1)

    overlap_ratio = inter_area / (bbox_area + 1e-6)

    return overlap_ratio >= min_overlap_ratio


def draw_roi_box(image: np.ndarray, roi_tl: Tuple[int, int], roi_br: Tuple[int, int],
                 color: Tuple[int, int, int] = (0, 255, 255), thickness: int = 2,
                 label: Optional[str] = None) -> np.ndarray:
    """Draw ROI box on image with label"""
    img = image.copy()

    # Draw ROI rectangle
    cv2.rectangle(img, roi_tl, roi_br, color, thickness)

    # Draw corner markers
    corner_size = 10
    x1, y1 = roi_tl
    x2, y2 = roi_br

    # Top-left corner
    cv2.line(img, (x1, y1), (x1 + corner_size, y1), color, thickness + 1)
    cv2.line(img, (x1, y1), (x1, y1 + corner_size), color, thickness + 1)

    # Top-right corner
    cv2.line(img, (x2, y1), (x2 - corner_size, y1), color, thickness + 1)
    cv2.line(img, (x2, y1), (x2, y1 + corner_size), color, thickness + 1)

    # Bottom-left corner
    cv2.line(img, (x1, y2), (x1 + corner_size, y2), color, thickness + 1)
    cv2.line(img, (x1, y2), (x1, y2 - corner_size), color, thickness + 1)

    # Bottom-right corner
    cv2.line(img, (x2, y2), (x2 - corner_size, y2), color, thickness + 1)
    cv2.line(img, (x2, y2), (x2, y2 - corner_size), color, thickness + 1)

    # ROI label
    if label is None:
        label = f"ROI [{x1},{y1}]-[{x2},{y2}]"
    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    cv2.rectangle(img, (x1, y1 - label_size[1] - 8), (x1 + label_size[0] + 4, y1), color, -1)
    cv2.putText(img, label, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    return img


@dataclass
class TrackedObject:
    """Persistent tracked object with state"""
    object_id: int
    kalman: 'PoseKalmanFilter'
    speed_filter: 'TemporalFilter'
    angle_filter: 'TemporalFilter'
    last_bbox: List[int] = field(default_factory=list)
    last_center_x: float = 0.0  # For right-to-left ordering
    last_position: Optional[np.ndarray] = None
    last_rotation: Optional[Dict[str, float]] = None
    detection_count: int = 0
    first_detection_frame: int = 0
    last_seen_frame: int = 0
    lost_frames: int = 0
    is_active: bool = True
    total_measurements: List[Dict] = field(default_factory=list)
    # Signal history for confirmation logic
    signal_history: List[str] = field(default_factory=list)
    last_signal_time: float = 0.0
    confirmed_signal: Optional[str] = None
    signal_published: bool = False


@dataclass
class DetectedObject:
    """Single detected object information"""
    object_id: int
    bbox: List[int]
    confidence: float
    mask: np.ndarray
    class_id: int
    class_name: str = ""
    center_x: float = 0.0
    center_2d: Optional[Tuple[int, int]] = None
    position: Optional[np.ndarray] = None
    rotation: Optional[Dict[str, float]] = None
    velocity: Optional[np.ndarray] = None
    speed: float = 0.0
    speed_m_min: float = 0.0           # m/min 단위 속도
    signal: str = "none"
    signal_confidence: float = 0.0
    relative_position: Optional[np.ndarray] = None
    relative_rotation: Optional[Dict[str, float]] = None
    relative_yaw: float = 0.0
    relative_yaw_0deg: float = 0.0     # housing_M: 0도 기준 상대각도
    relative_yaw_180deg: float = 0.0   # housing_M: 180도 기준 상대각도
    measurement_mode: str = "signal"   # "signal" 또는 "dual_angle"
    in_angle_roi: bool = False         # Angle ROI 내 여부
    in_speed_roi: bool = False         # Speed ROI 내 여부


class ROS2SignalPublisher:
    """ROS2 Foxy Signal Publisher for 'p_s' topic"""

    def __init__(self, topic_name='p_s'):
        self.topic_name = topic_name
        self.node = None
        self.publisher = None
        self.enabled = False

        if HAS_ROS2:
            try:
                rclpy.init()
                self.node = rclpy.create_node('pose_signal_publisher')
                self.publisher = self.node.create_publisher(String, topic_name, 10)
                self.enabled = True
                print(f"[ROS2] Publisher created for topic: '{topic_name}'")
            except Exception as e:
                print(f"[ROS2] Failed to initialize: {e}")
                self.enabled = False

    def publish(self, signal: str, object_id: int = 0, confidence: float = 0.0,
                relative_yaw: float = 0.0, position: np.ndarray = None,
                class_name: str = "", class_id: int = 0):
        """Publish signal message to ROS2 topic

        Message format: JSON with signal information
        {
            "signal": "-2" | "-1" | "0" | "1" | "2" | "none",
            "object_id": int,
            "class_name": str,
            "class_id": int,
            "confidence": float,
            "relative_yaw": float,
            "position": [x, y, z],
            "timestamp": float
        }
        """
        if not self.enabled or self.publisher is None:
            return

        try:
            msg_data = {
                "signal": signal,
                "object_id": object_id,
                "class_name": class_name,
                "class_id": class_id,
                "confidence": confidence,
                "relative_yaw": relative_yaw,
                "position": position.tolist() if position is not None else [0, 0, 0],
                "timestamp": time.time()
            }

            msg = String()
            msg.data = json.dumps(msg_data)
            self.publisher.publish(msg)

        except Exception as e:
            print(f"[ROS2] Publish error: {e}")

    def spin_once(self):
        """Process ROS2 callbacks once"""
        if self.enabled and self.node:
            rclpy.spin_once(self.node, timeout_sec=0.001)

    def shutdown(self):
        """Shutdown ROS2 node"""
        if self.enabled:
            try:
                if self.node:
                    self.node.destroy_node()
                rclpy.shutdown()
                print("[ROS2] Shutdown complete")
            except Exception as e:
                print(f"[ROS2] Shutdown error: {e}")


class TCPSignalPublisher:
    """TCP 서버 기반 시그널/속도 퍼블리셔

    백그라운드 스레드에서 TCP 서버를 실행하고, 다중 클라이언트 접속을 지원합니다.
    메시지는 JSON 문자열 + 개행문자(\\n)로 구분됩니다.
    """

    def __init__(self, host: str = '0.0.0.0', port: int = 9999):
        self.host = host
        self.port = port
        self.enabled = False
        self.server_socket = None
        self.clients: List[socket.socket] = []
        self.clients_lock = threading.Lock()
        self._accept_thread = None
        self._running = False

        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((host, port))
            self.server_socket.listen(5)
            self.server_socket.settimeout(1.0)
            self._running = True
            self.enabled = True

            self._accept_thread = threading.Thread(
                target=self._accept_loop, daemon=True, name="tcp_accept"
            )
            self._accept_thread.start()

            print(f"[TCP] 서버 시작: {host}:{port}")
        except Exception as e:
            print(f"[TCP] 서버 시작 실패: {e}")
            self.enabled = False

    def _accept_loop(self):
        """백그라운드에서 클라이언트 연결 수락"""
        while self._running:
            try:
                client, addr = self.server_socket.accept()
                client.settimeout(0.1)
                with self.clients_lock:
                    self.clients.append(client)
                print(f"[TCP] 클라이언트 연결: {addr} (총 {len(self.clients)}개)")
            except socket.timeout:
                continue
            except OSError:
                break

    def publish_speed(self, speed_m_min: float, object_id: int = 0):
        """속도 메시지 전송 (m/min)"""
        msg = {
            "type": "speed",
            "value": round(speed_m_min, 3),
            "unit": "m/min",
            "object_id": object_id,
            "timestamp": time.time()
        }
        self._send_to_all(json.dumps(msg) + "\n")

    def publish_angle(self, signal: str, relative_yaw: float,
                      object_id: int = 0, confidence: float = 0.0,
                      position: Optional[np.ndarray] = None,
                      class_name: str = "", class_id: int = 0):
        """각도/신호 메시지 전송 (signal 모드)"""
        msg = {
            "type": "angle",
            "signal": signal,
            "relative_yaw": round(relative_yaw, 2),
            "object_id": object_id,
            "class_name": class_name,
            "class_id": class_id,
            "confidence": round(confidence, 3),
            "position": position.tolist() if position is not None else [0, 0, 0],
            "timestamp": time.time()
        }
        self._send_to_all(json.dumps(msg) + "\n")

    def publish_dual_angle(self, relative_yaw_0deg: float, relative_yaw_180deg: float,
                           object_id: int = 0, confidence: float = 0.0,
                           position: Optional[np.ndarray] = None,
                           class_name: str = "", class_id: int = 0):
        """dual_angle 모드: 0도/180도 기준 두 상대각도 전송"""
        msg = {
            "type": "dual_angle",
            "relative_yaw_0deg": round(relative_yaw_0deg, 2),
            "relative_yaw_180deg": round(relative_yaw_180deg, 2),
            "object_id": object_id,
            "class_name": class_name,
            "class_id": class_id,
            "confidence": round(confidence, 3),
            "position": position.tolist() if position is not None else [0, 0, 0],
            "timestamp": time.time()
        }
        self._send_to_all(json.dumps(msg) + "\n")

    def _send_to_all(self, data: str):
        """모든 연결된 클라이언트에 데이터 전송"""
        if not self.enabled:
            return

        encoded = data.encode('utf-8')
        dead_clients = []

        with self.clients_lock:
            for client in self.clients:
                try:
                    client.sendall(encoded)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    dead_clients.append(client)

            for dead in dead_clients:
                try:
                    dead.close()
                except Exception:
                    pass
                self.clients.remove(dead)
                print(f"[TCP] 클라이언트 연결 끊김 (남은: {len(self.clients)}개)")

    @property
    def client_count(self) -> int:
        with self.clients_lock:
            return len(self.clients)

    def shutdown(self):
        """TCP 서버 종료"""
        self._running = False

        with self.clients_lock:
            for client in self.clients:
                try:
                    client.close()
                except Exception:
                    pass
            self.clients.clear()

        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass

        if self._accept_thread and self._accept_thread.is_alive():
            self._accept_thread.join(timeout=2.0)

        self.enabled = False
        print("[TCP] 서버 종료")


class SignalConfirmationManager:
    """
    시그널 확정 관리자

    0.2초 동안 객체가 감지되지 않으면 그동안 가장 빈도수가 높은 시그널을 확정하여 전송
    """

    CONFIRMATION_TIMEOUT = 0.2  # 0.2초 타임아웃

    def __init__(self, ros_publisher: Optional['ROS2SignalPublisher'] = None,
                 tcp_publisher: Optional['TCPSignalPublisher'] = None):
        self.ros_publisher = ros_publisher
        self.tcp_publisher = tcp_publisher
        self.object_signals: Dict[int, Dict] = {}  # object_id -> signal info

    def update_signal(self, object_id: int, signal: str, confidence: float,
                      relative_yaw: float, position: Optional[np.ndarray] = None,
                      class_name: str = "", class_id: int = 0,
                      measurement_mode: str = "signal",
                      relative_yaw_0deg: float = 0.0,
                      relative_yaw_180deg: float = 0.0):
        """객체의 시그널 업데이트"""
        current_time = time.time()

        if object_id not in self.object_signals:
            self.object_signals[object_id] = {
                'signal_history': [],
                'last_update_time': current_time,
                'confirmed_signal': None,
                'published': False,
                'last_position': position,
                'last_confidence': confidence,
                'last_relative_yaw': relative_yaw,
                'class_name': class_name,
                'class_id': class_id,
                'measurement_mode': measurement_mode,
                'last_relative_yaw_0deg': relative_yaw_0deg,
                'last_relative_yaw_180deg': relative_yaw_180deg,
                'yaw_0deg_history': [],
                'yaw_180deg_history': [],
            }

        obj_info = self.object_signals[object_id]

        # 시그널 히스토리에 추가 (none 포함)
        obj_info['signal_history'].append(signal)

        obj_info['last_update_time'] = current_time
        obj_info['last_position'] = position
        obj_info['last_confidence'] = confidence
        obj_info['last_relative_yaw'] = relative_yaw
        obj_info['class_name'] = class_name
        obj_info['class_id'] = class_id
        obj_info['measurement_mode'] = measurement_mode
        obj_info['last_relative_yaw_0deg'] = relative_yaw_0deg
        obj_info['last_relative_yaw_180deg'] = relative_yaw_180deg
        obj_info['published'] = False

        if measurement_mode == "dual_angle":
            obj_info['yaw_0deg_history'].append(relative_yaw_0deg)
            obj_info['yaw_180deg_history'].append(relative_yaw_180deg)

    def check_and_publish(self, current_time: Optional[float] = None) -> List[Dict]:
        """
        타임아웃된 객체들의 시그널을 확정하고 발행

        Returns:
            발행된 시그널 정보 리스트
        """
        if current_time is None:
            current_time = time.time()

        published_signals = []
        objects_to_remove = []

        for object_id, obj_info in self.object_signals.items():
            time_since_update = current_time - obj_info['last_update_time']

            # 타임아웃 체크: 0.2초 동안 업데이트가 없으면 확정
            if time_since_update >= self.CONFIRMATION_TIMEOUT and not obj_info['published']:
                if obj_info['signal_history']:
                    from collections import Counter

                    measurement_mode = obj_info.get('measurement_mode', 'signal')
                    class_name = obj_info.get('class_name', '')
                    class_id = obj_info.get('class_id', 0)

                    if measurement_mode == "dual_angle":
                        # 통합 인터페이스: 0도 기준 상대각도만 퍼블리시
                        # 최대빈도(mode) 방식: 정수 반올림 후 가장 빈번한 값 사용
                        yaw_history = obj_info.get('yaw_0deg_history', [0.0])
                        rounded_yaws = [round(y) for y in yaw_history]
                        yaw_counts = Counter(rounded_yaws)
                        avg_yaw_0 = float(yaw_counts.most_common(1)[0][0])
                        most_common_signal = "raw"

                        obj_info['confirmed_signal'] = most_common_signal
                        obj_info['published'] = True

                        if self.ros_publisher and self.ros_publisher.enabled:
                            self.ros_publisher.publish(
                                signal=most_common_signal,
                                object_id=object_id,
                                confidence=obj_info['last_confidence'],
                                relative_yaw=avg_yaw_0,
                                position=obj_info['last_position'],
                                class_name=class_name,
                                class_id=class_id
                            )

                        if self.tcp_publisher and self.tcp_publisher.enabled:
                            self.tcp_publisher.publish_angle(
                                signal=most_common_signal,
                                relative_yaw=avg_yaw_0,
                                object_id=object_id,
                                confidence=obj_info['last_confidence'],
                                position=obj_info['last_position'],
                                class_name=class_name,
                                class_id=class_id
                            )

                        published_signals.append({
                            'object_id': object_id,
                            'signal': most_common_signal,
                            'measurement_mode': 'dual_angle',
                            'relative_yaw_0deg': avg_yaw_0,
                            'confidence': obj_info['last_confidence'],
                        })

                        print(f"[Angle] Object {object_id} ({class_name}): "
                              f"d0={avg_yaw_0:+.1f}°")

                        obj_info['signal_history'] = []
                        obj_info['yaw_0deg_history'] = []
                        obj_info['yaw_180deg_history'] = []
                    else:
                        signal_counts = Counter(obj_info['signal_history'])
                        most_common_signal = signal_counts.most_common(1)[0][0]

                        obj_info['confirmed_signal'] = most_common_signal
                        obj_info['published'] = True

                        if self.ros_publisher and self.ros_publisher.enabled:
                            self.ros_publisher.publish(
                                signal=most_common_signal,
                                object_id=object_id,
                                confidence=obj_info['last_confidence'],
                                relative_yaw=obj_info['last_relative_yaw'],
                                position=obj_info['last_position'],
                                class_name=class_name,
                                class_id=class_id
                            )

                        if self.tcp_publisher and self.tcp_publisher.enabled:
                            self.tcp_publisher.publish_angle(
                                signal=most_common_signal,
                                relative_yaw=obj_info['last_relative_yaw'],
                                object_id=object_id,
                                confidence=obj_info['last_confidence'],
                                position=obj_info['last_position'],
                                class_name=class_name,
                                class_id=class_id
                            )

                        published_signals.append({
                            'object_id': object_id,
                            'signal': most_common_signal,
                            'confidence': obj_info['last_confidence'],
                            'signal_history': obj_info['signal_history'].copy(),
                            'signal_counts': dict(signal_counts)
                        })

                        print(f"[Signal] Object {object_id} ({class_name}): Confirmed signal '{most_common_signal}' "
                              f"(history: {dict(signal_counts)})")

                        obj_info['signal_history'] = []

            # 오래된 객체 정리 (2초 이상 업데이트 없으면 제거)
            if time_since_update > 2.0:
                objects_to_remove.append(object_id)

        # 오래된 객체 제거
        for obj_id in objects_to_remove:
            del self.object_signals[obj_id]

        return published_signals

    def get_confirmed_signal(self, object_id: int) -> Optional[str]:
        """객체의 확정된 시그널 반환"""
        if object_id in self.object_signals:
            return self.object_signals[object_id].get('confirmed_signal')
        return None

    def reset_object(self, object_id: int):
        """객체의 시그널 히스토리 리셋"""
        if object_id in self.object_signals:
            self.object_signals[object_id]['signal_history'] = []
            self.object_signals[object_id]['confirmed_signal'] = None
            self.object_signals[object_id]['published'] = False


@dataclass
class ShapeFeatures:
    """Shape features for signal matching"""
    bbox_aspect: float = 0.0      # width/height ratio
    circularity: float = 0.0      # 4*pi*area/perimeter^2
    solidity: float = 0.0         # area/convex_hull_area
    extent: float = 0.0           # area/bbox_area
    hu_moments: np.ndarray = field(default_factory=lambda: np.zeros(7))


class ReferenceAngle:
    """Reference image angle information"""
    def __init__(self, angle_id: int, image_path: str, yaw_angle: float,
                 image: np.ndarray, mask_features: np.ndarray,
                 shape_features: Optional[ShapeFeatures] = None):
        self.angle_id = angle_id
        self.image_path = image_path
        self.yaw_angle = yaw_angle
        self.image = image
        self.mask_features = mask_features
        self.shape_features = shape_features or ShapeFeatures()


class ReferencePose:
    """Reference pose extracted from specific image (yaw-based)"""
    def __init__(self, position: np.ndarray, rotation: Dict[str, float],
                 mask_features: np.ndarray, yaw_angle: float, image_path: str):
        self.position = position
        self.rotation = rotation
        self.mask_features = mask_features
        self.yaw_angle = yaw_angle
        self.image_path = image_path
        self.is_set = True


class ShapeBasedSignalGenerator:
    """Shape-feature based signal generator using reference images

    Uses geometric shape features (aspect ratio, circularity, solidity, extent)
    instead of PCA-based yaw angle for robust signal matching.

    클래스별 참조이미지 지원:
    - reference_config.yaml 기반 로딩 (YOLO 라벨에서 마스크 생성)
    - 기존 rgb_Color_*.png 파일명 기반 로딩 (하위 호환)
    """

    # Feature weights for distance calculation
    WEIGHT_ASPECT = 10.0      # bbox_aspect is the primary discriminator
    WEIGHT_CIRCULARITY = 5.0
    WEIGHT_SOLIDITY = 3.0
    WEIGHT_EXTENT = 3.0

    # Threshold for 'none' signal (if min distance > threshold)
    NONE_DISTANCE_THRESHOLD = 2.5

    def __init__(self, reference_dir: str, yolo_model_path: str,
                 reference_image_file: str = "",
                 standard_dirs: Optional[Dict[int, str]] = None):
        self.reference_dir = Path(reference_dir)
        self.yolo_model_path = yolo_model_path
        self.references: Dict[int, ReferenceAngle] = {}
        self.angle_boundaries: Dict[int, Tuple[float, float]] = {}
        self.reference_pose: Optional[ReferencePose] = None
        self.reference_image_file = reference_image_file
        self.reference_yaw: float = 0.0

        # Relative yaw for each reference (computed after loading)
        self.ref_relative_yaws: Dict[int, float] = {}
        self.sorted_ref_order: List[int] = []
        self.sorted_ref_yaws: List[float] = []
        self.yaw_boundaries: Dict[Tuple[int, int], float] = {}

        # Relative yaw boundaries for 'none' signal detection (kept for compatibility)
        self.min_relative_yaw: float = -180.0
        self.max_relative_yaw: float = 180.0

        # 클래스별 기준이미지 세트 (class_id → {signal_id → ReferenceAngle})
        self.class_references: Dict[int, Dict[int, ReferenceAngle]] = {}
        self.class_ref_relative_yaws: Dict[int, Dict[int, float]] = {}
        self.class_reference_yaw: Dict[int, float] = {}
        self.class_none_bounds: Dict[int, Tuple[float, float]] = {}

        # 클래스별 측정 모드 ("signal" 또는 "dual_angle")
        self.class_measurement_mode: Dict[int, str] = {}
        # dual_angle 모드: {class_id: {angle_deg: yaw_value}} (0°, 180° 기준 yaw)
        self.class_baseline_yaws: Dict[int, Dict[int, float]] = {}

        # Depth 기반 각도 측정 강화
        self._intrinsics: Optional[Dict] = None
        # 180° 모호성 자동보정: depth gradient 방향과 기준이미지 PCA 방향 간 오프셋
        self._depth_ref_calibrated: Dict[int, bool] = {}      # {class_id: calibrated?}
        self._depth_ref_offset_180: Dict[int, bool] = {}      # {class_id: 180° flip needed?}
        self._depth_calibration_samples: Dict[int, list] = {} # {class_id: [relative_yaw samples]}
        self._DEPTH_CALIBRATION_COUNT = 10  # 자동보정에 필요한 샘플 수


        from ultralytics import YOLO
        self.yolo = YOLO(yolo_model_path)

        # 클래스별 기준이미지 디렉토리가 있으면 멀티클래스 로딩
        if standard_dirs:
            self._load_multiclass_standards(standard_dirs)
        else:
            # 기존 단일 참조 디렉토리 로딩 (하위 호환)
            self._load_reference_images()
            self._compute_angle_boundaries()
            self._set_reference_pose()

    def _load_reference_images(self):
        """Load reference images and extract YAW angles for signal matching"""
        print(f"\n[Signal] Loading reference images from: {self.reference_dir}")
        print(f"[Signal] Using YAW-BASED matching (PCA yaw angle)")

        # Support both .jpg and .png files
        # Pattern for new format: rgb_Color_YYYYMMDD_000000_X_Color.png
        # Pattern for old format: rgb_Color_YYYYMMDD_000000_X_png.rf.*.jpg
        all_images = list(self.reference_dir.glob("rgb_Color_*.jpg")) + \
                     list(self.reference_dir.glob("rgb_Color_*.png"))

        for img_path in sorted(all_images):
            filename = img_path.name
            # Extract angle ID from filename
            angle_id = None
            if "_-2_" in filename:
                angle_id = -2
            elif "_-1_" in filename:
                angle_id = -1
            elif "_0_" in filename:
                angle_id = 0
            elif "_1_" in filename and "_-1_" not in filename:
                angle_id = 1
            elif "_2_" in filename and "_-2_" not in filename:
                angle_id = 2

            if angle_id is not None and angle_id in [-2, -1, 0, 1, 2]:
                # Skip if already loaded (prefer newer files)
                if angle_id in self.references:
                    continue

                image = cv2.imread(str(img_path))
                if image is None:
                    continue

                mask_features, yaw_angle = self._extract_features(image)
                shape_features = self._extract_shape_features(image)

                self.references[angle_id] = ReferenceAngle(
                    angle_id=angle_id,
                    image_path=str(img_path),
                    yaw_angle=yaw_angle,
                    image=image,
                    mask_features=mask_features,
                    shape_features=shape_features
                )
                print(f"  [Signal] Loaded reference {angle_id:+d}: yaw={yaw_angle:.2f} deg")

        print(f"[Signal] Total reference images loaded: {len(self.references)}")

        # Store reference yaw (Signal 0) as baseline
        if 0 in self.references:
            self.reference_yaw = self.references[0].yaw_angle
            print(f"[Signal] Reference (Signal 0) yaw: {self.reference_yaw:.2f} deg (baseline)")

        # Calculate and store relative yaw for each reference
        if all(ref_id in self.references for ref_id in [-2, -1, 0, 1, 2]):
            print(f"[Signal] Relative yaw angles (from Signal 0):")
            self.ref_relative_yaws = {}
            for ref_id in [-2, -1, 0, 1, 2]:
                rel_yaw = self._normalize_angle(self.references[ref_id].yaw_angle - self.reference_yaw)
                self.ref_relative_yaws[ref_id] = rel_yaw
                print(f"  Signal {ref_id:+d}: relative_yaw = {rel_yaw:.2f} deg")

            # Calculate yaw boundaries between references
            self._compute_yaw_boundaries()

    @staticmethod
    def _polygon_to_mask(label_path: str, img_width: int, img_height: int) -> np.ndarray:
        """YOLO 세그멘테이션 폴리곤 라벨 → 바이너리 마스크"""
        mask = np.zeros((img_height, img_width), dtype=np.uint8)
        if not os.path.exists(label_path):
            return mask
        with open(label_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) < 5:
                    continue
                coords = list(map(float, parts[1:]))
                points = []
                for i in range(0, len(coords), 2):
                    if i + 1 < len(coords):
                        x = int(coords[i] * img_width)
                        y = int(coords[i + 1] * img_height)
                        points.append([x, y])
                if len(points) >= 3:
                    pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
                    cv2.fillPoly(mask, [pts], 255)
        return mask

    def _load_multiclass_standards(self, standard_dirs: Dict[int, str]):
        """클래스별 기준이미지 로딩 (reference_config.yaml 또는 기존 방식)"""
        import yaml as _yaml

        for class_id, ref_dir in standard_dirs.items():
            ref_path = Path(ref_dir)
            config_path = ref_path / "reference_config.yaml"
            class_name = CLASS_NAMES.get(class_id, f"class_{class_id}")

            print(f"\n[Signal] 클래스 {class_id} ({class_name}) 참조 로딩: {ref_dir}")

            if config_path.exists():
                self._load_from_config(class_id, ref_path, config_path, _yaml)
            else:
                # 기존 rgb_Color 패턴 fallback
                print(f"  [Signal] reference_config.yaml 없음, 기존 방식 시도")
                self._load_legacy_references(class_id, ref_path)

            # 결과 출력
            mode = self.class_measurement_mode.get(class_id, "signal")
            if mode == "dual_angle" and class_id in self.class_baseline_yaws:
                baselines = self.class_baseline_yaws[class_id]
                print(f"  [Signal] 클래스 {class_id} ({class_name}): dual_angle 모드")
                for angle_deg, yaw_val in sorted(baselines.items()):
                    print(f"    기준 {angle_deg}°: PCA yaw={yaw_val:.2f}°")
            elif class_id in self.class_references:
                refs = self.class_references[class_id]
                print(f"  [Signal] 클래스 {class_id}: {len(refs)}개 참조 로딩됨 (signal 모드)")
                for sig_id in sorted(refs.keys()):
                    ref = refs[sig_id]
                    print(f"    Signal {sig_id:+d}: yaw={ref.yaw_angle:.2f}°")

        # 기본 클래스 참조를 self.references에도 설정 (하위 호환)
        if self.class_references:
            first_class = min(self.class_references.keys())
            self.references = self.class_references[first_class]
            if first_class in self.class_reference_yaw:
                self.reference_yaw = self.class_reference_yaw[first_class]
            if first_class in self.class_ref_relative_yaws:
                self.ref_relative_yaws = self.class_ref_relative_yaws[first_class]

    def _load_from_config(self, class_id: int, ref_path: Path,
                          config_path: Path, _yaml):
        """reference_config.yaml에서 참조 로딩"""
        with open(config_path, 'r', encoding='utf-8') as f:
            config = _yaml.safe_load(f)

        measurement_mode = config.get('measurement_mode', 'signal')
        self.class_measurement_mode[class_id] = measurement_mode

        images_dir = ref_path / "train" / "images"
        labels_dir = ref_path / "train" / "labels"
        if not images_dir.exists():
            images_dir = ref_path / "images"
            labels_dir = ref_path / "labels"

        if measurement_mode == "dual_angle":
            self._load_dual_angle_config(class_id, config, images_dir, labels_dir)
            return

        # signal 모드 (기존 방식)
        signal_mapping = config.get('signal_mapping', {})
        none_lower = config.get('none_lower_bound', -36.0)
        none_upper = config.get('none_upper_bound', 46.0)
        self.class_none_bounds[class_id] = (none_lower, none_upper)

        refs: Dict[int, ReferenceAngle] = {}

        for sig_id, img_filename in signal_mapping.items():
            sig_id = int(sig_id)
            img_file = images_dir / img_filename
            if not img_file.exists():
                img_file = ref_path / img_filename
            if not img_file.exists():
                print(f"    [경고] Signal {sig_id:+d} 이미지 없음: {img_filename}")
                continue

            image = cv2.imread(str(img_file))
            if image is None:
                continue

            h, w = image.shape[:2]

            lbl_name = os.path.splitext(img_filename)[0] + ".txt"
            lbl_path = labels_dir / lbl_name
            if lbl_path.exists():
                mask = self._polygon_to_mask(str(lbl_path), w, h)
            else:
                mask_features, yaw_angle = self._extract_features(image)
                refs[sig_id] = ReferenceAngle(
                    angle_id=sig_id, image_path=str(img_file),
                    yaw_angle=yaw_angle, image=image,
                    mask_features=mask_features
                )
                continue

            yaw_angle = self._compute_yaw_from_mask(mask)
            features = self._compute_mask_features(mask)

            refs[sig_id] = ReferenceAngle(
                angle_id=sig_id, image_path=str(img_file),
                yaw_angle=yaw_angle, image=image,
                mask_features=features
            )

        self.class_references[class_id] = refs

        # 기준 yaw (Signal 0) 설정
        if 0 in refs:
            baseline_yaw = refs[0].yaw_angle
            self.class_reference_yaw[class_id] = baseline_yaw

            # 상대 yaw 계산
            rel_yaws = {}
            for sig_id, ref in refs.items():
                rel_yaws[sig_id] = self._normalize_angle(ref.yaw_angle - baseline_yaw)
            self.class_ref_relative_yaws[class_id] = rel_yaws

            print(f"  [Signal] 클래스 {class_id} 기준 yaw: {baseline_yaw:.2f}°")
            for sig_id in sorted(rel_yaws.keys()):
                print(f"    Signal {sig_id:+d}: rel_yaw={rel_yaws[sig_id]:+.2f}°")

    def _load_dual_angle_config(self, class_id: int, config: dict,
                                images_dir: Path, labels_dir: Path):
        """dual_angle 모드 기준이미지 로딩 (0°, 180° 두 기준점)

        depth .raw 파일이 있으면 3D PCA로 기준 yaw를 계산하여
        실시간 3D PCA 측정과 동일한 좌표계를 사용한다.
        """
        baseline_angles = config.get('baseline_angles', {})
        baselines: Dict[int, float] = {}

        for angle_deg, img_filename in baseline_angles.items():
            angle_deg = int(angle_deg)
            img_file = images_dir / img_filename
            if not img_file.exists():
                img_file = Path(images_dir).parent / img_filename
            if not img_file.exists():
                print(f"    [경고] 기준 {angle_deg}° 이미지 없음: {img_filename}")
                continue

            image = cv2.imread(str(img_file))
            if image is None:
                print(f"    [경고] 이미지 로드 실패: {img_filename}")
                continue

            h, w = image.shape[:2]

            # 라벨에서 마스크 생성
            lbl_name = os.path.splitext(img_filename)[0] + ".txt"
            lbl_path = labels_dir / lbl_name
            if lbl_path.exists():
                mask = self._polygon_to_mask(str(lbl_path), w, h)
            else:
                mask = None

            # depth .raw 파일 탐색: 이미지명에서 depth 파일명 유추
            # 예: 0deg_png.rf.XXX.png → 0deg_depth_png.rf.XXX.raw
            stem = os.path.splitext(img_filename)[0]  # "0deg_png.rf.XXX"
            parts = stem.split("_", 1)  # ["0deg", "png.rf.XXX"]
            depth_stem = parts[0] + "_depth_" + parts[1] if len(parts) > 1 else stem + "_depth"
            depth_raw_file = images_dir / (depth_stem + ".raw")
            depth_csv_file = images_dir / (depth_stem + ".csv")

            pca_mode = "2D"
            if mask is not None and depth_raw_file.exists() and depth_csv_file.exists():
                # depth + intrinsics로 3D PCA 계산
                ref_depth, ref_intrinsics = self._load_depth_raw(depth_raw_file, depth_csv_file, w, h)
                if ref_depth is not None and ref_intrinsics is not None:
                    # 임시로 intrinsics 설정 후 3D PCA 계산
                    saved_intrinsics = self._intrinsics
                    self._intrinsics = ref_intrinsics
                    yaw_angle, _, used_3d = self._compute_yaw_with_depth(mask, ref_depth)
                    self._intrinsics = saved_intrinsics
                    if used_3d:
                        pca_mode = "3D"
                    else:
                        yaw_angle = self._compute_yaw_from_mask(mask) if mask is not None else 0.0
                else:
                    yaw_angle = self._compute_yaw_from_mask(mask)
            elif mask is not None:
                yaw_angle = self._compute_yaw_from_mask(mask)
            else:
                _, yaw_angle = self._extract_features(image)

            baselines[angle_deg] = yaw_angle
            print(f"    기준 {angle_deg}°: {pca_mode} PCA yaw={yaw_angle:.2f}° ({img_filename})")

        self.class_baseline_yaws[class_id] = baselines

        # 0° 기준 yaw를 class_reference_yaw에도 설정 (get_relative_yaw_only 호환)
        if 0 in baselines:
            self.class_reference_yaw[class_id] = baselines[0]

    def _load_depth_raw(self, raw_path: Path, csv_path: Path,
                        img_w: int, img_h: int) -> Tuple[Optional[np.ndarray], Optional[Dict]]:
        """depth .raw (Z16) 파일과 intrinsics .csv 파일 로드

        Returns:
            (depth_meters, intrinsics_dict) 또는 (None, None)
        """
        try:
            # CSV에서 intrinsics 파싱
            intrinsics = {}
            with open(str(csv_path), 'r') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('Fx,'):
                        intrinsics['fx'] = float(line.split(',')[1])
                    elif line.startswith('Fy,'):
                        intrinsics['fy'] = float(line.split(',')[1])
                    elif line.startswith('PPx,'):
                        intrinsics['cx'] = float(line.split(',')[1])
                    elif line.startswith('PPy,'):
                        intrinsics['cy'] = float(line.split(',')[1])
                    elif line.startswith('Resolution x,'):
                        depth_w = int(line.split(',')[1])
                    elif line.startswith('Resolution y,'):
                        depth_h = int(line.split(',')[1])

            if not all(k in intrinsics for k in ('fx', 'fy', 'cx', 'cy')):
                print(f"    [경고] intrinsics 불완전: {csv_path}")
                return None, None

            # .raw 파일에서 Z16 depth 로드
            raw_data = np.fromfile(str(raw_path), dtype=np.uint16)
            expected_size = depth_w * depth_h
            if raw_data.size != expected_size:
                print(f"    [경고] depth raw 크기 불일치: {raw_data.size} != {expected_size}")
                return None, None

            depth_uint16 = raw_data.reshape(depth_h, depth_w)
            # Z16: 밀리미터 → 미터 변환
            depth_meters = depth_uint16.astype(np.float32) / 1000.0

            # depth 해상도와 이미지 해상도가 다르면 리사이즈
            if depth_h != img_h or depth_w != img_w:
                depth_meters = cv2.resize(depth_meters, (img_w, img_h), interpolation=cv2.INTER_NEAREST)

            print(f"    [Depth] 로드 성공: {raw_path.name} ({depth_w}x{depth_h})")
            return depth_meters, intrinsics

        except Exception as e:
            print(f"    [경고] depth 로드 실패: {e}")
            return None, None

    def _load_legacy_references(self, class_id: int, ref_path: Path):
        """기존 rgb_Color_*.png 패턴으로 참조 로딩 (하위 호환)"""
        all_images = list(ref_path.glob("rgb_Color_*.jpg")) + \
                     list(ref_path.glob("rgb_Color_*.png"))
        refs: Dict[int, ReferenceAngle] = {}

        for img_path in sorted(all_images):
            filename = img_path.name
            angle_id = None
            if "_-2_" in filename:
                angle_id = -2
            elif "_-1_" in filename:
                angle_id = -1
            elif "_0_" in filename:
                angle_id = 0
            elif "_1_" in filename and "_-1_" not in filename:
                angle_id = 1
            elif "_2_" in filename and "_-2_" not in filename:
                angle_id = 2

            if angle_id is not None and angle_id not in refs:
                image = cv2.imread(str(img_path))
                if image is None:
                    continue
                mask_features, yaw_angle = self._extract_features(image)
                refs[angle_id] = ReferenceAngle(
                    angle_id=angle_id, image_path=str(img_path),
                    yaw_angle=yaw_angle, image=image,
                    mask_features=mask_features
                )

        self.class_references[class_id] = refs

        if 0 in refs:
            baseline_yaw = refs[0].yaw_angle
            self.class_reference_yaw[class_id] = baseline_yaw
            rel_yaws = {}
            for sig_id, ref in refs.items():
                rel_yaws[sig_id] = self._normalize_angle(ref.yaw_angle - baseline_yaw)
            self.class_ref_relative_yaws[class_id] = rel_yaws

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to -180 ~ +180 range"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle

    # ── Depth 기반 각도 측정 강화 메서드 ──────────────────────────────

    def set_intrinsics(self, intrinsics: Dict):
        """카메라 내부 파라미터 설정 (depth 기반 각도 측정에 필요)"""
        self._intrinsics = intrinsics
        print(f"[Signal] Depth-enhanced angle: intrinsics set "
              f"(fx={intrinsics['fx']:.1f}, fy={intrinsics['fy']:.1f}, "
              f"cx={intrinsics['cx']:.1f}, cy={intrinsics['cy']:.1f})")

    def _filter_mask_by_depth(self, mask: np.ndarray, depth: np.ndarray,
                              z_threshold_m: float = 0.03) -> np.ndarray:
        """Depth 기반 마스크 정제 — 깊이 이상치 픽셀 제거.

        YOLO 마스크 경계에 섞인 배경 픽셀이나 노이즈를 깊이값 기준으로 걸러내어
        PCA 입력을 깨끗하게 만든다.

        Args:
            mask: 이진 마스크 (uint8, 0/255)
            depth: 깊이 이미지 (float32, 미터)
            z_threshold_m: 중앙값 대비 허용 깊이 편차 (기본 30mm)

        Returns:
            정제된 이진 마스크. 유효 픽셀 부족 시 원본 반환.
        """
        valid = (mask > 0) & (depth > 0.1) & (depth < 10.0)
        n_valid = int(np.sum(valid))

        if n_valid < 20:
            return mask  # depth 데이터 부족 → 원본 사용

        median_z = float(np.median(depth[valid]))

        # 중앙값 ± threshold 범위 내 픽셀만 유지
        refined = valid & (np.abs(depth - median_z) < z_threshold_m)
        n_refined = int(np.sum(refined))

        if n_refined < 20:
            return mask  # 필터링이 너무 공격적 → 원본 사용

        result = np.zeros_like(mask)
        result[refined] = 255
        return result

    def _compute_yaw_with_depth(self, mask: np.ndarray, depth: np.ndarray
                                ) -> Tuple[float, int, bool]:
        """3D 메트릭 공간 PCA + depth gradient 180° 모호성 해결.

        1) depth 기반 마스크 정제 (배경 노이즈 제거)
        2) 마스크 픽셀을 카메라 내부 파라미터로 3D 좌표 (X, Y)에 투영
        3) X-Y 평면 PCA → 원근 왜곡이 보정된 yaw 각도
        4) 주축 방향의 depth gradient로 180° 방향 결정
           Convention: 주축이 카메라에 가까운 쪽(낮은 depth)을 가리키도록 정규화

        Args:
            mask: 이진 마스크 (uint8)
            depth: 깊이 이미지 (float32, 미터)

        Returns:
            (yaw_deg, depth_direction, depth_used)
            - yaw_deg: PCA yaw (-180 ~ +180)
            - depth_direction: +1=양방향 끝이 가까움, -1=반전됨, 0=비대칭 불충분
            - depth_used: True이면 depth가 성공적으로 사용됨
        """
        if self._intrinsics is None:
            return self._compute_yaw_from_mask(mask), 0, False

        # 1) depth 기반 마스크 정제
        refined_mask = self._filter_mask_by_depth(mask, depth)

        # 2) 마스크 픽셀 좌표 + depth 추출
        ys, xs = np.where(refined_mask > 0)
        if len(ys) < 20:
            return self._compute_yaw_from_mask(mask), 0, False

        depths = depth[ys, xs]
        valid = (depths > 0.1) & (depths < 10.0)
        if np.sum(valid) < 20:
            return self._compute_yaw_from_mask(mask), 0, False

        xs_v = xs[valid].astype(np.float64)
        ys_v = ys[valid].astype(np.float64)
        zs_v = depths[valid].astype(np.float64)

        # 3) 3D 메트릭 좌표 투영 (카메라 내부 파라미터 사용)
        fx = self._intrinsics['fx']
        fy = self._intrinsics['fy']
        cx = self._intrinsics['cx']
        cy = self._intrinsics['cy']

        X = (xs_v - cx) * zs_v / fx
        Y = (ys_v - cy) * zs_v / fy

        # 4) X-Y 평면 PCA (위에서 내려다보는 카메라 → 수평면)
        points = np.column_stack([X, Y])
        mean_pt = np.mean(points, axis=0)
        centered = points - mean_pt

        try:
            cov = np.cov(centered.T)
            eigenvalues, eigenvectors = np.linalg.eigh(cov)
        except Exception:
            return self._compute_yaw_from_mask(mask), 0, False

        idx = np.argsort(eigenvalues)[::-1]
        eigenvectors = eigenvectors[:, idx]
        primary_axis = eigenvectors[:, 0]

        yaw = np.degrees(np.arctan2(primary_axis[1], primary_axis[0]))

        # 5) Depth gradient 180° 모호성 해결
        projections = centered @ primary_axis
        pos_mask = projections > 0
        neg_mask = projections <= 0

        depth_direction = 0  # 0 = 비대칭 불충분 / 판단 불가
        n_pos = int(np.sum(pos_mask))
        n_neg = int(np.sum(neg_mask))

        if n_pos > 5 and n_neg > 5:
            depth_pos = float(np.median(zs_v[pos_mask]))
            depth_neg = float(np.median(zs_v[neg_mask]))
            depth_diff = depth_pos - depth_neg  # 양수 = 양방향 끝이 더 멀리 있음

            # 비대칭 임계값: 2mm
            if abs(depth_diff) > 0.002:
                if depth_diff > 0:
                    # 양방향 끝이 더 멀다 → 반전하여 가까운 쪽을 가리키게
                    yaw = self._normalize_angle(yaw + 180)
                    depth_direction = -1
                else:
                    # 양방향 끝이 이미 가까움 → 유지
                    depth_direction = +1

        return yaw, depth_direction, True

    def _auto_calibrate_depth_offset(self, class_id: int, relative_yaw: float) -> bool:
        """Depth 기반 3D PCA와 기준이미지 2D PCA 간 yaw 오프셋 자동보정.

        기준이미지의 yaw는 2D PCA(_compute_yaw_from_mask)로 계산되지만,
        실시간 측정은 3D PCA(_compute_yaw_with_depth)를 사용하여
        좌표계 차이로 인한 일반적인 각도 오프셋이 발생한다.
        처음 N개 샘플의 중앙값을 기준 yaw에 반영하여 오프셋을 보정.

        Args:
            class_id: 클래스 ID
            relative_yaw: 현재 프레임의 상대 yaw (depth 기준)

        Returns:
            True if 보정이 적용됨
        """
        if class_id not in self._depth_calibration_samples:
            self._depth_calibration_samples[class_id] = []

        self._depth_calibration_samples[class_id].append(relative_yaw)
        samples = self._depth_calibration_samples[class_id]

        if len(samples) < self._DEPTH_CALIBRATION_COUNT:
            return False  # 아직 샘플 부족

        # 중앙값 오프셋 계산: 2D PCA 기준 yaw와 3D PCA 실시간 yaw 간 체계적 차이
        median_offset = float(np.median(samples))
        median_abs = abs(median_offset)

        # 기준 yaw에 오프셋 반영 (이후 relative_yaw ≈ 0° 이 됨)
        if class_id in self.class_reference_yaw:
            old_ref = self.class_reference_yaw[class_id]
            self.class_reference_yaw[class_id] = self._normalize_angle(old_ref + median_offset)

        # 180° 플립 여부도 판단 (오프셋 보정과 별도)
        needs_flip = median_abs > 90.0
        self._depth_ref_offset_180[class_id] = needs_flip
        self._depth_ref_calibrated[class_id] = True

        if class_id in self._depth_calibration_samples:
            del self._depth_calibration_samples[class_id]

        class_name = CLASS_NAMES.get(class_id, f"class_{class_id}")
        print(f"[Signal] 클래스 {class_id} ({class_name}): "
              f"depth 오프셋 보정 완료 (median_offset={median_offset:+.1f}°, "
              f"기준yaw 조정: {old_ref:.1f}° → {self.class_reference_yaw.get(class_id, 0):.1f}°)")

        return needs_flip

    # ── (기존 메서드 계속) ────────────────────────────────────────

    def _compute_yaw_boundaries(self):
        """Compute yaw boundaries between reference signals for signal determination"""
        if not hasattr(self, 'ref_relative_yaws') or len(self.ref_relative_yaws) < 5:
            return

        # Sort references by their relative yaw values
        sorted_refs = sorted(self.ref_relative_yaws.items(), key=lambda x: x[1])
        print(f"[Signal] References sorted by relative yaw:")
        for ref_id, rel_yaw in sorted_refs:
            print(f"    Signal {ref_id:+d}: {rel_yaw:.2f} deg")

        # Calculate and print signal boundaries
        yaw_m2 = self.ref_relative_yaws[-2]
        yaw_m1 = self.ref_relative_yaws[-1]
        yaw_0 = self.ref_relative_yaws[0]
        yaw_1 = self.ref_relative_yaws[1]
        yaw_2 = self.ref_relative_yaws[2]

        b_m2_m1 = (yaw_m2 + yaw_m1) / 2
        b_m1_0 = (yaw_m1 + yaw_0) / 2
        b_0_1 = (yaw_0 + yaw_1) / 2
        b_1_2 = (yaw_1 + yaw_2) / 2

        # 사용자 지정 외부 경계값
        NONE_LOWER = -36.0
        NONE_UPPER = 46.0

        print(f"[Signal] Yaw boundaries for signal determination:")
        print(f"    none:      yaw < {NONE_LOWER:.2f} deg")
        print(f"    Signal -2: {NONE_LOWER:.2f} <= yaw < {b_m2_m1:.2f} deg")
        print(f"    Signal -1: {b_m2_m1:.2f} <= yaw < {b_m1_0:.2f} deg")
        print(f"    Signal  0: {b_m1_0:.2f} <= yaw < {b_0_1:.2f} deg")
        print(f"    Signal +1: {b_0_1:.2f} <= yaw < {b_1_2:.2f} deg")
        print(f"    Signal +2: {b_1_2:.2f} <= yaw < {NONE_UPPER:.2f} deg")
        print(f"    none:      yaw >= {NONE_UPPER:.2f} deg")

        # Create mapping from sorted position to signal boundaries
        # Signal boundaries are midpoints between adjacent references
        self.yaw_boundaries = {}
        for i in range(len(sorted_refs) - 1):
            ref1_id, ref1_yaw = sorted_refs[i]
            ref2_id, ref2_yaw = sorted_refs[i + 1]
            boundary = (ref1_yaw + ref2_yaw) / 2
            self.yaw_boundaries[(ref1_id, ref2_id)] = boundary

        print(f"[Signal] Yaw boundaries between references:")
        for (ref1_id, ref2_id), boundary in self.yaw_boundaries.items():
            print(f"    Between Signal {ref1_id:+d} and {ref2_id:+d}: {boundary:.2f} deg")

        # Store sorted reference order for signal determination
        self.sorted_ref_order = [ref_id for ref_id, _ in sorted_refs]
        self.sorted_ref_yaws = [rel_yaw for _, rel_yaw in sorted_refs]

    def _extract_shape_features(self, image: np.ndarray) -> ShapeFeatures:
        """Extract shape features from image for signal matching"""
        results = self.yolo.predict(source=image, conf=0.25, iou=0.45, verbose=False)

        if not results or not results[0].boxes or len(results[0].boxes) == 0:
            return ShapeFeatures()

        det = results[0]

        # Get mask
        if hasattr(det, 'masks') and det.masks:
            m = det.masks.data[0].cpu().numpy()
            if m.shape != image.shape[:2]:
                mask = cv2.resize(m, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
            else:
                mask = m
            mask = (mask > 0.5).astype(np.uint8) * 255
        else:
            box = det.boxes[0]
            bbox = box.xyxy[0].cpu().numpy().astype(int)
            mask = np.zeros(image.shape[:2], dtype=np.uint8)
            mask[bbox[1]:bbox[3], bbox[0]:bbox[2]] = 255

        return self._compute_shape_features_from_mask(mask)

    def _compute_shape_features_from_mask(self, mask: np.ndarray) -> ShapeFeatures:
        """Compute shape features from a binary mask"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return ShapeFeatures()

        largest = max(contours, key=cv2.contourArea)

        # Bounding box aspect ratio
        x, y, w, h = cv2.boundingRect(largest)
        bbox_aspect = w / (h + 1e-6)

        # Area and perimeter based features
        area = cv2.contourArea(largest)
        perimeter = cv2.arcLength(largest, True)
        circularity = 4 * np.pi * area / (perimeter ** 2 + 1e-6)

        # Convex hull solidity
        hull = cv2.convexHull(largest)
        hull_area = cv2.contourArea(hull)
        solidity = area / (hull_area + 1e-6)

        # Extent (area / bbox area)
        extent = area / (w * h + 1e-6)

        # Hu moments
        moments = cv2.moments(mask)
        hu = cv2.HuMoments(moments).flatten()
        hu = -np.sign(hu) * np.log10(np.abs(hu) + 1e-10)

        return ShapeFeatures(
            bbox_aspect=bbox_aspect,
            circularity=circularity,
            solidity=solidity,
            extent=extent,
            hu_moments=hu
        )

    def _compute_shape_distance(self, f1: ShapeFeatures, f2: ShapeFeatures) -> float:
        """Compute weighted distance between two shape feature sets"""
        d_aspect = abs(f1.bbox_aspect - f2.bbox_aspect) * self.WEIGHT_ASPECT
        d_circ = abs(f1.circularity - f2.circularity) * self.WEIGHT_CIRCULARITY
        d_solid = abs(f1.solidity - f2.solidity) * self.WEIGHT_SOLIDITY
        d_extent = abs(f1.extent - f2.extent) * self.WEIGHT_EXTENT
        return d_aspect + d_circ + d_solid + d_extent

    def _set_reference_pose(self):
        """Set reference pose from specific image file"""
        ref_path = self.reference_dir / self.reference_image_file
        if not ref_path.exists():
            print(f"[Signal] Warning: Reference image not found: {ref_path}")
            return

        print(f"[Signal] Setting reference pose from: {self.reference_image_file}")

        image = cv2.imread(str(ref_path))
        if image is None:
            return

        mask_features, yaw_angle = self._extract_features(image)
        self.reference_yaw = yaw_angle

        results = self.yolo.predict(source=image, conf=0.25, iou=0.45, verbose=False)
        if results and results[0].boxes and len(results[0].boxes) > 0:
            det = results[0]
            box = det.boxes[0]
            bbox = box.xyxy[0].cpu().numpy().astype(int)

            if hasattr(det, 'masks') and det.masks:
                m = det.masks.data[0].cpu().numpy()
                if m.shape != image.shape[:2]:
                    mask = cv2.resize(m, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
                else:
                    mask = m
                mask = (mask > 0.5).astype(np.uint8) * 255
            else:
                mask = np.zeros(image.shape[:2], dtype=np.uint8)
                mask[bbox[1]:bbox[3], bbox[0]:bbox[2]] = 255

            intrinsics = {'fx': 383.883, 'fy': 383.883, 'cx': 320.499, 'cy': 237.913}
            valid_indices = np.where(mask > 0)
            if len(valid_indices[0]) > 0:
                cy = np.mean(valid_indices[0])
                cx = np.mean(valid_indices[1])
                z = 1.0
                x = (cx - intrinsics['cx']) * z / intrinsics['fx']
                y = (cy - intrinsics['cy']) * z / intrinsics['fy']
                position = np.array([x, y, z])
            else:
                position = np.array([0.0, 0.0, 1.0])

            rotation = self._estimate_rotation_from_mask(mask)

            self.reference_pose = ReferencePose(
                position=position,
                rotation=rotation,
                mask_features=mask_features,
                yaw_angle=yaw_angle,
                image_path=str(ref_path)
            )

            print(f"  [Signal] Reference Yaw (Base): {yaw_angle:.1f} deg")

    def _extract_features(self, image: np.ndarray) -> Tuple[np.ndarray, float]:
        """Extract mask features and yaw angle from image"""
        results = self.yolo.predict(source=image, conf=0.25, iou=0.45, verbose=False)

        if not results or not results[0].boxes or len(results[0].boxes) == 0:
            return np.zeros(128), 0.0

        det = results[0]

        mask = None
        if hasattr(det, 'masks') and det.masks:
            m = det.masks.data[0].cpu().numpy()
            if m.shape != image.shape[:2]:
                mask = cv2.resize(m, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST)
            else:
                mask = m
            mask = (mask > 0.5).astype(np.uint8) * 255
        else:
            box = det.boxes[0]
            bbox = box.xyxy[0].cpu().numpy().astype(int)
            mask = np.zeros(image.shape[:2], dtype=np.uint8)
            mask[bbox[1]:bbox[3], bbox[0]:bbox[2]] = 255

        yaw_angle = self._compute_yaw_from_mask(mask)
        features = self._compute_mask_features(mask)

        return features, yaw_angle

    def _estimate_rotation_from_mask(self, mask: np.ndarray) -> Dict[str, float]:
        """Estimate rotation from mask shape using PCA"""
        pts = np.column_stack(np.where(mask > 0)).astype(np.float32)
        if len(pts) < 20:
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        try:
            mean = np.mean(pts, axis=0)
            centered = pts - mean
            cov = np.cov(centered.T)
            eigenvalues, eigenvectors = np.linalg.eigh(cov)

            idx = np.argsort(eigenvalues)[::-1]
            eigenvectors = eigenvectors[:, idx]
            eigenvalues = eigenvalues[idx]

            primary_axis = eigenvectors[:, 0]
            yaw = np.degrees(np.arctan2(primary_axis[1], primary_axis[0]))

            aspect_ratio = eigenvalues[0] / (eigenvalues[1] + 1e-6)
            pitch = np.clip((aspect_ratio - 1.5) * 10, -45, 45)

            return {'roll': 0.0, 'pitch': pitch, 'yaw': yaw}
        except:
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

    def _compute_yaw_from_mask(self, mask: np.ndarray) -> float:
        """Compute yaw angle from mask shape using PCA"""
        pts = np.column_stack(np.where(mask > 0)).astype(np.float32)
        if len(pts) < 20:
            return 0.0

        try:
            mean = np.mean(pts, axis=0)
            centered = pts - mean
            cov = np.cov(centered.T)
            eigenvalues, eigenvectors = np.linalg.eigh(cov)

            idx = np.argsort(eigenvalues)[::-1]
            eigenvectors = eigenvectors[:, idx]

            primary_axis = eigenvectors[:, 0]
            yaw = np.degrees(np.arctan2(primary_axis[1], primary_axis[0]))

            return yaw
        except:
            return 0.0

    def _compute_mask_features(self, mask: np.ndarray) -> np.ndarray:
        """Compute feature vector from mask"""
        features = []

        moments = cv2.moments(mask)
        hu_moments = cv2.HuMoments(moments).flatten()
        hu_moments = -np.sign(hu_moments) * np.log10(np.abs(hu_moments) + 1e-10)
        features.extend(hu_moments[:7])

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            perimeter = cv2.arcLength(largest, True)

            circularity = 4 * np.pi * area / (perimeter ** 2 + 1e-6)
            features.append(circularity)

            x, y, w, h = cv2.boundingRect(largest)
            aspect_ratio = w / (h + 1e-6)
            features.append(aspect_ratio)

            hull = cv2.convexHull(largest)
            hull_area = cv2.contourArea(hull)
            solidity = area / (hull_area + 1e-6)
            features.append(solidity)

            extent = area / (w * h + 1e-6)
            features.append(extent)
        else:
            features.extend([0, 1, 0, 0])

        features = np.array(features)
        if len(features) < 128:
            features = np.pad(features, (0, 128 - len(features)))

        return features[:128]

    def _compute_angle_boundaries(self):
        """Compute angle boundaries between reference images"""
        if len(self.references) < 2:
            return

        sorted_refs = sorted(self.references.items(), key=lambda x: x[0])

        for i, (angle_id, ref) in enumerate(sorted_refs):
            if i == 0:
                if i + 1 < len(sorted_refs):
                    next_ref = sorted_refs[i + 1][1]
                    diff = abs(next_ref.yaw_angle - ref.yaw_angle)
                    lower = ref.yaw_angle - diff
                else:
                    lower = ref.yaw_angle - 30
                upper = ref.yaw_angle
            elif i == len(sorted_refs) - 1:
                prev_ref = sorted_refs[i - 1][1]
                diff = abs(ref.yaw_angle - prev_ref.yaw_angle)
                lower = ref.yaw_angle
                upper = ref.yaw_angle + diff
            else:
                prev_ref = sorted_refs[i - 1][1]
                next_ref = sorted_refs[i + 1][1]
                lower = (prev_ref.yaw_angle + ref.yaw_angle) / 2
                upper = (ref.yaw_angle + next_ref.yaw_angle) / 2

            self.angle_boundaries[angle_id] = (lower, upper)

    def get_measurement_mode(self, class_id: int) -> str:
        """클래스별 측정 모드 반환"""
        return self.class_measurement_mode.get(class_id, "signal")

    def get_dual_angle(self, mask: np.ndarray, class_id: int,
                       depth: np.ndarray = None) -> Tuple[float, float, float]:
        """
        dual_angle 모드: 0도 기준 상대각도 + 180도 기준 상대각도 동시 측정

        Args:
            mask: 이진 마스크
            class_id: 클래스 ID
            depth: 깊이 이미지 (선택, 있으면 depth 강화 PCA 사용)

        Returns:
            current_yaw: 현재 PCA yaw 값
            relative_yaw_0deg: 0도 기준 상대각도
            relative_yaw_180deg: 180도 기준 상대각도
        """
        baselines = self.class_baseline_yaws.get(class_id, {})
        if not baselines:
            return 0.0, 0.0, 0.0

        # depth가 있으면 3D PCA 사용 (마스크 정제 + 원근 보정)
        if depth is not None and self._intrinsics is not None:
            current_yaw, _, _ = self._compute_yaw_with_depth(mask, depth)
        else:
            current_yaw = self._compute_yaw_from_mask(mask)

        yaw_0 = baselines.get(0, 0.0)
        yaw_180 = baselines.get(180, 0.0)

        rel_0 = self._normalize_angle(current_yaw - yaw_0)
        rel_180 = self._normalize_angle(current_yaw - yaw_180)

        return current_yaw, rel_0, rel_180

    def get_relative_yaw_only(self, mask: np.ndarray, class_id: int = -1,
                              depth: np.ndarray = None) -> float:
        """
        기준(Signal 0 또는 0도 기준이미지) 대비 상대 yaw 각도만 계산 (시그널 분류 없음)

        Wiring_tray 등 시그널 분류가 불필요한 클래스에서 사용.
        기존 signal 모드의 signal=0 레퍼런스를 0도 기준으로 활용.

        depth가 제공되면:
          1) depth 기반 마스크 정제 (배경 노이즈 제거)
          2) 3D 메트릭 공간 PCA (원근 왜곡 보정)
          3) depth gradient 180° 모호성 해결

        Args:
            mask: 이진 마스크
            class_id: 클래스 ID
            depth: 깊이 이미지 (선택, 있으면 depth 강화 PCA 사용)

        Returns:
            relative_yaw: 기준 대비 상대 yaw 각도 (도)
        """
        # 클래스별 기준 yaw 사용 (signal 모드의 signal=0 레퍼런스 yaw)
        if class_id >= 0 and class_id in self.class_reference_yaw:
            ref_yaw = self.class_reference_yaw[class_id]
        elif hasattr(self, 'reference_yaw') and self.reference_yaw != 0.0:
            ref_yaw = self.reference_yaw
        else:
            return 0.0

        # 기준이미지와 동일한 방식으로 측정 (depth 있으면 3D PCA, 없으면 2D PCA)
        if depth is not None and self._intrinsics is not None:
            current_yaw, _, _ = self._compute_yaw_with_depth(mask, depth)
        else:
            current_yaw = self._compute_yaw_from_mask(mask)

        relative_yaw = self._normalize_angle(current_yaw - ref_yaw)

        # PCA 180° 모호성 해결: 기준 yaw 기반
        # PCA 주축은 방향이 임의적이라 ±180° 뒤집힐 수 있음
        # 기준 대비 |relative_yaw| > 90° 이면 PCA 뒤집힘으로 판단하여 보정
        if abs(relative_yaw) > 90.0:
            relative_yaw = self._normalize_angle(relative_yaw + 180.0)

        # housing_M 캘리브레이션 오프셋 보정
        if class_id == 0:
            relative_yaw += 9.0

        return relative_yaw

    def get_signal(self, mask: np.ndarray, class_id: int = -1,
                   depth: np.ndarray = None) -> Tuple[str, float, float]:
        """
        Get signal based on YAW ANGLE comparison with references.

        Uses PCA-based yaw angle for signal matching.
        class_id가 지정되면 해당 클래스의 참조 세트를 사용.
        dual_angle 모드일 경우 시그널 없이 0도 기준 상대각도만 반환.

        Args:
            mask: 이진 마스크
            class_id: 클래스 ID
            depth: 깊이 이미지 (선택, 있으면 depth 강화 PCA 사용)

        Returns:
            signal: "-2", "-1", "0", "1", "2", or "none" (dual_angle이면 항상 "raw")
            confidence: based on distance to closest reference (higher is better)
            relative_yaw: yaw angle relative to reference 0
        """
        # dual_angle 모드 체크
        mode = self.class_measurement_mode.get(class_id, "signal")
        if mode == "dual_angle":
            current_yaw, rel_0, rel_180 = self.get_dual_angle(mask, class_id, depth=depth)
            return "raw", 1.0, rel_0

        # signal 모드 (기존 방식)
        if class_id >= 0 and class_id in self.class_references:
            refs = self.class_references[class_id]
            ref_yaw = self.class_reference_yaw.get(class_id, 0.0)
            ref_rel_yaws = self.class_ref_relative_yaws.get(class_id, {})
            none_bounds = self.class_none_bounds.get(class_id, (-36.0, 46.0))
        else:
            refs = self.references
            ref_yaw = self.reference_yaw
            ref_rel_yaws = self.ref_relative_yaws
            none_bounds = (-36.0, 46.0)

        if len(refs) == 0:
            return "none", 0.0, 0.0

        # Compute current yaw angle (depth 강화 or 기존 2D PCA)
        if depth is not None and self._intrinsics is not None:
            current_yaw, _, _ = self._compute_yaw_with_depth(mask, depth)
        else:
            current_yaw = self._compute_yaw_from_mask(mask)
        relative_yaw = self._normalize_angle(current_yaw - ref_yaw)

        # Calculate yaw distances to all references
        yaw_distances = {}
        for angle_id, rel_y in ref_rel_yaws.items():
            diff = abs(self._normalize_angle(relative_yaw - rel_y))
            yaw_distances[angle_id] = diff

        if not yaw_distances:
            return "none", 0.0, relative_yaw

        # Find minimum distance
        min_distance = min(yaw_distances.values())

        # Convert distance to confidence (0-1 range, higher is better)
        confidence = max(0.0, 1.0 - min_distance / 45.0)

        # Determine signal based on yaw position in sorted reference order
        signal = self._determine_signal_by_yaw(relative_yaw, ref_rel_yaws, none_bounds)

        return signal, confidence, relative_yaw

    def _determine_signal_by_yaw(self, current_relative_yaw: float,
                                ref_rel_yaws: Optional[Dict[int, float]] = None,
                                none_bounds: Optional[Tuple[float, float]] = None) -> str:
        """
        Yaw 각도를 기반으로 시그널 범위 결정

        ref_rel_yaws: 클래스별 상대 yaw 값 (없으면 self.ref_relative_yaws 사용)
        none_bounds: (lower, upper) 외부 경계 (없으면 기본값 사용)
        """
        rel_yaws = ref_rel_yaws if ref_rel_yaws is not None else self.ref_relative_yaws
        if not rel_yaws:
            return "0"

        # 레퍼런스 yaw 값들 가져오기
        yaw_m2 = rel_yaws.get(-2, -30.0)
        yaw_m1 = rel_yaws.get(-1, -20.0)
        yaw_0 = rel_yaws.get(0, 0.0)
        yaw_1 = rel_yaws.get(1, 20.0)
        yaw_2 = rel_yaws.get(2, 40.0)

        # 경계값 계산 (두 레퍼런스 사이의 중간점)
        boundary_m2_m1 = (yaw_m2 + yaw_m1) / 2
        boundary_m1_0 = (yaw_m1 + yaw_0) / 2
        boundary_0_1 = (yaw_0 + yaw_1) / 2
        boundary_1_2 = (yaw_1 + yaw_2) / 2

        # 외부 경계값
        NONE_LOWER_BOUND = none_bounds[0] if none_bounds else -36.0
        NONE_UPPER_BOUND = none_bounds[1] if none_bounds else 46.0

        # Yaw 기반 시그널 결정
        if current_relative_yaw < NONE_LOWER_BOUND:
            return "none"
        elif current_relative_yaw < boundary_m2_m1:
            return "-2"
        elif current_relative_yaw < boundary_m1_0:
            return "-1"
        elif current_relative_yaw < boundary_0_1:
            return "0"
        elif current_relative_yaw < boundary_1_2:
            return "1"
        elif current_relative_yaw < NONE_UPPER_BOUND:
            return "2"
        else:
            return "none"

    def _match_by_yaw(self, yaw: float) -> Optional[int]:
        """Match yaw angle to closest reference"""
        if not self.references:
            return None

        best_match = None
        min_diff = float('inf')

        for angle_id, ref in self.references.items():
            diff = abs(yaw - ref.yaw_angle)
            if diff > 180:
                diff = 360 - diff

            if diff < min_diff:
                min_diff = diff
                best_match = angle_id

        return best_match

    def _match_by_relative_yaw(self, relative_yaw: float) -> Optional[int]:
        """Match relative yaw angle to closest reference signal using relative yaw values"""
        if not self.references or 0 not in self.references:
            return None

        ref_yaw_0 = self.references[0].yaw_angle
        best_match = None
        min_diff = float('inf')

        for angle_id, ref in self.references.items():
            # Calculate reference's relative yaw
            ref_relative_yaw = ref.yaw_angle - ref_yaw_0
            if ref_relative_yaw > 180:
                ref_relative_yaw -= 360
            elif ref_relative_yaw < -180:
                ref_relative_yaw += 360

            # Compare with current relative yaw
            diff = abs(relative_yaw - ref_relative_yaw)
            if diff > 180:
                diff = 360 - diff

            if diff < min_diff:
                min_diff = diff
                best_match = angle_id

        return best_match

    def _match_by_features(self, features: np.ndarray) -> Tuple[int, float]:
        """Match features to closest reference"""
        if not self.references:
            return 0, 0.0

        best_match = 0
        max_similarity = 0.0

        for angle_id, ref in self.references.items():
            dot = np.dot(features, ref.mask_features)
            norm = np.linalg.norm(features) * np.linalg.norm(ref.mask_features)
            similarity = dot / (norm + 1e-6)

            if similarity > max_similarity:
                max_similarity = similarity
                best_match = angle_id

        return best_match, max_similarity

    def compute_relative_pose(self, position: np.ndarray, rotation: Dict[str, float]) -> Tuple[np.ndarray, Dict[str, float], float]:
        """Compute pose relative to reference pose"""
        if self.reference_pose is None:
            return np.zeros(3), {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}, 0.0

        rel_position = position - self.reference_pose.position

        rel_yaw = rotation['yaw'] - self.reference_pose.rotation['yaw']
        if rel_yaw > 180:
            rel_yaw -= 360
        elif rel_yaw < -180:
            rel_yaw += 360

        rel_rotation = {
            'roll': rotation['roll'] - self.reference_pose.rotation['roll'],
            'pitch': rotation['pitch'] - self.reference_pose.rotation['pitch'],
            'yaw': rel_yaw
        }

        return rel_position, rel_rotation, rel_yaw


class RealSenseCamera:
    """Intel RealSense D455 Camera Handler"""

    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = None
        self.config = None
        self.align = None
        self.depth_scale = None
        self.intrinsics = None
        self.is_running = False

    def start(self):
        """Start the camera pipeline with automatic profile detection"""
        print("[Camera] Initializing Intel RealSense D455...")

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        configs_to_try = [
            (self.width, self.height, self.width, self.height, self.fps, "Requested"),
            (848, 480, 848, 480, 30, "848x480 @ 30fps"),
            (640, 480, 640, 480, 30, "640x480 @ 30fps"),
            (1280, 720, 1280, 720, 30, "1280x720 @ 30fps"),
            (640, 480, 640, 480, 15, "640x480 @ 15fps"),
        ]

        for color_w, color_h, depth_w, depth_h, fps, desc in configs_to_try:
            try:
                self.pipeline = rs.pipeline()
                self.config = rs.config()

                print(f"[Camera] Trying {desc}: Color {color_w}x{color_h}, Depth {depth_w}x{depth_h} @ {fps}fps")

                self.config.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, fps)
                self.config.enable_stream(rs.stream.depth, depth_w, depth_h, rs.format.z16, fps)

                profile = self.pipeline.start(self.config)

                self.width = color_w
                self.height = color_h
                self.fps = fps

                depth_sensor = profile.get_device().first_depth_sensor()
                self.depth_scale = depth_sensor.get_depth_scale()
                print(f"[Camera] Depth Scale: {self.depth_scale}")

                try:
                    depth_sensor.set_option(rs.option.visual_preset, 3)
                except:
                    pass

                color_profile = profile.get_stream(rs.stream.color)
                intr = color_profile.as_video_stream_profile().get_intrinsics()
                self.intrinsics = {
                    'fx': intr.fx, 'fy': intr.fy,
                    'cx': intr.ppx, 'cy': intr.ppy,
                    'width': intr.width, 'height': intr.height
                }
                print(f"[Camera] Intrinsics: fx={intr.fx:.2f}, fy={intr.fy:.2f}")

                self.align = rs.align(rs.stream.color)
                self.is_running = True
                print(f"[Camera] Started successfully: {self.width}x{self.height} @ {self.fps}fps")
                return True

            except Exception as e:
                print(f"[Camera] Config failed: {e}")
                try:
                    self.pipeline.stop()
                except:
                    pass
                continue

        print("[Camera] All configurations failed!")
        return False

    def read(self):
        """Read a frame from the camera"""
        if not self.is_running:
            return None, None

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            aligned_frames = self.align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return None, None

            rgb = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            depth = depth.astype(np.float32) * self.depth_scale

            return rgb, depth

        except Exception as e:
            print(f"[Camera] Read error: {e}")
            return None, None

    def stop(self):
        """Stop the camera pipeline"""
        if self.is_running:
            self.pipeline.stop()
            self.is_running = False
            print("[Camera] Stopped")

    def get_intrinsics(self):
        """Get camera intrinsics"""
        return self.intrinsics if self.intrinsics else {
            'fx': 383.883, 'fy': 383.883, 'cx': 320.499, 'cy': 237.913
        }


class GPUImageProcessor:
    """GPU-accelerated image processing"""

    def __init__(self, use_gpu=True):
        self.use_gpu = use_gpu and HAS_CUPY

    def extract_center_and_depth(self, depth, mask, intrinsics):
        """Extract 3D center position from depth and mask.

        depth 유효 픽셀이 부족하면 마스크 중심 기반 fallback으로 처리.
        """
        if self.use_gpu:
            try:
                depth_gpu = cp.asarray(depth)
                mask_gpu = cp.asarray(mask)

                mask_indices = cp.where(mask_gpu > 0)
                if len(mask_indices[0]) < 10:
                    return None

                valid_mask = (mask_gpu > 0) & (depth_gpu > 0.1) & (depth_gpu < 10.0)
                valid_indices = cp.where(valid_mask)

                if len(valid_indices[0]) >= 10:
                    cy = float(cp.mean(valid_indices[0]))
                    cx = float(cp.mean(valid_indices[1]))
                    z = float(cp.median(depth_gpu[valid_mask]))
                else:
                    cy = float(cp.mean(mask_indices[0]))
                    cx = float(cp.mean(mask_indices[1]))
                    nonzero = depth_gpu[(mask_gpu > 0) & (depth_gpu > 0)]
                    z = float(cp.median(nonzero)) if len(nonzero) > 0 else 1.0

                x = (cx - intrinsics['cx']) * z / intrinsics['fx']
                y = (cy - intrinsics['cy']) * z / intrinsics['fy']

                return np.array([x, y, z]), (int(cx), int(cy))
            except:
                pass

        # CPU fallback
        mask_indices = np.where(mask > 0)
        if len(mask_indices[0]) < 10:
            return None

        valid_mask = (mask > 0) & (depth > 0.1) & (depth < 10.0)
        valid_indices = np.where(valid_mask)

        if len(valid_indices[0]) >= 10:
            cy = np.mean(valid_indices[0])
            cx = np.mean(valid_indices[1])
            z = np.median(depth[valid_mask])
        else:
            cy = np.mean(mask_indices[0])
            cx = np.mean(mask_indices[1])
            nonzero = depth[(mask > 0) & (depth > 0)]
            z = float(np.median(nonzero)) if len(nonzero) > 0 else 1.0

        x = (cx - intrinsics['cx']) * z / intrinsics['fx']
        y = (cy - intrinsics['cy']) * z / intrinsics['fy']

        return np.array([x, y, z]), (int(cx), int(cy))


class TemporalFilter:
    """Temporal filter to reject spikes"""

    def __init__(self, window=30, threshold=3.0):
        self.window = window
        self.threshold = threshold
        self.values = deque(maxlen=window)

    def add(self, v):
        self.values.append(v)
        if len(self.values) < 5:
            return v
        arr = np.array(self.values)
        mean, std = np.mean(arr), np.std(arr)
        if std > 0 and abs(v - mean) > self.threshold * std:
            return np.median(arr)
        return v

    def is_stable(self):
        if len(self.values) < self.window:
            return False
        return np.std(self.values) < 0.1 * (abs(np.mean(self.values)) + 1e-6)


class PoseTracker:
    """6DOF Pose tracker"""

    def __init__(self, intrinsics, use_gpu=True):
        self.intrinsics = intrinsics
        self.gpu_processor = GPUImageProcessor(use_gpu)

    def extract_pose(self, depth, mask):
        """Extract 6DOF pose from depth and mask"""
        result = self.gpu_processor.extract_center_and_depth(depth, mask, self.intrinsics)
        if result is None:
            return None, None, None

        position, center_2d = result
        rotation = self._estimate_rotation_from_mask(mask, center_2d)

        return position, rotation, center_2d

    def _estimate_rotation_from_mask(self, mask, center_2d):
        """Estimate rotation from mask shape using PCA"""
        pts = np.column_stack(np.where(mask > 0)).astype(np.float32)
        if len(pts) < 20:
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        try:
            mean = np.mean(pts, axis=0)
            centered = pts - mean
            cov = np.cov(centered.T)
            eigenvalues, eigenvectors = np.linalg.eigh(cov)

            idx = np.argsort(eigenvalues)[::-1]
            eigenvectors = eigenvectors[:, idx]
            eigenvalues = eigenvalues[idx]

            primary_axis = eigenvectors[:, 0]
            yaw = np.degrees(np.arctan2(primary_axis[1], primary_axis[0]))

            aspect_ratio = eigenvalues[0] / (eigenvalues[1] + 1e-6)
            pitch = np.clip((aspect_ratio - 1.5) * 10, -45, 45)

            return {'roll': 0.0, 'pitch': pitch, 'yaw': yaw}
        except:
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}


class PersistentMultiObjectTracker:
    """
    Multi-object tracker with persistent ID assignment
    Objects are ordered right-to-left (rightmost gets lowest ID)
    From test_result_22
    """

    def __init__(self, fps: float = 30.0, intrinsics: Dict = None):
        self.fps = fps
        self.trackers: Dict[int, TrackedObject] = OrderedDict()
        self.next_object_id = 0
        self.intrinsics = intrinsics or {'fx': 383.883, 'fy': 383.883, 'cx': 320.499, 'cy': 237.913}
        self.frame_count = 0
        self.gpu_processor = GPUImageProcessor(use_gpu=HAS_CUPY)
        # Consensus speed: Speed ROI 내 전체 객체 속도의 중앙값 → 프레임 레벨 스무딩
        self.consensus_speed_filter = TemporalFilter(window=60, threshold=3.0)
        self.consensus_speed_m_min = 0.0

    def _compute_iou(self, box1: List[int], box2: List[int]) -> float:
        """Compute IoU between two bounding boxes"""
        x1_1, y1_1, x2_1, y2_1 = box1
        x1_2, y1_2, x2_2, y2_2 = box2

        xi1 = max(x1_1, x1_2)
        yi1 = max(y1_1, y1_2)
        xi2 = min(x2_1, x2_2)
        yi2 = min(y2_1, y2_2)

        if xi2 <= xi1 or yi2 <= yi1:
            return 0.0

        inter_area = (xi2 - xi1) * (yi2 - yi1)
        box1_area = (x2_1 - x1_1) * (y2_1 - y1_1)
        box2_area = (x2_2 - x1_2) * (y2_2 - y1_2)
        union_area = box1_area + box2_area - inter_area

        return inter_area / (union_area + 1e-6)

    def _create_tracker(self, frame_idx: int, center_x: float) -> TrackedObject:
        """Create new object tracker"""
        obj_id = self.next_object_id
        self.next_object_id += 1

        return TrackedObject(
            object_id=obj_id,
            kalman=PoseKalmanFilter(dt=1.0/self.fps, mode=FilterMode.QUATERNION),
            speed_filter=TemporalFilter(45, 3.0),
            angle_filter=TemporalFilter(45, 3.0),
            last_center_x=center_x,
            first_detection_frame=frame_idx,
            last_seen_frame=frame_idx
        )

    def _match_detections_to_trackers(self, detections: List[Dict]) -> Tuple[Dict[int, int], List[int]]:
        """
        Match detections to existing trackers using IoU
        Returns: (matched: {det_idx: tracker_id}, unmatched_det_indices)
        """
        if not self.trackers or not detections:
            return {}, list(range(len(detections)))

        # Build cost matrix based on IoU
        active_trackers = [(tid, t) for tid, t in self.trackers.items() if t.is_active]

        if not active_trackers:
            return {}, list(range(len(detections)))

        iou_matrix = np.zeros((len(detections), len(active_trackers)))

        for i, det in enumerate(detections):
            for j, (tid, tracker) in enumerate(active_trackers):
                if tracker.last_bbox:
                    iou_matrix[i, j] = self._compute_iou(det['bbox'], tracker.last_bbox)

        # Greedy matching
        matched = {}
        matched_trackers = set()

        while True:
            if iou_matrix.size == 0:
                break

            max_iou = np.max(iou_matrix)
            if max_iou < IOU_THRESHOLD:
                break

            max_idx = np.unravel_index(np.argmax(iou_matrix), iou_matrix.shape)
            det_idx, tracker_idx = max_idx

            tracker_id = active_trackers[tracker_idx][0]
            matched[det_idx] = tracker_id
            matched_trackers.add(tracker_idx)

            # Remove matched entries
            iou_matrix[det_idx, :] = -1
            iou_matrix[:, tracker_idx] = -1

        unmatched = [i for i in range(len(detections)) if i not in matched]
        return matched, unmatched

    def extract_pose(self, depth: np.ndarray, mask: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[Dict], Optional[Tuple]]:
        """Extract 6DOF pose from depth and mask.

        depth 유효 픽셀이 부족하면 마스크 중심 기반 fallback으로 처리.
        (비디오 depth 인코딩에서 값 범위가 맞지 않는 경우 대비)
        """
        mask_indices = np.where(mask > 0)
        if len(mask_indices[0]) < 10:
            return None, None, None

        # depth 유효 픽셀 확인
        valid_mask = (mask > 0) & (depth > 0.1) & (depth < 10.0)
        valid_indices = np.where(valid_mask)

        if len(valid_indices[0]) >= 10:
            # 정상: depth 기반 위치 추정
            cy = np.mean(valid_indices[0])
            cx = np.mean(valid_indices[1])
            z = np.median(depth[valid_mask])
        else:
            # Fallback: 마스크 중심 사용, depth는 기본값 1.0m
            cy = np.mean(mask_indices[0])
            cx = np.mean(mask_indices[1])
            # 마스크 영역 내 0이 아닌 depth 값이 있으면 사용
            nonzero_depth = depth[(mask > 0) & (depth > 0)]
            z = float(np.median(nonzero_depth)) if len(nonzero_depth) > 0 else 1.0

        center_2d = (int(cx), int(cy))

        x = (cx - self.intrinsics['cx']) * z / self.intrinsics['fx']
        y = (cy - self.intrinsics['cy']) * z / self.intrinsics['fy']
        position = np.array([x, y, z])

        euler_angles = self._estimate_rotation_from_mask(mask)

        return position, euler_angles, center_2d

    def _estimate_rotation_from_mask(self, mask: np.ndarray) -> Dict[str, float]:
        """Estimate rotation from mask shape using PCA"""
        pts = np.column_stack(np.where(mask > 0)).astype(np.float32)
        if len(pts) < 20:
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        try:
            mean = np.mean(pts, axis=0)
            centered = pts - mean
            cov = np.cov(centered.T)
            eigenvalues, eigenvectors = np.linalg.eigh(cov)

            idx = np.argsort(eigenvalues)[::-1]
            eigenvectors = eigenvectors[:, idx]
            eigenvalues = eigenvalues[idx]

            primary_axis = eigenvectors[:, 0]
            yaw = np.degrees(np.arctan2(primary_axis[1], primary_axis[0]))

            aspect_ratio = eigenvalues[0] / (eigenvalues[1] + 1e-6)
            pitch = np.clip((aspect_ratio - 1.5) * 10, -45, 45)

            return {'roll': 0.0, 'pitch': pitch, 'yaw': yaw}
        except:
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

    def process_detections(
        self,
        detections: List[Dict],
        depth: np.ndarray,
        frame_idx: int,
        signal_gen: 'ShapeBasedSignalGenerator'
    ) -> List[DetectedObject]:
        """Process multiple detections with persistent tracking"""
        self.frame_count = frame_idx
        processed_objects = []

        # Sort detections by center_x (right to left - descending)
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            det['center_x'] = (x1 + x2) / 2

        sorted_dets = sorted(detections, key=lambda x: x['center_x'], reverse=True)

        # Match detections to existing trackers
        matched, unmatched = self._match_detections_to_trackers(sorted_dets)

        # Update matched trackers
        for det_idx, tracker_id in matched.items():
            det = sorted_dets[det_idx]
            tracker = self.trackers[tracker_id]

            obj = self._process_single_detection(det, depth, tracker, frame_idx, signal_gen)
            if obj is not None:
                processed_objects.append(obj)

        # Create new trackers for unmatched detections
        for det_idx in sorted(unmatched, key=lambda i: sorted_dets[i]['center_x'], reverse=True):
            det = sorted_dets[det_idx]
            center_x = det['center_x']

            tracker = self._create_tracker(frame_idx, center_x)
            self.trackers[tracker.object_id] = tracker

            obj = self._process_single_detection(det, depth, tracker, frame_idx, signal_gen)
            if obj is not None:
                processed_objects.append(obj)

        # Update lost frames for trackers not seen this frame
        seen_tracker_ids = set(matched.values())
        for tracker_id, tracker in self.trackers.items():
            if tracker_id not in seen_tracker_ids and tracker.is_active:
                tracker.lost_frames += 1
                if tracker.lost_frames > MAX_LOST_FRAMES:
                    tracker.is_active = False

        processed_objects.sort(key=lambda x: x.object_id)

        # Consensus speed: Speed ROI 내 모든 객체 속도의 중앙값 → 프레임 스무딩
        speed_roi_speeds = [obj.speed_m_min for obj in processed_objects
                           if obj.in_speed_roi and obj.speed_m_min > 0.01]
        if speed_roi_speeds:
            median_speed = float(np.median(speed_roi_speeds))
            self.consensus_speed_m_min = self.consensus_speed_filter.add(median_speed)
            # 각 객체의 speed_m_min을 consensus로 대체
            for obj in processed_objects:
                if obj.in_speed_roi:
                    obj.speed_m_min = self.consensus_speed_m_min

        return processed_objects

    def _process_single_detection(
        self,
        det: Dict,
        depth: np.ndarray,
        tracker: TrackedObject,
        frame_idx: int,
        signal_gen: 'ShapeBasedSignalGenerator'
    ) -> Optional[DetectedObject]:
        """Process a single detection with its tracker"""
        mask = det['mask']

        position, rotation, center_2d = self.extract_pose(depth, mask)
        if position is None:
            return None

        tracker.detection_count += 1
        tracker.last_bbox = det['bbox']
        tracker.last_center_x = det['center_x']
        tracker.last_seen_frame = frame_idx
        tracker.lost_frames = 0

        yaw_rad = np.radians(rotation['yaw'])
        quat = Quaternion(x=0, y=0, z=np.sin(yaw_rad/2), w=np.cos(yaw_rad/2))

        if tracker.detection_count == 1:
            tracker.kalman.initialize(position, quat)
            state = tracker.kalman.get_state()
        else:
            state = tracker.kalman.predict_and_update(position, quat)

        filt_speed = tracker.speed_filter.add(state.speed)
        filt_yaw = tracker.angle_filter.add(state.orientation_euler.yaw)
        rotation['yaw'] = filt_yaw

        tracker.last_position = state.position
        tracker.last_rotation = rotation

        # Dual ROI 확인
        in_angle_roi = det.get('in_angle_roi', False)
        in_speed_roi = det.get('in_speed_roi', False)

        # Angle ROI에 있을 때만 signal 계산
        det_class_id = det.get('class_id', -1)
        measurement_mode = signal_gen.get_measurement_mode(det_class_id)
        rel_yaw_0deg = 0.0
        rel_yaw_180deg = 0.0

        if in_angle_roi:
            # 통합 인터페이스: 모든 클래스 0도 기준 상대각도만 측정
            # [비활성화] housing_M 180도 동시 측정:
            # if measurement_mode == "dual_angle":
            #     current_yaw, rel_yaw_0deg, rel_yaw_180deg = signal_gen.get_dual_angle(mask, det_class_id)
            # [비활성화] Wiring_tray 시그널 분류(-2~+2):
            # signal, signal_conf, relative_yaw = signal_gen.get_signal(mask, class_id=det_class_id)
            relative_yaw = signal_gen.get_relative_yaw_only(mask, class_id=det_class_id, depth=depth)
            rel_yaw_0deg = relative_yaw
            rel_yaw_180deg = 0.0
            signal, signal_conf = "raw", 1.0
            measurement_mode = "dual_angle"
            rel_position, rel_rotation, rel_yaw = signal_gen.compute_relative_pose(state.position, rotation)
        else:
            signal, signal_conf = "none", 0.0
            rel_position = np.zeros(3)
            rel_rotation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
            rel_yaw = 0.0

        # 속도: m/s -> m/min 변환
        speed_m_min = filt_speed * 60.0

        measurement = {
            'frame': frame_idx,
            'position': state.position.tolist(),
            'rotation': rotation.copy(),
            'velocity': state.velocity.tolist(),
            'speed': filt_speed,
            'speed_m_min': speed_m_min,
            'signal': signal,
            'relative_yaw': rel_yaw,
            'relative_yaw_0deg': rel_yaw_0deg,
            'relative_yaw_180deg': rel_yaw_180deg,
            'measurement_mode': measurement_mode,
            'in_angle_roi': in_angle_roi,
            'in_speed_roi': in_speed_roi
        }
        tracker.total_measurements.append(measurement)

        detected_obj = DetectedObject(
            object_id=tracker.object_id,
            bbox=det['bbox'],
            confidence=det['confidence'],
            mask=mask,
            class_id=det.get('class_id', 0),
            class_name=det.get('class_name', CLASS_NAMES.get(det.get('class_id', 0), '')),
            center_x=det['center_x'],
            center_2d=center_2d,
            position=state.position,
            rotation=rotation,
            velocity=state.velocity,
            speed=filt_speed,
            speed_m_min=speed_m_min,
            signal=signal,
            signal_confidence=signal_conf,
            relative_position=rel_position,
            relative_rotation=rel_rotation,
            relative_yaw=rel_yaw,
            relative_yaw_0deg=rel_yaw_0deg,
            relative_yaw_180deg=rel_yaw_180deg,
            measurement_mode=measurement_mode,
            in_angle_roi=in_angle_roi,
            in_speed_roi=in_speed_roi
        )

        return detected_obj

    def get_all_tracked_objects(self) -> List[TrackedObject]:
        return list(self.trackers.values())

    def get_active_tracked_objects(self) -> List[TrackedObject]:
        return [t for t in self.trackers.values() if t.is_active]

    def reset(self):
        """Reset all trackers"""
        self.trackers.clear()
        self.next_object_id = 0
        self.frame_count = 0


def get_object_color(obj_id: int) -> Tuple[int, int, int]:
    """Get unique color for each object ID"""
    colors = [
        (0, 255, 0),    # Green
        (255, 0, 0),    # Blue
        (0, 0, 255),    # Red
        (255, 255, 0),  # Cyan
        (255, 0, 255),  # Magenta
        (0, 255, 255),  # Yellow
        (128, 255, 0),  # Light Green
        (255, 128, 0),  # Light Blue
    ]
    return colors[obj_id % len(colors)]


def get_signal_color(signal: str) -> Tuple[int, int, int]:
    """Get color for signal visualization"""
    colors = {
        "-2": (255, 0, 0),      # Blue
        "-1": (255, 128, 0),    # Light Blue
        "0":  (0, 255, 0),      # Green (center)
        "1":  (0, 165, 255),    # Orange
        "2":  (0, 0, 255),      # Red
        "raw": (0, 255, 255),   # Yellow (dual_angle 모드)
        "none": (128, 128, 128) # Gray
    }
    return colors.get(signal, (128, 128, 128))


def draw_coordinate_axes(image, center, rotation, scale=50):
    """Draw 3D coordinate axes on image"""
    if center is None:
        return image

    cx, cy = center
    roll = np.radians(rotation.get('roll', 0))
    pitch = np.radians(rotation.get('pitch', 0))
    yaw = np.radians(rotation.get('yaw', 0))

    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])

    R = Rz @ Ry @ Rx

    axes_3d = np.array([[scale, 0, 0], [0, scale, 0], [0, 0, scale]]).T
    rotated = R @ axes_3d

    x_end = (int(cx + rotated[0, 0]), int(cy - rotated[1, 0]))
    y_end = (int(cx + rotated[0, 1]), int(cy - rotated[1, 1]))
    z_end = (int(cx + rotated[0, 2] * 0.5), int(cy - rotated[1, 2] * 0.5))

    cv2.arrowedLine(image, (cx, cy), x_end, (0, 0, 255), 3, tipLength=0.2)
    cv2.putText(image, "X", (x_end[0] + 5, x_end[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.arrowedLine(image, (cx, cy), y_end, (0, 255, 0), 3, tipLength=0.2)
    cv2.putText(image, "Y", (y_end[0] + 5, y_end[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.arrowedLine(image, (cx, cy), z_end, (255, 0, 0), 3, tipLength=0.2)
    cv2.putText(image, "Z", (z_end[0] + 5, z_end[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    return image


def draw_mask_overlay(image, mask, color=(0, 255, 100), alpha=0.35):
    """Draw semi-transparent mask overlay"""
    overlay = image.copy()
    mask_bool = mask > 0
    overlay[mask_bool] = (
        overlay[mask_bool] * (1 - alpha) +
        np.array(color) * alpha
    ).astype(np.uint8)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay, contours, -1, color, 2)

    return overlay


def draw_signal_bar(image, signals: List[str], y_offset: int = 10):
    """Draw signal indicator bar on image for multi-object support"""
    bar_start = 10
    bar_width = 50
    bar_height = 25

    for i, sig in enumerate(["-2", "-1", "0", "1", "2"]):
        x = bar_start + i * (bar_width + 5)
        color = get_signal_color(sig)
        is_active = sig in signals

        # Draw background
        cv2.rectangle(image, (x, y_offset), (x + bar_width, y_offset + bar_height),
                     color if is_active else (50, 50, 50), -1)
        cv2.rectangle(image, (x, y_offset), (x + bar_width, y_offset + bar_height),
                     (255, 255, 255) if is_active else (100, 100, 100), 2)

        # Draw label
        label_color = (255, 255, 255) if is_active else (150, 150, 150)
        cv2.putText(image, sig, (x + 15, y_offset + 18),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, label_color, 2 if is_active else 1)

    # Draw 'none' indicator
    x = bar_start + 5 * (bar_width + 5)
    has_none = "none" in signals
    cv2.rectangle(image, (x, y_offset), (x + bar_width + 10, y_offset + bar_height),
                 (128, 128, 128) if has_none else (50, 50, 50), -1)
    cv2.rectangle(image, (x, y_offset), (x + bar_width + 10, y_offset + bar_height),
                 (255, 255, 255) if has_none else (100, 100, 100), 2)
    cv2.putText(image, "none", (x + 5, y_offset + 18),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255) if has_none else (150, 150, 150), 2 if has_none else 1)

    return image


FIXED_PANEL_HEIGHT = 350  # Fixed panel height for consistent video output


def create_info_panel(width, frame_data, objects: List[DetectedObject], stats, signal_gen,
                      ros_enabled, roi_enabled, tcp_publisher=None,
                      consensus_speed_m_min: float = 0.0):
    """Create information panel with Dual ROI (Speed + Angle) measurement data"""
    num_objects = len(objects)
    panel_height = FIXED_PANEL_HEIGHT
    panel = np.zeros((panel_height, width, 3), dtype=np.uint8)

    y_offset = 25

    # Frame info
    cv2.putText(panel, f"Frame {frame_data['frame']} | Conf >= {CONFIDENCE_THRESHOLD:.2f} | Active: {num_objects} | FPS: {stats['fps']:.1f}",
                (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    y_offset += 25

    # Consensus speed (전체 객체 평균 속도)
    speed_color = (0, 200, 255) if consensus_speed_m_min > 0.01 else (100, 100, 100)
    cv2.putText(panel, f"Consensus Speed: {consensus_speed_m_min:.2f} m/min",
                (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, speed_color, 2)
    y_offset += 22

    # Dual ROI info
    if roi_enabled:
        cv2.putText(panel, f"Angle ROI: [{ANGLE_ROI_TOP_LEFT[0]},{ANGLE_ROI_TOP_LEFT[1]}]-[{ANGLE_ROI_BOTTOM_RIGHT[0]},{ANGLE_ROI_BOTTOM_RIGHT[1]}]",
                    (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, ANGLE_ROI_COLOR, 1)
        y_offset += 16
        cv2.putText(panel, f"Speed ROI: [{SPEED_ROI_TOP_LEFT[0]},{SPEED_ROI_TOP_LEFT[1]}]-[{SPEED_ROI_BOTTOM_RIGHT[0]},{SPEED_ROI_BOTTOM_RIGHT[1]}] | Filtered: {stats.get('filtered_by_roi', 0)}",
                    (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, SPEED_ROI_COLOR, 1)
        y_offset += 18

    # Reference info
    if signal_gen.reference_pose:
        cv2.putText(panel, f"Ref Yaw: {signal_gen.reference_yaw:.1f} | RelYaw Valid: [{signal_gen.min_relative_yaw:.1f}, {signal_gen.max_relative_yaw:.1f}]",
                    (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 255, 100), 1)
        y_offset += 18

    # Signal bar 비활성화 (시그널 분류 기능 제거됨, 통합 상대각도 인터페이스)
    # [비활성화] active_signals = [obj.signal for obj in objects if obj.in_angle_roi]
    # [비활성화] panel = draw_signal_bar(panel, active_signals, y_offset)
    # y_offset += 40

    # Per-object information (통합 인터페이스: 속도 + 상대각도)
    for i, obj in enumerate(objects[:3]):
        obj_color = get_object_color(obj.object_id)
        cls_tag = obj.class_name if obj.class_name else f"c{obj.class_id}"

        cv2.rectangle(panel, (10, y_offset - 12), (width - 10, y_offset + 55), (40, 40, 40), -1)

        # 객체 헤더 (클래스명 + ROI 표시)
        roi_tag = ""
        if obj.in_angle_roi:
            roi_tag = "[Angle]"
        if obj.in_speed_roi:
            roi_tag += "[Speed]"
        cv2.putText(panel, f"{cls_tag} Obj{obj.object_id} {roi_tag} | Conf: {obj.confidence:.3f}",
                    (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, obj_color, 2)
        y_offset += 18

        if obj.position is not None:
            cv2.putText(panel, f"  Pos: X={obj.position[0]:.3f} Y={obj.position[1]:.3f} Z={obj.position[2]:.3f} [m]",
                        (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
            y_offset += 15

        # Speed ROI: m/min 속도 표시
        if obj.in_speed_roi:
            cv2.putText(panel, f"  [SPEED] {obj.speed_m_min:.2f} m/min",
                        (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, SPEED_ROI_COLOR, 1)
            y_offset += 15

        # Angle ROI: 통합 상대각도 표시 (0도 기준만)
        if obj.in_angle_roi:
            cv2.putText(panel, f"  [ANGLE] d0={obj.relative_yaw_0deg:+.1f} deg",
                        (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            y_offset += 15

        y_offset += 5

    if num_objects > 3:
        cv2.putText(panel, f"  ... and {num_objects - 3} more objects",
                    (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

    # Stats at bottom
    y_offset = panel_height - 25
    ros_status = "ON" if ros_enabled else "OFF"
    roi_status = "ON" if roi_enabled else "OFF"
    tcp_status = "OFF"
    if tcp_publisher and tcp_publisher.enabled:
        tcp_status = f"ON({tcp_publisher.client_count})"
    cv2.putText(panel, f"GPU: {'ON' if HAS_CUDA else 'OFF'} | ROS2: {ros_status} | TCP: {tcp_status} | ROI: {roi_status} | [Q]uit [S]ave [R]eset [P]ause [F]ullscreen",
                (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 200, 100) if HAS_CUDA else (150, 150, 150), 1)

    return panel


def create_no_detection_panel(width, frame_idx, stats, roi_enabled, ros_enabled):
    """Create panel for frames without detection"""
    panel_height = FIXED_PANEL_HEIGHT
    panel = np.zeros((panel_height, width, 3), dtype=np.uint8)

    y_offset = 25
    cv2.putText(panel, f"Frame {frame_idx} | Conf >= {CONFIDENCE_THRESHOLD:.2f} | No Detection in ROI | FPS: {stats['fps']:.1f}",
                (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    y_offset += 35

    # Dual ROI info
    if roi_enabled:
        cv2.putText(panel, f"Angle ROI: [{ANGLE_ROI_TOP_LEFT[0]},{ANGLE_ROI_TOP_LEFT[1]}]-[{ANGLE_ROI_BOTTOM_RIGHT[0]},{ANGLE_ROI_BOTTOM_RIGHT[1]}]",
                    (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, ANGLE_ROI_COLOR, 1)
        y_offset += 16
        cv2.putText(panel, f"Speed ROI: [{SPEED_ROI_TOP_LEFT[0]},{SPEED_ROI_TOP_LEFT[1]}]-[{SPEED_ROI_BOTTOM_RIGHT[0]},{SPEED_ROI_BOTTOM_RIGHT[1]}]",
                    (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, SPEED_ROI_COLOR, 1)
        y_offset += 20

    cv2.putText(panel, "No objects detected within ROI",
                (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)

    # Stats at bottom
    y_offset = panel_height - 25
    ros_status = "ON" if ros_enabled else "OFF"
    roi_status = "ON" if roi_enabled else "OFF"
    cv2.putText(panel, f"GPU: {'ON' if HAS_CUDA else 'OFF'} | ROS2: {ros_status} | ROI: {roi_status} | [Q]uit [S]ave [R]eset [P]ause [F]ullscreen",
                (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 200, 100) if HAS_CUDA else (150, 150, 150), 1)

    return panel


def main():
    parser = argparse.ArgumentParser(description='RealSense D455 6DOF Pose Streaming with ROS2 Signal')
    parser.add_argument('--width', type=int, default=640, help='Frame width')
    parser.add_argument('--height', type=int, default=480, help='Frame height')
    parser.add_argument('--fps', type=int, default=30, help='Frames per second')
    parser.add_argument('--save', type=str, default=None, help='Save video to file')
    parser.add_argument('--yolo-model', type=str, default=YOLO_MODEL_PATH, help='YOLO model path')
    parser.add_argument('--reference-dir', type=str, default=REFERENCE_DIR, help='Reference images directory')
    parser.add_argument('--conf-threshold', type=float, default=CONFIDENCE_THRESHOLD, help='YOLO confidence threshold')
    parser.add_argument('--no-display', action='store_true', help='Disable display window')
    parser.add_argument('--no-ros', action='store_true', help='Disable ROS2 publishing')
    parser.add_argument('--no-roi', action='store_true', help='Disable ROI filtering')
    parser.add_argument('--ros-topic', type=str, default='p_s', help='ROS2 topic name for signal publishing')
    parser.add_argument('--no-tcp', action='store_true', help='TCP 서버 비활성화')
    parser.add_argument('--tcp-host', type=str, default=TCP_HOST, help='TCP 서버 호스트')
    parser.add_argument('--tcp-port', type=int, default=TCP_PORT, help='TCP 서버 포트')
    args = parser.parse_args()

    # ROI enable flag
    roi_enabled = ROI_ENABLED and not args.no_roi

    print("=" * 70)
    print("RealSense D455 6DOF Pose Estimation with Multi-Object Tracking")
    print("=" * 70)
    print(f"[GPU] PyTorch CUDA: {HAS_CUDA}")
    print(f"[GPU] CuPy: {HAS_CUPY}")
    if HAS_CUDA:
        print(f"[GPU] Device: {torch.cuda.get_device_name(0)}")

    print(f"\n[Config]:")
    print(f"  - Confidence Threshold: {args.conf_threshold*100:.0f}%")
    print(f"  - ROI Enabled: {roi_enabled}")
    if roi_enabled:
        print(f"  - Angle ROI: [{ANGLE_ROI_TOP_LEFT[0]},{ANGLE_ROI_TOP_LEFT[1]}] to [{ANGLE_ROI_BOTTOM_RIGHT[0]},{ANGLE_ROI_BOTTOM_RIGHT[1]}]")
        print(f"  - Speed ROI: [{SPEED_ROI_TOP_LEFT[0]},{SPEED_ROI_TOP_LEFT[1]}] to [{SPEED_ROI_BOTTOM_RIGHT[0]},{SPEED_ROI_BOTTOM_RIGHT[1]}]")
    print(f"  - Multi-Object Tracking: Enabled")
    print(f"  - Object Ordering: Right-to-left")

    # Initialize ROS2 Publisher
    ros_publisher = None
    if not args.no_ros and HAS_ROS2:
        ros_publisher = ROS2SignalPublisher(topic_name=args.ros_topic)
        if ros_publisher.enabled:
            print(f"[ROS2] Publishing signals to topic: '{args.ros_topic}'")
    else:
        print("[ROS2] Signal publishing disabled")

    # Initialize TCP Publisher
    tcp_publisher = None
    if not args.no_tcp:
        tcp_publisher = TCPSignalPublisher(host=args.tcp_host, port=args.tcp_port)
        if tcp_publisher.enabled:
            print(f"[TCP] 서버 대기 중: {args.tcp_host}:{args.tcp_port}")
    else:
        print("[TCP] TCP 서버 비활성화")

    # Initialize camera
    camera = RealSenseCamera(width=args.width, height=args.height, fps=args.fps)
    if not camera.start():
        print("[ERROR] Failed to start camera")
        if ros_publisher:
            ros_publisher.shutdown()
        return

    intrinsics = camera.get_intrinsics()

    # Initialize YOLO
    print(f"[YOLO] Loading model: {args.yolo_model}")
    from ultralytics import YOLO
    yolo = YOLO(args.yolo_model)
    # YOLO .predict()는 내부적으로 GPU 자동 배치하므로 .to('cuda') 불필요
    print(f"[YOLO] Classes: {yolo.names}")

    # Initialize signal generator
    print(f"[Signal] Loading reference images from: {args.reference_dir}")
    signal_gen = ShapeBasedSignalGenerator(
        args.reference_dir, args.yolo_model, REFERENCE_IMAGE_FILE,
        standard_dirs=STANDARD_DIRS
    )
    # Depth 기반 각도 측정 강화: 카메라 내부 파라미터 전달
    signal_gen.set_intrinsics(intrinsics)

    # Initialize Signal Confirmation Manager (0.2s timeout)
    signal_confirm_mgr = SignalConfirmationManager(ros_publisher, tcp_publisher)
    print(f"[Signal] Signal confirmation timeout: {SignalConfirmationManager.CONFIRMATION_TIMEOUT}s")

    # Initialize Multi-Object Tracker (from test_result_22)
    multi_tracker = PersistentMultiObjectTracker(fps=float(args.fps), intrinsics=intrinsics)

    # State
    stats = {
        'fps': 0,
        'detections': 0,
        'rejected': 0,
        'filtered_by_roi': 0,
        'total_objects_created': 0,
        'confirmed_signals': 0
    }
    frame_idx = 0
    paused = False

    # FPS calculation
    frame_times = deque(maxlen=30)
    last_time = time.time()

    # Video writer
    writer = None
    if args.save:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out_height = args.height + FIXED_PANEL_HEIGHT
        writer = cv2.VideoWriter(args.save, fourcc, args.fps, (args.width, out_height))
        print(f"[Video] Saving to: {args.save}")

    print("\n[INFO] Starting stream... Press 'q' to quit, 'f' to toggle fullscreen")
    print("=" * 70)

    # Create resizable window with maximize/minimize support
    window_name = 'RealSense 6DOF + Multi-Object Tracking'
    if not args.no_display:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, args.width, args.height + FIXED_PANEL_HEIGHT)

    is_fullscreen = False

    try:
        while True:
            # Handle pause
            if paused:
                key = cv2.waitKey(100) & 0xFF
                if key == ord('p'):
                    paused = False
                    print("[INFO] Resumed")
                elif key == ord('q') or key == 27:
                    break
                continue

            # ROS2 spin
            if ros_publisher and ros_publisher.enabled:
                ros_publisher.spin_once()

            # Capture frame
            frame_start = time.time()
            rgb, depth = camera.read()

            if rgb is None or depth is None:
                continue

            # YOLO detection (YOLO .predict()는 내부적으로 device/dtype 관리하므로 외부 context 불필요)
            dets = yolo.predict(source=rgb, conf=args.conf_threshold, iou=0.45, verbose=False)

            # Default visualization
            vis = rgb.copy()

            # Dual ROI 그리기 (검출 뒤에 표시되도록 먼저 그림)
            if roi_enabled:
                vis = draw_roi_box(vis, SPEED_ROI_TOP_LEFT, SPEED_ROI_BOTTOM_RIGHT,
                                   SPEED_ROI_COLOR, ROI_THICKNESS, label="Speed ROI")
                vis = draw_roi_box(vis, ANGLE_ROI_TOP_LEFT, ANGLE_ROI_BOTTOM_RIGHT,
                                   ANGLE_ROI_COLOR, ROI_THICKNESS, label="Angle ROI")

            frame_data = {'frame': frame_idx}
            processed_objects = []
            has_detection = False

            # Dual ROI 기반 검출 처리
            if dets and dets[0].boxes and len(dets[0].boxes) > 0:
                det = dets[0]

                valid_detections = []
                for i, box in enumerate(det.boxes):
                    conf = float(box.conf[0].cpu().numpy())
                    stats['detections'] += 1

                    if conf < args.conf_threshold:
                        stats['rejected'] += 1
                        continue

                    bbox = box.xyxy[0].cpu().numpy().astype(int).tolist()
                    class_id = int(box.cls[0].cpu().numpy()) if hasattr(box, 'cls') else 0
                    class_name = CLASS_NAMES.get(class_id, f"class_{class_id}")

                    # Dual ROI 필터링 - 두 ROI 중 하나에 있으면 처리
                    if roi_enabled:
                        in_angle_roi = is_bbox_in_roi(bbox, ANGLE_ROI_TOP_LEFT,
                                                       ANGLE_ROI_BOTTOM_RIGHT, 0.5)
                        in_speed_roi = is_bbox_in_roi(bbox, SPEED_ROI_TOP_LEFT,
                                                       SPEED_ROI_BOTTOM_RIGHT, 0.5)
                        if not in_angle_roi and not in_speed_roi:
                            stats['filtered_by_roi'] += 1
                            continue
                    else:
                        in_angle_roi = True
                        in_speed_roi = True

                    # Get mask
                    mask = None
                    if hasattr(det, 'masks') and det.masks and i < len(det.masks.data):
                        m = det.masks.data[i].cpu().numpy()
                        if m.shape != rgb.shape[:2]:
                            mask = cv2.resize(m, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
                        else:
                            mask = m
                        mask = (mask > 0.5).astype(np.uint8) * 255
                    else:
                        mask = np.zeros(rgb.shape[:2], dtype=np.uint8)
                        x1, y1, x2, y2 = bbox
                        mask[y1:y2, x1:x2] = 255

                    valid_detections.append({
                        'bbox': bbox,
                        'confidence': conf,
                        'mask': mask,
                        'class_id': class_id,
                        'class_name': class_name,
                        'in_angle_roi': in_angle_roi,
                        'in_speed_roi': in_speed_roi,
                    })

                if valid_detections:
                    has_detection = True

                    processed_objects = multi_tracker.process_detections(
                        valid_detections, depth, frame_idx, signal_gen
                    )

                    stats['total_objects_created'] = multi_tracker.next_object_id

                    # 시각화 및 퍼블리시
                    for obj in processed_objects:
                        signal_color = get_signal_color(obj.signal)
                        obj_color = get_object_color(obj.object_id)
                        cls_color = CLASS_COLORS_MAP.get(obj.class_id, obj_color)

                        vis = draw_mask_overlay(vis, obj.mask, color=cls_color, alpha=0.35)

                        x1, y1, x2, y2 = obj.bbox
                        cv2.rectangle(vis, (x1, y1), (x2, y2), cls_color, 2)

                        # ROI별 라벨 표시 (클래스 이름 포함, 통합 dual_angle 인터페이스)
                        cls_tag = obj.class_name if obj.class_name else f"c{obj.class_id}"
                        if obj.in_angle_roi:
                            # 통합: 모든 클래스 0도 기준 상대각도(d0)만 표시
                            label = f"{cls_tag} Obj{obj.object_id} {obj.confidence:.2f}"
                            label2 = f"  d0:{obj.relative_yaw_0deg:+.1f}"
                            cv2.putText(vis, label, (x1, y1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cls_color, 2)
                            cv2.putText(vis, label2, (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)
                        elif obj.in_speed_roi:
                            label = f"{cls_tag} Obj{obj.object_id} {obj.confidence:.2f} Spd:{obj.speed_m_min:.1f}m/min"
                            cv2.putText(vis, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cls_color, 2)
                        else:
                            label = f"{cls_tag} Obj{obj.object_id} {obj.confidence:.2f}"
                            cv2.putText(vis, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cls_color, 2)

                        if obj.center_2d is not None and obj.rotation is not None:
                            vis = draw_coordinate_axes(vis, obj.center_2d, obj.rotation, scale=60)
                            cv2.circle(vis, obj.center_2d, 5, (255, 255, 255), -1)
                            cv2.circle(vis, obj.center_2d, 5, obj_color, 2)

                        # Angle ROI: signal confirmation manager로 전달
                        if obj.in_angle_roi:
                            signal_confirm_mgr.update_signal(
                                object_id=obj.object_id,
                                signal=obj.signal,
                                confidence=obj.confidence,
                                relative_yaw=obj.relative_yaw,
                                position=obj.position,
                                class_name=obj.class_name,
                                class_id=obj.class_id,
                                measurement_mode=obj.measurement_mode,
                                relative_yaw_0deg=obj.relative_yaw_0deg,
                                relative_yaw_180deg=obj.relative_yaw_180deg
                            )


            # Consensus speed TCP 퍼블리시 (전체 객체 평균 속도)
            if tcp_publisher and tcp_publisher.enabled and multi_tracker.consensus_speed_m_min > 0.01:
                tcp_publisher.publish_speed(multi_tracker.consensus_speed_m_min)

            # Signal confirmation 타임아웃 체크 및 발행
            confirmed_signals = signal_confirm_mgr.check_and_publish()
            stats['confirmed_signals'] += len(confirmed_signals)

            # Calculate FPS
            current_time = time.time()
            frame_times.append(current_time - last_time)
            last_time = current_time
            if len(frame_times) > 0:
                stats['fps'] = 1.0 / np.mean(frame_times)

            # Create info panel
            ros_enabled = ros_publisher is not None and ros_publisher.enabled
            if processed_objects:
                panel = create_info_panel(vis.shape[1], frame_data, processed_objects, stats, signal_gen, ros_enabled, roi_enabled, tcp_publisher, multi_tracker.consensus_speed_m_min)
            else:
                panel = create_no_detection_panel(vis.shape[1], frame_idx, stats, roi_enabled, ros_enabled)

            # Combine
            output_frame = np.vstack([vis, panel])

            # Display
            if not args.no_display:
                cv2.imshow(window_name, output_frame)

            # Save video
            if writer:
                writer.write(output_frame)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:
                break
            elif key == ord('s'):
                filename = f"frame_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
                cv2.imwrite(filename, output_frame)
                print(f"[Saved] {filename}")
            elif key == ord('r'):
                multi_tracker.reset()
                print("[INFO] Reset trackers")
            elif key == ord('p'):
                paused = True
                print("[INFO] Paused")
            elif key == ord('f'):
                # Toggle fullscreen mode
                is_fullscreen = not is_fullscreen
                if is_fullscreen:
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                    print("[INFO] Fullscreen ON")
                else:
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                    print("[INFO] Fullscreen OFF")

            frame_idx += 1

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")

    finally:
        camera.stop()
        if writer:
            writer.release()
        cv2.destroyAllWindows()
        if ros_publisher:
            ros_publisher.shutdown()
        if tcp_publisher:
            tcp_publisher.shutdown()

        print("\n" + "=" * 70)
        print("Streaming ended")
        print(f"Total frames: {frame_idx}")
        print(f"Total detections: {stats['detections']}")
        print(f"Rejected (low conf): {stats['rejected']}")
        print(f"Filtered by ROI: {stats['filtered_by_roi']}")
        print(f"Unique objects tracked: {stats['total_objects_created']}")
        print(f"Confirmed signals published: {stats['confirmed_signals']}")
        print("=" * 70)


if __name__ == "__main__":
    main()
