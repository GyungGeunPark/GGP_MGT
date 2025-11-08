#!/usr/bin/env python3

import sys
import os
import numpy as np
import cv2
from datetime import datetime
import threading
from collections import deque
import time
import struct

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2

# PyQt5
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QPushButton, QLabel, QFileDialog,
                            QMessageBox, QGroupBox, QSplitter, QCheckBox,
                            QSpinBox, QSlider, QComboBox)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QPixmap, QImage, QPainter, QPen, QColor

class SignalEmitter(QObject):
    """Signal emitter for thread-safe GUI updates"""
    update_rgb_signal = pyqtSignal(np.ndarray)
    update_depth_signal = pyqtSignal(np.ndarray)
    update_fps_signal = pyqtSignal(str)
    update_point_count_signal = pyqtSignal(int)

class PointCloudProcessor(QThread):
    """Separate thread for point cloud processing"""
    point_count_updated = pyqtSignal(int)
    
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.pointcloud_queue = deque(maxlen=1)
        self.running = True
        self.skip_frames = 1
        self.frame_counter = 0
        self.max_points = 0  # 0 means no limit
        
    def add_pointcloud(self, pc_msg):
        """Add new point cloud data"""
        self.pointcloud_queue.clear()
        self.pointcloud_queue.append(pc_msg)
        
    def set_max_points(self, max_points):
        """Set maximum number of points (0 for no limit)"""
        self.max_points = max_points
        
    def run(self):
        """Process point clouds in separate thread"""
        while self.running:
            if len(self.pointcloud_queue) > 0:
                pc_msg = self.pointcloud_queue.popleft()
                
                self.frame_counter += 1
                if self.frame_counter % self.skip_frames != 0:
                    continue
                
                try:
                    self.process_pointcloud(pc_msg)
                        
                except Exception as e:
                    print(f"Point cloud processing error: {e}")
                    import traceback
                    traceback.print_exc()
                    
            else:
                time.sleep(0.01)
                
    def process_pointcloud(self, pc_msg):
        """Process point cloud for saving"""
        try:
            if pc_msg.width * pc_msg.height == 0:
                print("Empty point cloud received")
                return
            
            # Read points
            points_list = []
            colors_list = []
            
            # Debug: Check fields
            field_names = [f.name for f in pc_msg.fields]
            if self.frame_counter % 30 == 1:  # Print occasionally
                print(f"PointCloud2 fields: {field_names}")
            
            has_rgb = any(field.name == 'rgb' for field in pc_msg.fields)
            
            if has_rgb:
                point_generator = pc2.read_points(pc_msg, skip_nans=True,
                                                field_names=("x", "y", "z", "rgb"))
            else:
                point_generator = pc2.read_points(pc_msg, skip_nans=True,
                                                field_names=("x", "y", "z"))

            point_count = 0
            valid_rgb_count = 0
            
            # ### 수정된 필터링 로직 ###
            # 시야각(FOV) 기반 필터링을 위한 비율 설정
            # 이 값들을 조정하여 필터링 강도를 변경할 수 있습니다.
            # 값이 작을수록 더 좁은 영역의 포인트만 남습니다. (더 많이 잘라냄)
            # 일반적인 카메라 렌즈의 비율과 유사하게 설정합니다.
            horizontal_fov_ratio = 0.8  # X / Z 비율 임계값
            vertical_fov_ratio = 0.6    # Y / Z 비율 임계값

            for point in point_generator:
                if self.max_points > 0 and point_count >= self.max_points:
                    break
                
                # Z값이 유효한지 먼저 확인
                if not (np.isnan(point[0]) or np.isnan(point[1]) or np.isnan(point[2])):
                    # 기존 Z 필터링
                    if 0.01 < point[2] < 20.0:
                        
                        # <<< 새로운 FOV 필터링 로직 추가 >>>
                        # Z 거리에 비례하여 X, Y 좌표를 필터링합니다.
                        z = point[2]
                        if z > 0 and abs(point[0] / z) < horizontal_fov_ratio and abs(point[1] / z) < vertical_fov_ratio:
                            points_list.append([point[0], point[1], point[2]])
                            
                            if has_rgb:
                                try:
                                    rgb_packed = point[3]
                                    s = struct.pack('>f', rgb_packed)
                                    i = struct.unpack('>l', s)[0]
                                    r = (i >> 16) & 0xFF
                                    g = (i >> 8) & 0xFF
                                    b = i & 0xFF
                                    
                                    if r > 0 or g > 0 or b > 0:
                                        valid_rgb_count += 1
                                    
                                    colors_list.append([r, g, b])
                                except:
                                    colors_list.append([255, 255, 255])
                            else:
                                colors_list.append([255, 255, 255])
                            
                            point_count += 1
            # ### 필터링 로직 수정 끝 ###

            if not points_list:
                return
            
            points = np.array(points_list)
            colors = np.array(colors_list)
            
            # Debug: Print statistics occasionally
            if self.frame_counter % 30 == 1:
                print(f"Processed {len(points)} points, {valid_rgb_count} with valid RGB")
                if len(points) > 0:
                    print(f"Point ranges - X: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}], "
                          f"Y: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}], "
                          f"Z: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")
            
            # If no valid colors, use white
            if valid_rgb_count == 0 and len(colors) > 0:
                colors.fill(255)
            
            # Store for saving
            self.ros_node.latest_points = points
            self.ros_node.latest_colors = colors
            
            self.point_count_updated.emit(len(points))

        except Exception as e:
            print(f"Error in process_pointcloud: {e}")
            import traceback
            traceback.print_exc()
        
    def stop(self):
        """Stop the processing thread"""
        self.running = False

class RealSenseNode(Node):
    """ROS2 Node for RealSense D455 camera"""
    
    def __init__(self, signal_emitter):
        super().__init__('realsense_viewer_node')
        
        self.signal_emitter = signal_emitter
        self.bridge = CvBridge()
        
        # Latest data storage
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.latest_points = None
        self.latest_colors = None
        
        # Performance monitoring
        self.last_time = time.time()
        self.frame_count = 0
        
        # Point cloud processor
        self.pc_processor = PointCloudProcessor(self)
        
        # Connect the signal before starting the thread
        self.pc_processor.point_count_updated.connect(
            lambda count: self.signal_emitter.update_point_count_signal.emit(count)
        )
        
        self.pc_processor.start()
        
        # Get max points parameter
        self.declare_parameter('max_points', 0)
        max_points = self.get_parameter('max_points').value
        self.pc_processor.set_max_points(max_points)
        
        # 토픽 이름 확인 및 구독 설정
        self.get_logger().info('Setting up camera topic subscriptions...')
        
        # RGB 구독 - 여러 토픽 시도
        rgb_topics = [
            '/camera/camera/color/image_raw',
            '/camera/color/image_raw', 
            '/camera/rgb/image_raw'
        ]
        
        depth_topics = [
            '/camera/camera/depth/image_rect_raw',
            '/camera/depth/image_rect_raw',
            '/camera/depth/image_raw'
        ]
        
        pointcloud_topics = [
            '/camera/camera/depth/color/points',
            '/camera/depth/color/points',
            '/camera/points'
        ]
        
        # RGB 구독
        for topic in rgb_topics:
            try:
                self.rgb_subscription = self.create_subscription(
                    Image,
                    topic,
                    self.rgb_callback,
                    1
                )
                self.get_logger().info(f'Subscribed to RGB topic: {topic}')
                break
            except Exception as e:
                self.get_logger().warn(f'Failed to subscribe to {topic}: {e}')
                continue
        
        # Depth 구독
        for topic in depth_topics:
            try:
                self.depth_subscription = self.create_subscription(
                    Image,
                    topic,
                    self.depth_callback,
                    1
                )
                self.get_logger().info(f'Subscribed to Depth topic: {topic}')
                break
            except Exception as e:
                self.get_logger().warn(f'Failed to subscribe to {topic}: {e}')
                continue
        
        # Point cloud 구독
        for topic in pointcloud_topics:
            try:
                self.pointcloud_subscription = self.create_subscription(
                    PointCloud2,
                    topic,
                    self.pointcloud_callback,
                    1
                )
                self.get_logger().info(f'Subscribed to PointCloud topic: {topic}')
                break
            except Exception as e:
                self.get_logger().warn(f'Failed to subscribe to {topic}: {e}')
                continue
        
        # FPS timer
        self.fps_timer = self.create_timer(1.0, self.update_fps)
        
        # 토픽 상태 확인 타이머
        self.topic_check_timer = self.create_timer(5.0, self.check_topics)
        
        self.get_logger().info('RealSense viewer node initialized')
        
    def check_topics(self):
        """토픽 상태 주기적 확인"""
        topic_names = self.get_topic_names_and_types()
        camera_topics = [name for name, _ in topic_names if 'camera' in name]
        
        if camera_topics:
            self.get_logger().info(f'Available camera topics: {camera_topics[:5]}...')  # 처음 5개만 표시
        else:
            self.get_logger().warn('No camera topics found')
        
    def rgb_callback(self, msg):
        """Handle RGB image messages"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_rgb_image = cv_image
            self.signal_emitter.update_rgb_signal.emit(cv_image)
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {str(e)}')
    
    def depth_callback(self, msg):
        """Handle depth image messages"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth_image = cv_image
            
            # Normalize for visualization
            depth_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
            
            self.signal_emitter.update_depth_signal.emit(depth_colored)
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {str(e)}')
    
    def pointcloud_callback(self, msg):
        """Handle point cloud messages"""
        try:
            self.pc_processor.add_pointcloud(msg)
        except Exception as e:
            self.get_logger().error(f'PointCloud callback error: {str(e)}')
        
    def update_fps(self):
        """Update FPS display"""
        current_time = time.time()
        elapsed = current_time - self.last_time
        
        if elapsed > 0:
            fps = self.frame_count / elapsed
            fps_text = f"FPS: {fps:.1f}"
            self.signal_emitter.update_fps_signal.emit(fps_text)
            
        self.frame_count = 0
        self.last_time = current_time
        
    def cleanup(self):
        """Cleanup resources"""
        self.pc_processor.stop()
        self.pc_processor.wait()

class RealSenseViewer(QMainWindow):
    """PyQt5 GUI for RealSense camera viewer"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RealSense D455 Viewer - RGB/Depth Only")
        self.setGeometry(100, 100, 1400, 900)
        
        # Signal emitter
        self.signal_emitter = SignalEmitter()
        self.signal_emitter.update_rgb_signal.connect(self.update_rgb_display)
        self.signal_emitter.update_depth_signal.connect(self.update_depth_display)
        self.signal_emitter.update_fps_signal.connect(self.update_fps_display)
        self.signal_emitter.update_point_count_signal.connect(self.update_point_count_display)
        
        # ROS2 Node
        self.ros_node = None
        
        # Current data
        self.current_rgb_image = None
        self.current_depth_image = None
        
        # Setup UI
        self.setup_ui()
        
        # Start ROS2 in separate thread
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
    def setup_ui(self):
        """Setup the user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Control panel
        control_layout = QHBoxLayout()
        
        # Performance settings
        perf_label = QLabel("Performance Settings:")
        control_layout.addWidget(perf_label)
        
        skip_label = QLabel("Process every N frames:")
        self.skip_spinner = QSpinBox()
        self.skip_spinner.setMinimum(1)
        self.skip_spinner.setMaximum(10)
        self.skip_spinner.setValue(1)
        self.skip_spinner.valueChanged.connect(self.on_skip_frames_changed)
        
        control_layout.addWidget(skip_label)
        control_layout.addWidget(self.skip_spinner)
        
        # Point limit settings
        limit_label = QLabel("Max Points (0=no limit):")
        self.limit_spinner = QSpinBox()
        self.limit_spinner.setMinimum(0)
        self.limit_spinner.setMaximum(10000000)
        self.limit_spinner.setSingleStep(100000)
        self.limit_spinner.setValue(0)
        self.limit_spinner.valueChanged.connect(self.on_max_points_changed)
        
        control_layout.addWidget(limit_label)
        control_layout.addWidget(self.limit_spinner)
        
        # Status labels
        self.fps_label = QLabel("FPS: 0.0")
        self.fps_label.setStyleSheet("font-weight: bold; color: green;")
        control_layout.addWidget(self.fps_label)
        
        self.point_count_label = QLabel("Points: 0")
        self.point_count_label.setStyleSheet("font-weight: bold; color: blue;")
        control_layout.addWidget(self.point_count_label)
        
        control_layout.addStretch()
        main_layout.addLayout(control_layout)
        
        # Display area - only RGB and Depth
        image_layout = QHBoxLayout()
        
        # RGB Image group
        rgb_group = QGroupBox("RGB Image")
        rgb_layout = QVBoxLayout()
        self.rgb_label = QLabel("Waiting for RGB image...")
        self.rgb_label.setMinimumSize(640, 480)
        self.rgb_label.setScaledContents(True)
        self.rgb_label.setStyleSheet("border: 1px solid black;")
        rgb_layout.addWidget(self.rgb_label)
        rgb_group.setLayout(rgb_layout)
        
        # Depth Image group
        depth_group = QGroupBox("Depth Image")
        depth_layout = QVBoxLayout()
        self.depth_label = QLabel("Waiting for depth image...")
        self.depth_label.setMinimumSize(640, 480)
        self.depth_label.setScaledContents(True)
        self.depth_label.setStyleSheet("border: 1px solid black;")
        depth_layout.addWidget(self.depth_label)
        depth_group.setLayout(depth_layout)
        
        image_layout.addWidget(rgb_group)
        image_layout.addWidget(depth_group)
        
        main_layout.addLayout(image_layout)
        
        # Control buttons
        button_layout = QHBoxLayout()
        
        self.save_rgb_btn = QPushButton("Save RGB Image (.png)")
        self.save_rgb_btn.clicked.connect(self.save_rgb_image)
        
        self.save_depth_btn = QPushButton("Save Depth Image (.png)")
        self.save_depth_btn.clicked.connect(self.save_depth_image)
        
        self.save_pc_btn = QPushButton("Save Point Cloud (.ply)")
        self.save_pc_btn.clicked.connect(self.save_pointcloud)
        
        self.save_all_btn = QPushButton("Save All")
        self.save_all_btn.clicked.connect(self.save_all)
        
        button_layout.addWidget(self.save_rgb_btn)
        button_layout.addWidget(self.save_depth_btn)
        button_layout.addWidget(self.save_pc_btn)
        button_layout.addWidget(self.save_all_btn)
        
        main_layout.addLayout(button_layout)
        
        # Status bar
        self.statusBar().showMessage("Ready")
        
    def on_skip_frames_changed(self, value):
        """Update frame skip setting"""
        if self.ros_node and hasattr(self.ros_node, 'pc_processor'):
            self.ros_node.pc_processor.skip_frames = value
            
    def on_max_points_changed(self, value):
        """Update maximum points limit"""
        if self.ros_node and hasattr(self.ros_node, 'pc_processor'):
            self.ros_node.pc_processor.set_max_points(value)
            
    def update_fps_display(self, fps_text):
        """Update FPS display"""
        self.fps_label.setText(fps_text)
        
    def update_point_count_display(self, count):
        """Update point count display"""
        self.point_count_label.setText(f"Points: {count:,}")
        
    def ros_spin(self):
        """Run ROS2 node in separate thread"""
        rclpy.init()
        self.ros_node = RealSenseNode(self.signal_emitter)
        rclpy.spin(self.ros_node)
        
    def update_rgb_display(self, cv_image):
        """Update RGB image display"""
        self.current_rgb_image = cv_image
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(q_image)
        scaled_pixmap = pixmap.scaled(self.rgb_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.rgb_label.setPixmap(scaled_pixmap)
        
    def update_depth_display(self, cv_image):
        """Update depth image display"""
        self.current_depth_image = cv_image
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_BGR888)
        pixmap = QPixmap.fromImage(q_image)
        scaled_pixmap = pixmap.scaled(self.depth_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.depth_label.setPixmap(scaled_pixmap)
        
    def update_pointcloud_display(self, preview_image):
        """Update point cloud display"""
        # This method is no longer used but kept for compatibility
        pass
            
    def save_rgb_image(self):
        """Save RGB image as PNG"""
        if self.current_rgb_image is None:
            QMessageBox.warning(self, "Warning", "No RGB image available to save!")
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"rgb_image_{timestamp}.png"
        
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save RGB Image", filename, "PNG Files (*.png)"
        )
        
        if file_path:
            cv2.imwrite(file_path, self.current_rgb_image)
            self.statusBar().showMessage(f"RGB image saved: {file_path}")
            
    def save_depth_image(self):
        """Save depth image as PNG"""
        if self.ros_node is None or self.ros_node.latest_depth_image is None:
            QMessageBox.warning(self, "Warning", "No depth image available to save!")
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"depth_image_{timestamp}.png"
        
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Depth Image", filename, "PNG Files (*.png)"
        )
        
        if file_path:
            cv2.imwrite(file_path, self.ros_node.latest_depth_image.astype(np.uint16))
            self.statusBar().showMessage(f"Depth image saved: {file_path}")
            
    def save_pointcloud(self):
        """Save point cloud with RGB as PLY"""
        if self.ros_node is None or self.ros_node.latest_points is None:
            QMessageBox.warning(self, "Warning", "No point cloud available to save!")
            return
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"pointcloud_rgb_{timestamp}.ply"
        
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Save Point Cloud", filename, "PLY Files (*.ply)"
        )
        
        if file_path:
            try:
                points = self.ros_node.latest_points
                colors = self.ros_node.latest_colors
                
                self.save_ply_file(file_path, points, colors)
                self.statusBar().showMessage(f"Point cloud saved: {file_path} ({len(points):,} points)")
                
            except Exception as e:
                QMessageBox.error(self, "Error", f"Failed to save point cloud: {str(e)}")
    
    def save_ply_file(self, filename, points, colors=None):
        """Save points and colors to PLY file"""
        num_points = len(points)
        
        with open(filename, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"comment RealSense D455 point cloud with RGB\n")
            f.write(f"comment Total points: {num_points}\n")
            f.write(f"element vertex {num_points}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            
            if colors is not None:
                f.write("property uchar red\n")
                f.write("property uchar green\n")
                f.write("property uchar blue\n")
            
            f.write("end_header\n")
            
            for i in range(num_points):
                if colors is not None:
                    f.write(f"{points[i, 0]:.6f} {points[i, 1]:.6f} {points[i, 2]:.6f} ")
                    f.write(f"{int(colors[i, 0])} {int(colors[i, 1])} {int(colors[i, 2])}\n")
                else:
                    f.write(f"{points[i, 0]:.6f} {points[i, 1]:.6f} {points[i, 2]:.6f}\n")
            
    def save_all(self):
        """Save all data at once"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        directory = QFileDialog.getExistingDirectory(self, "Select Directory to Save All Files")
        
        if directory:
            saved_files = []
            
            if self.current_rgb_image is not None:
                rgb_path = os.path.join(directory, f"rgb_image_{timestamp}.png")
                cv2.imwrite(rgb_path, self.current_rgb_image)
                saved_files.append("RGB image")
                
            if self.ros_node and self.ros_node.latest_depth_image is not None:
                depth_path = os.path.join(directory, f"depth_image_{timestamp}.png")
                cv2.imwrite(depth_path, self.ros_node.latest_depth_image.astype(np.uint16))
                saved_files.append("Depth image")
                
            if self.ros_node and self.ros_node.latest_points is not None:
                pc_path = os.path.join(directory, f"pointcloud_rgb_{timestamp}.ply")
                try:
                    self.save_ply_file(pc_path, 
                                       self.ros_node.latest_points, 
                                       self.ros_node.latest_colors)
                    saved_files.append(f"Point cloud ({len(self.ros_node.latest_points):,} points)")
                except Exception as e:
                    self.statusBar().showMessage(f"Point cloud save error: {str(e)}")
                
            if saved_files:
                self.statusBar().showMessage(f"Saved: {', '.join(saved_files)} to {directory}")
            else:
                QMessageBox.warning(self, "Warning", "No data available to save!")
                
    def closeEvent(self, event):
        """Handle window close event"""
        if self.ros_node:
            self.ros_node.cleanup()
            self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    viewer = RealSenseViewer()
    viewer.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()