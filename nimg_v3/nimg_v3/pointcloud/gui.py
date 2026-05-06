"""
PointCloud Capture & Publish GUI Interface

PyQt5-based graphical user interface for pointcloud capture,
visualization, and ROS2 publishing.
"""

import sys
import os
import time
import numpy as np
from pathlib import Path
from typing import Optional

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QListWidget, QListWidgetItem, QCheckBox,
    QStatusBar, QGroupBox, QMessageBox, QSplitter, QFrame
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QImage, QPixmap, QFont

from .camera_wrapper import RealSenseCameraWrapper
from .capture_manager import PointCloudCaptureManager
from .visualizer import PointCloudVisualizer
from .ros2_publisher import ROS2PointCloudPublisher


class CameraWorker(QThread):
    """Worker thread for camera frame acquisition."""

    frame_ready = pyqtSignal(np.ndarray)
    error_occurred = pyqtSignal(str)

    def __init__(self, camera: RealSenseCameraWrapper):
        super().__init__()
        self.camera = camera
        self.running = True
        self.target_fps = 30
        self.frame_interval = 1.0 / self.target_fps

    def run(self):
        """Main loop for frame acquisition."""
        while self.running:
            try:
                start_time = time.time()

                rgb, depth = self.camera.read()

                if rgb is not None:
                    self.frame_ready.emit(rgb.copy())

                # Maintain target FPS
                elapsed = time.time() - start_time
                sleep_time = max(0, self.frame_interval - elapsed)
                time.sleep(sleep_time)

            except Exception as e:
                self.error_occurred.emit(str(e))
                time.sleep(0.1)

    def stop(self):
        """Stop the worker thread."""
        self.running = False


class PointCloudInterfaceGUI(QMainWindow):
    """Main GUI Window for PointCloud Interface"""

    def __init__(self, save_dir: str = None):
        super().__init__()

        # Determine save directory
        if save_dir is None:
            base_path = Path(__file__).parent.parent.parent
            save_dir = str(base_path / "pointcloud_data")

        self.save_dir = save_dir

        # Initialize components
        self.camera = None
        self.capture_manager = None
        self.visualizer = PointCloudVisualizer(point_size=2.0)
        self.ros_publisher = None

        self.camera_worker = None
        self.selected_pointcloud_path = None

        # FPS calculation
        self.fps_counter = 0
        self.fps_timer = QTimer()
        self.fps_timer.timeout.connect(self._update_fps)
        self.fps_timer.start(1000)  # Update every second
        self.current_fps = 0

        # Initialize UI
        self.init_ui()

        # Initialize camera
        self.init_camera()

        # Initialize ROS2 (optional)
        self.init_ros2()

        # Load saved list
        self.update_pointcloud_list()

    def init_ui(self):
        """Initialize the user interface."""
        self.setWindowTitle("PointCloud Capture & Publish Interface - FurSys nimg_v3")
        self.setMinimumSize(900, 700)
        self.resize(1000, 750)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # === Camera Preview Section ===
        preview_group = QGroupBox("Camera Preview")
        preview_layout = QVBoxLayout(preview_group)

        self.label_camera_preview = QLabel()
        self.label_camera_preview.setAlignment(Qt.AlignCenter)
        self.label_camera_preview.setMinimumSize(640, 480)
        self.label_camera_preview.setMaximumHeight(500)
        self.label_camera_preview.setStyleSheet(
            "QLabel { background-color: #2b2b2b; border: 1px solid #555; }"
        )
        self.label_camera_preview.setText("Camera initializing...")
        preview_layout.addWidget(self.label_camera_preview)

        main_layout.addWidget(preview_group)

        # === Control Buttons Section ===
        button_group = QGroupBox("Controls")
        button_layout = QHBoxLayout(button_group)

        # Capture button
        self.btn_capture = QPushButton("Capture")
        self.btn_capture.setMinimumHeight(40)
        self.btn_capture.setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }"
            "QPushButton:hover { background-color: #45a049; }"
            "QPushButton:disabled { background-color: #888; }"
        )
        self.btn_capture.clicked.connect(self.on_capture_clicked)
        button_layout.addWidget(self.btn_capture)

        # Publish button
        self.btn_publish = QPushButton("Publish to ROS2")
        self.btn_publish.setMinimumHeight(40)
        self.btn_publish.setStyleSheet(
            "QPushButton { background-color: #2196F3; color: white; font-weight: bold; }"
            "QPushButton:hover { background-color: #1976D2; }"
            "QPushButton:disabled { background-color: #888; }"
        )
        self.btn_publish.clicked.connect(self.on_publish_clicked)
        button_layout.addWidget(self.btn_publish)

        # Visualize button
        self.btn_visualize = QPushButton("Visualize 3D")
        self.btn_visualize.setMinimumHeight(40)
        self.btn_visualize.setStyleSheet(
            "QPushButton { background-color: #FF9800; color: white; font-weight: bold; }"
            "QPushButton:hover { background-color: #F57C00; }"
            "QPushButton:disabled { background-color: #888; }"
        )
        self.btn_visualize.clicked.connect(self.on_visualize_clicked)
        button_layout.addWidget(self.btn_visualize)

        # Preprocessing checkbox
        self.checkbox_preprocessing = QCheckBox("Preprocessing")
        self.checkbox_preprocessing.setChecked(True)
        self.checkbox_preprocessing.stateChanged.connect(self.on_preprocessing_toggled)
        button_layout.addWidget(self.checkbox_preprocessing)

        main_layout.addWidget(button_group)

        # === Saved PointCloud List Section ===
        list_group = QGroupBox("Saved PointClouds")
        list_layout = QVBoxLayout(list_group)

        self.list_saved_pointclouds = QListWidget()
        self.list_saved_pointclouds.setMinimumHeight(120)
        self.list_saved_pointclouds.itemClicked.connect(self.on_pointcloud_selected)
        self.list_saved_pointclouds.setStyleSheet(
            "QListWidget { font-family: monospace; }"
            "QListWidget::item:selected { background-color: #3d8ec9; }"
        )
        list_layout.addWidget(self.list_saved_pointclouds)

        # Delete button
        delete_layout = QHBoxLayout()
        self.btn_delete = QPushButton("Delete Selected")
        self.btn_delete.clicked.connect(self.on_delete_clicked)
        self.btn_delete.setStyleSheet(
            "QPushButton { background-color: #f44336; color: white; }"
            "QPushButton:hover { background-color: #d32f2f; }"
        )
        delete_layout.addStretch()
        delete_layout.addWidget(self.btn_delete)
        list_layout.addLayout(delete_layout)

        main_layout.addWidget(list_group)

        # === Status Bar ===
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.update_status_bar()

    def init_camera(self):
        """Initialize the camera."""
        try:
            self.camera = RealSenseCameraWrapper()

            if self.camera.start():
                self.capture_manager = PointCloudCaptureManager(
                    self.camera,
                    save_dir=self.save_dir,
                    preprocessing_enabled=self.checkbox_preprocessing.isChecked()
                )

                # Start camera worker thread
                self.camera_worker = CameraWorker(self.camera)
                self.camera_worker.frame_ready.connect(self.update_camera_frame)
                self.camera_worker.error_occurred.connect(self.on_camera_error)
                self.camera_worker.start()

                self.show_message("Camera initialized successfully", "success")
            else:
                self.show_message("Failed to initialize camera", "error")
                self.btn_capture.setEnabled(False)

        except Exception as e:
            self.show_message(f"Camera error: {e}", "error")
            self.btn_capture.setEnabled(False)

    def init_ros2(self):
        """Initialize ROS2 publisher."""
        try:
            if ROS2PointCloudPublisher.is_ros2_available():
                self.ros_publisher = ROS2PointCloudPublisher(topic_name="pc_point")

                if self.ros_publisher.init_ros2_node():
                    self.show_message("ROS2 initialized", "info")
                else:
                    self.show_message("ROS2 init failed", "warning")
            else:
                self.show_message("ROS2 not available", "warning")
                self.btn_publish.setEnabled(False)

        except Exception as e:
            print(f"ROS2 init error: {e}")
            self.btn_publish.setEnabled(False)

    def update_camera_frame(self, frame: np.ndarray):
        """Update camera preview with new frame."""
        try:
            self.fps_counter += 1

            # Convert BGR to RGB
            rgb_frame = frame[:, :, ::-1].copy()

            # Convert to QImage
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(
                rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888
            )

            # Scale to fit label while maintaining aspect ratio
            scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
                self.label_camera_preview.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )

            self.label_camera_preview.setPixmap(scaled_pixmap)

        except Exception as e:
            print(f"Frame update error: {e}")

    def _update_fps(self):
        """Update FPS counter."""
        self.current_fps = self.fps_counter
        self.fps_counter = 0
        self.update_status_bar()

    def on_capture_clicked(self):
        """Handle capture button click."""
        try:
            self.btn_capture.setEnabled(False)
            self.show_message("Capturing...", "info")

            # Capture pointcloud
            pcd = self.capture_manager.capture_pointcloud()

            # Save
            filepath = self.capture_manager.save_pointcloud(pcd, format="ply")

            # Update list
            self.update_pointcloud_list()

            filename = os.path.basename(filepath)
            self.show_message(f"Saved: {filename} ({len(pcd.points)} points)", "success")

        except Exception as e:
            self.show_message(f"Capture failed: {e}", "error")

        finally:
            self.btn_capture.setEnabled(True)

    def on_publish_clicked(self):
        """Handle publish button click."""
        if not self.selected_pointcloud_path:
            self.show_message("Please select a pointcloud", "warning")
            return

        if self.ros_publisher is None or not self.ros_publisher.is_ready():
            self.show_message("ROS2 not initialized", "error")
            return

        try:
            self.btn_publish.setEnabled(False)
            self.show_message("Publishing...", "info")

            # Load pointcloud
            pcd = self.capture_manager.load_pointcloud(self.selected_pointcloud_path)

            # Publish
            success = self.ros_publisher.publish_pointcloud(pcd)

            if success:
                self.show_message(f"Published to '{self.ros_publisher.get_topic_name()}'", "success")
            else:
                self.show_message("Publish failed", "error")

        except Exception as e:
            self.show_message(f"Publish failed: {e}", "error")

        finally:
            self.btn_publish.setEnabled(True)

    def on_visualize_clicked(self):
        """Handle visualize button click."""
        if not self.selected_pointcloud_path:
            self.show_message("Please select a pointcloud", "warning")
            return

        try:
            self.show_message("Opening visualizer...", "info")

            # Load pointcloud
            pcd = self.capture_manager.load_pointcloud(self.selected_pointcloud_path)

            # Visualize (blocking)
            filename = os.path.basename(self.selected_pointcloud_path)
            self.visualizer.visualize_pointcloud(
                pcd,
                window_name=f"PointCloud: {filename}"
            )

            self.show_message("Visualization closed", "info")

        except Exception as e:
            self.show_message(f"Visualization failed: {e}", "error")

    def on_preprocessing_toggled(self, state):
        """Handle preprocessing checkbox toggle."""
        enabled = state == Qt.Checked
        if self.capture_manager:
            self.capture_manager.set_preprocessing_enabled(enabled)
        self.show_message(f"Preprocessing: {'ON' if enabled else 'OFF'}", "info")

    def on_pointcloud_selected(self, item: QListWidgetItem):
        """Handle pointcloud selection."""
        if item:
            # Parse the path from item data
            data = item.data(Qt.UserRole)
            if data:
                self.selected_pointcloud_path = data
                self.show_message(f"Selected: {os.path.basename(data)}", "info")

    def on_delete_clicked(self):
        """Handle delete button click."""
        if not self.selected_pointcloud_path:
            self.show_message("Please select a pointcloud", "warning")
            return

        # Confirm deletion
        reply = QMessageBox.question(
            self, "Confirm Delete",
            f"Delete {os.path.basename(self.selected_pointcloud_path)}?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            success = self.capture_manager.delete_pointcloud(self.selected_pointcloud_path)
            if success:
                self.selected_pointcloud_path = None
                self.update_pointcloud_list()
                self.show_message("Deleted", "success")
            else:
                self.show_message("Delete failed", "error")

    def on_camera_error(self, error_msg: str):
        """Handle camera error."""
        print(f"Camera error: {error_msg}")

    def update_pointcloud_list(self):
        """Update the saved pointcloud list."""
        self.list_saved_pointclouds.clear()

        if self.capture_manager:
            saved_list = self.capture_manager.get_saved_list()

            for item_data in saved_list:
                filename = item_data['filename']
                num_points = item_data['num_points']
                size_mb = item_data.get('size_mb', 0)

                display_text = f"{filename}  |  {num_points:,} points  |  {size_mb:.2f} MB"

                list_item = QListWidgetItem(display_text)
                list_item.setData(Qt.UserRole, item_data['path'])
                self.list_saved_pointclouds.addItem(list_item)

    def update_status_bar(self):
        """Update the status bar."""
        # Camera status
        camera_ok = self.camera and self.camera.is_running
        camera_status = "Camera: OK" if camera_ok else "Camera: Error"

        # ROS2 status
        ros_ok = self.ros_publisher and self.ros_publisher.is_ready()
        ros_status = "ROS2: Connected" if ros_ok else "ROS2: Disconnected"

        # Saved count
        saved_count = len(self.capture_manager.saved_pointclouds) if self.capture_manager else 0

        status_text = f"{camera_status}  |  {ros_status}  |  FPS: {self.current_fps}  |  Saved: {saved_count}"
        self.status_bar.showMessage(status_text)

    def show_message(self, msg: str, level: str = "info"):
        """Show message in status bar."""
        prefix = ""
        if level == "success":
            prefix = "[OK] "
        elif level == "warning":
            prefix = "[WARN] "
        elif level == "error":
            prefix = "[ERROR] "

        self.status_bar.showMessage(f"{prefix}{msg}", 5000)
        print(f"[GUI] {prefix}{msg}")

    def closeEvent(self, event):
        """Handle window close event."""
        print("[GUI] Closing...")

        # Stop camera worker
        if self.camera_worker:
            self.camera_worker.stop()
            self.camera_worker.wait()

        # Stop camera
        if self.camera:
            self.camera.stop()

        # Shutdown ROS2
        if self.ros_publisher:
            self.ros_publisher.shutdown()

        event.accept()


def run_gui(save_dir: str = None):
    """Run the GUI application."""
    app = QApplication(sys.argv)

    # Set application style
    app.setStyle('Fusion')

    window = PointCloudInterfaceGUI(save_dir=save_dir)
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    run_gui()
