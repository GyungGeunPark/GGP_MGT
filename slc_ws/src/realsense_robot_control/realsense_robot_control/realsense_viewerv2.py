#!/usr/bin/env python3

import sys
import os
import numpy as np
import cv2
from datetime import datetime
import threading
import queue
import signal
from collections import deque
import time
import struct
import json
import yaml
import subprocess
import shutil

# ROS2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2

try:
    import sip
    # SIP 4.x와 5.x 호환성
    if hasattr(sip, 'setapi'):
        sip.setapi('QString', 2)
        sip.setapi('QVariant', 2)
    print(f"SIP version: {sip.SIP_VERSION_STR}")
except ImportError:
    print("SIP not available")

# PyQt5 import 전에 환경 변수 설정
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = ''
os.environ['QT_LOGGING_RULES'] = 'qt5ct.debug=false'

# PyQt5 - QRadioButton을 명시적으로 포함
try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                QHBoxLayout, QPushButton, QLabel, QFileDialog,
                                QMessageBox, QGroupBox, QSplitter, QCheckBox,
                                QSpinBox, QSlider, QComboBox, QTabWidget,
                                QDialog, QDialogButtonBox, QTextEdit, QListWidget,
                                QProgressBar, QRadioButton)
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
    from PyQt5.QtGui import QPixmap, QImage, QPainter, QPen, QColor
    
    PYQT5_AVAILABLE = True
    print("PyQt5 successfully imported with QRadioButton")
    
except ImportError as e:
    print(f"PyQt5 import error: {e}")
    PYQT5_AVAILABLE = False
    
    # 더미 클래스들 생성 - QRadioButton 포함
    class QWidget:
        def __init__(self, *args, **kwargs): pass
        def setMinimumSize(self, *args): pass
        def setLayout(self, *args): pass
        def setStyleSheet(self, *args): pass
        def setEnabled(self, *args): pass
        def setVisible(self, *args): pass
        def setText(self, *args): pass
        def setWordWrap(self, *args): pass
        def setScaledContents(self, *args): pass
        def setMaximumHeight(self, *args): pass
        def setMaximumSize(self, *args): pass
        def setReadOnly(self, *args): pass
        def setPlainText(self, *args): pass
        def setToolTip(self, *args): pass
        def clicked(self): 
            return type('obj', (object,), {'connect': lambda x: None})()
        def valueChanged(self):
            return type('obj', (object,), {'connect': lambda x: None})()
        def timeout(self):
            return type('obj', (object,), {'connect': lambda x: None})()
        def size(self):
            return type('obj', (object,), {'width': lambda: 640, 'height': lambda: 480})()
    
    class QMainWindow(QWidget):
        def __init__(self, *args, **kwargs): 
            super().__init__()
            
        def setCentralWidget(self, *args): pass
        def statusBar(self): 
            return type('obj', (object,), {'showMessage': lambda x: print(f"Status: {x}")})()
        def setWindowTitle(self, *args): pass
        def setGeometry(self, *args): pass
        def show(self): pass
    
    class QMessageBox:
        @staticmethod
        def warning(parent, title, message):
            print(f"WARNING: {title} - {message}")
        
        @staticmethod
        def error(parent, title, message):
            print(f"ERROR: {title} - {message}")
        
        @staticmethod
        def information(parent, title, message):
            print(f"INFO: {title} - {message}")
    
    class QRadioButton(QWidget):
        def __init__(self, text=""):
            super().__init__()
            self.text = text
            self._checked = False
        
        def setChecked(self, checked):
            self._checked = checked
        
        def isChecked(self):
            return self._checked
        
        def setToolTip(self, tooltip):
            pass
    
    # 다른 필요한 더미 클래스들
    class QVBoxLayout: 
        def __init__(self): pass
        def addWidget(self, *args): pass
        def addLayout(self, *args): pass
        def addStretch(self): pass
    
    class QHBoxLayout(QVBoxLayout): pass
    class QPushButton(QWidget):
        def __init__(self, text=""):
            super().__init__()
            self.text = text
            self._click_handler = None
        
        def clicked(self):
            """더미 clicked 시그널"""
            return type('obj', (object,), {
                'connect': self._connect_click
            })()
        
        def _connect_click(self, handler):
            """클릭 핸들러 연결"""
            self._click_handler = handler
            print(f"Button '{self.text}' connected to handler")
        
        def click(self):
            """프로그래매틱 클릭"""
            if self._click_handler:
                print(f"Executing click handler for button '{self.text}'")
                try:
                    self._click_handler()
                except Exception as e:
                    print(f"Error in click handler: {e}")
            else:
                print(f"No handler for button '{self.text}'")
    class QLabel(QWidget): pass
    class QTabWidget(QWidget): 
        def addTab(self, *args): pass
        def setCurrentIndex(self, *args): pass
    class QTextEdit(QWidget): 
        def append(self, *args): pass
        def verticalScrollBar(self):
            return type('obj', (object,), {
                'setValue': lambda x: None,
                'maximum': lambda: 0
            })()
    class QListWidget(QWidget): 
        def addItem(self, *args): pass
        def clear(self): pass
        def currentRow(self): return 0
        def takeItem(self, *args): pass
    class QProgressBar(QWidget): 
        def setVisible(self, *args): pass
        def setRange(self, *args): pass
    class QSpinBox(QWidget):
        def setRange(self, *args): pass
        def setValue(self, *args): pass
        def getValue(self): return 100
        def value(self): return 100
        def setSuffix(self, *args): pass
        def setMinimum(self, *args): pass
        def setMaximum(self, *args): pass
    class QComboBox(QWidget):
        def addItems(self, *args): pass
        def currentText(self): return "INTENDED"
        def findText(self, *args): return 0
        def setCurrentIndex(self, *args): pass
    class QGroupBox(QWidget): pass
    class QCheckBox(QWidget):
        def setChecked(self, *args): pass
        def isChecked(self): return False
    class QSlider(QWidget):
        def setRange(self, *args): pass
        def setValue(self, *args): pass
        def value(self): return 50
    class QSplitter(QWidget): pass
    class QDialog(QWidget): pass
    class QDialogButtonBox(QWidget): pass
    class QFileDialog:
        @staticmethod
        def getOpenFileName(parent=None, caption="", directory="", filter="", options=None):
            """더미 파일 다이얼로그 - 터미널 기반으로 처리"""
            print(f"File dialog called: {caption}")
            
            # 자동으로 터미널 기반 파일 선택으로 전환
            if hasattr(parent, 'get_file_via_terminal'):
                if "yaml" in filter.lower():
                    file_path = parent.get_file_via_terminal(caption, "*.yaml")
                elif "image" in filter.lower():
                    file_path = parent.get_file_via_terminal(caption, "*.png")
                elif "json" in filter.lower():
                    file_path = parent.get_file_via_terminal(caption, "*.json")
                else:
                    file_path = parent.get_file_via_terminal(caption, "*.*")
                
                return file_path or "", ""
            else:
                # 기본 경로들에서 파일 찾기
                import glob
                import os
                
                search_paths = [
                    "/root/slc_ws/src/realsense_robot_control/*.yaml",
                    "/root/slc_ws/src/realsense_robot_control/*.yml", 
                    "./output/*/*.yaml",
                    "./test_data/*.json",
                    "./*.yaml",
                    "./*.yml"
                ]
                
                found_files = []
                for pattern in search_paths:
                    try:
                        files = glob.glob(pattern)
                        found_files.extend(files)
                    except:
                        pass
                
                if found_files:
                    print(f"Auto-selecting first available file: {found_files[0]}")
                    return found_files[0], ""
                else:
                    print("No files found")
                    return "", ""
        
        @staticmethod
        def getExistingDirectory(parent=None, caption="", directory="", options=None):
            """더미 디렉토리 다이얼로그"""
            print(f"Directory dialog called: {caption}")
            return directory or os.getcwd()
    
    # Qt 상수들
    class Qt:
        KeepAspectRatio = 1
        SmoothTransformation = 1
        Horizontal = 1
    
    class pyqtSignal:
        def __init__(self, *args): pass
        def emit(self, *args): pass
        def connect(self, *args): pass
    
    class QObject: pass
    class QThread: 
        def start(self): pass
        def wait(self, *args): pass
        def isRunning(self): return False
    class QTimer: 
        def start(self, *args): pass
        def stop(self): pass
    class QPixmap: 
        @staticmethod
        def fromImage(*args): return QPixmap()
        def scaled(self, *args, **kwargs): return QPixmap()
        def setPixmap(self, *args): pass
    class QImage: 
        Format_BGR888 = 1
        def __init__(self, *args): pass
    class QApplication:
        def __init__(self, *args): pass
        def exec_(self): return 0
        def quit(self): pass
        @staticmethod
        def processEvents(): pass
    
    # Additional classes
    class QPainter: pass
    class QPen: pass
    class QColor: pass

# 안전한 메시지 박스 함수들
def safe_message_box_error(parent, title, message):
    """안전한 메시지 박스 에러 표시"""
    if PYQT5_AVAILABLE:
        try:
            QMessageBox.error(parent, title, message)
        except Exception as e:
            print(f"QMessageBox error failed: {e}")
            print(f"ERROR: {title} - {message}")
    else:
        print(f"ERROR: {title} - {message}")

def safe_message_box_warning(parent, title, message):
    """안전한 메시지 박스 경고 표시"""
    if PYQT5_AVAILABLE:
        try:
            QMessageBox.warning(parent, title, message)
        except Exception as e:
            print(f"QMessageBox warning failed: {e}")
            print(f"WARNING: {title} - {message}")
    else:
        print(f"WARNING: {title} - {message}")

def safe_message_box_info(parent, title, message):
    """안전한 메시지 박스 정보 표시"""
    if PYQT5_AVAILABLE:
        try:
            QMessageBox.information(parent, title, message)
        except Exception as e:
            print(f"QMessageBox info failed: {e}")
            print(f"INFO: {title} - {message}")
    else:
        print(f"INFO: {title} - {message}")

# Import our custom modules - 절대 import로 수정
try:
    # 패키지에서 import 시도
    from realsense_robot_control.motion_generator import MotionGenerator
    from realsense_robot_control.motion_visualizer import MotionVisualizerWidget
    from realsense_robot_control.robot_controller import RobotMotionExecutor
    print("Successfully imported modules from package")
except ImportError as e:
    print(f"Package import failed: {e}")
    try:
        # 상대 import 시도
        from .motion_generator import MotionGenerator
        from .motion_visualizer import MotionVisualizerWidget
        from .robot_controller import RobotMotionExecutor
        print("Successfully imported modules with relative import")
    except ImportError as e2:
        print(f"Relative import also failed: {e2}")
        print("Creating dummy classes...")
        
        # 더미 클래스들 생성
        class MotionGenerator:
            def __init__(self):
                pass
            def generate_motion_from_surfaces(self, *args, **kwargs):
                return "dummy.yaml", "dummy.json"
        
        class MotionVisualizerWidget(QWidget):
            def __init__(self):
                super().__init__()
                self.setMinimumSize(400, 300)
                label = QLabel("Motion Visualizer\n(Module not available)")
                layout = QVBoxLayout()
                layout.addWidget(label)
                self.setLayout(layout)
            
            def load_motion(self, *args):
                pass
        
        class RobotMotionExecutor:
            def __init__(self):
                pass
            def initialize(self):
                pass
            def connect_robot(self):
                return True
            def disconnect_robot(self):
                pass
            def load_motion_yaml(self, *args):
                return True
            def execute_motion(self, *args):
                return True
            def stop_motion(self):
                pass
            def set_gripper(self, *args):
                pass
            def shutdown(self):
                pass

class SignalEmitter(QObject):
    """Signal emitter for thread-safe GUI updates"""
    update_rgb_signal = pyqtSignal(np.ndarray)
    update_depth_signal = pyqtSignal(np.ndarray)
    update_fps_signal = pyqtSignal(str)
    update_point_count_signal = pyqtSignal(int)
    motion_generation_complete = pyqtSignal(str, str)
    robot_status_update = pyqtSignal(str)
    scan_complete = pyqtSignal(str, list)
    scan_error = pyqtSignal(str)
    scan_progress = pyqtSignal(str)

class ScanThread(QThread):
    """Thread for running surface detection scan with improved responsiveness"""
    progress = pyqtSignal(str)
    finished = pyqtSignal(str, list)  # output_dir, json_files
    error = pyqtSignal(str)
    
    def __init__(self, image_path, output_dir, script_path=None):
        super().__init__()
        self.image_path = image_path
        self.output_dir = output_dir
        self.script_path = script_path or "sd.py"
        self.process = None
        self.stop_requested = False
        
    def run(self):
        try:
            # 1. 'detection_dir' 변수를 정의합니다. (기존 코드)
            timestamp = datetime.now().strftime("%H%M%S")
            detection_dir = os.path.join(self.output_dir, f"detection_{timestamp}") # <--- 'base_output_dir'을 'output_dir'로 수정
            os.makedirs(detection_dir, exist_ok=True)
            
            # ---▼▼▼▼▼ [이 한 줄만 추가하세요!] ▼▼▼▼▼---
            # 상대 경로를 절대 경로로 변환하여 혼선을 방지합니다.
            detection_dir = os.path.abspath(detection_dir)
            # ---▲▲▲▲▲ [여기까지] ▲▲▲▲▲---

            self.progress.emit(f"Starting surface detection in {detection_dir}")
            
            # 2. 이미지 유효성을 검사합니다. (기존 코드)
            if not os.path.exists(self.image_path):
                raise Exception(f"Image file not found: {self.image_path}")
            
            import cv2
            img = cv2.imread(self.image_path)
            if img is None:
                raise Exception(f"Cannot read image file: {self.image_path}")
            h, w = img.shape[:2]
            self.progress.emit(f"Image size: {w}x{h}")
            
            # 3. 사용할 장치(device)를 결정합니다.
            import torch
            device = "cuda" if torch.cuda.is_available() else "cpu"
            self.progress.emit(f"Surface detection will run on device: {device.upper()}")
            
            # 4. 모든 변수가 준비된 후, 명령어를 생성합니다.
            cmd = [
                sys.executable,
                self.script_path,
                self.image_path,
                "--output_dir", detection_dir,
                "--grid_size", "10.0",
                "--device", device
            ]
            
            # ---▲▲▲▲▲ [수정된 부분 종료] ▲▲▲▲▲---
            
            self.progress.emit(f"Running command: {' '.join(cmd)}")
            
            # Run the surface detection script
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
                universal_newlines=True,
                cwd=os.path.dirname(self.script_path) if os.path.dirname(self.script_path) else "."
            )
            
            # Monitor process with timeout
            timeout_seconds = 300  # 5 minutes timeout
            start_time = time.time()
            
            # Read output in a non-blocking way
            import select
            
            while self.process.poll() is None:
                if self.stop_requested:
                    self.progress.emit("Scan cancelled by user")
                    self.process.terminate()
                    self.process.wait(timeout=5)
                    if self.process.poll() is None:
                        self.process.kill()
                    return
                
                # Check timeout
                if time.time() - start_time > timeout_seconds:
                    self.progress.emit("Scan timeout - terminating process")
                    self.process.terminate()
                    self.process.wait(timeout=5)
                    if self.process.poll() is None:
                        self.process.kill()
                    raise Exception("Surface detection timed out after 5 minutes")
                
                # Read available output
                try:
                    # Use select on Unix systems, or implement polling on Windows
                    if hasattr(select, 'select'):
                        ready, _, _ = select.select([self.process.stdout], [], [], 0.1)
                        if ready:
                            line = self.process.stdout.readline()
                            if line:
                                self.progress.emit(line.strip())
                    else:
                        # Fallback for Windows
                        line = self.process.stdout.readline()
                        if line:
                            self.progress.emit(line.strip())
                        else:
                            time.sleep(0.1)
                except:
                    time.sleep(0.1)
                
                # Allow other threads to run
                time.sleep(0.01)
            
            # Wait for process to complete
            self.process.wait()
            
            if self.process.returncode != 0:
                error_output = self.process.stderr.read()
                raise Exception(f"Surface detection failed (exit code: {self.process.returncode}): {error_output}")
            
            # Find generated JSON files
            json_files = []
            
            # Search in multiple possible locations
            search_dirs = [
                os.path.join(detection_dir, "normal_vectors"),
                detection_dir,
                os.path.join(detection_dir, "results"),
            ]
            
            for search_dir in search_dirs:
                if os.path.exists(search_dir):
                    for root, dirs, files in os.walk(search_dir):
                        for file in files:
                            if file.endswith(('_bt_face_data.json', '_surface_data.json', '.json')):
                                json_files.append(os.path.join(root, file))
            
            self.progress.emit(f"Detection completed. Found {len(json_files)} result files.")
            
            # Also check for any generated images as indicators of success
            image_files = []
            for root, dirs, files in os.walk(detection_dir):
                for file in files:
                    if file.endswith(('.png', '.jpg', '.jpeg')):
                        image_files.append(os.path.join(root, file))
            
            self.progress.emit(f"Generated {len(image_files)} result images.")
            
            if len(json_files) == 0 and len(image_files) == 0:
                self.progress.emit("Warning: No output files found. Check if surface detection completed successfully.")
            
            self.finished.emit(detection_dir, json_files)
            
        except Exception as e:
            self.error.emit(str(e))
            import traceback
            traceback.print_exc()
        finally:
            if self.process and self.process.poll() is None:
                try:
                    self.process.terminate()
                    self.process.wait(timeout=5)
                    if self.process.poll() is None:
                        self.process.kill()
                except:
                    pass

    def stop(self):
        """Stop the scan process"""
        self.stop_requested = True
        if self.process and self.process.poll() is None:
            try:
                self.process.terminate()
            except:
                pass

class MotionGenerationThread(QThread):
    """Thread for motion generation"""
    finished = pyqtSignal(str, str)  # yaml_file, json_file
    error = pyqtSignal(str)
    
    def __init__(self, surface_files, output_dir, generator):
        super().__init__()
        self.surface_files = surface_files
        self.output_dir = output_dir
        self.generator = generator
        
    def run(self):
        try:
            yaml_file, json_file = self.generator.generate_motion_from_surfaces(
                self.surface_files,
                self.output_dir
            )
            self.finished.emit(yaml_file, json_file)
        except Exception as e:
            self.error.emit(str(e))

class RealSenseViewerV4(QMainWindow):
    """Enhanced PyQt5 GUI with surface detection integration"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RealSense D455 Vision-Based Robot Control System V4")
        self.setGeometry(100, 100, 1800, 1000)
        
        # Create base output directory
        self.base_output_dir = self.create_dated_directory()
        
        # Signal emitter
        self.signal_emitter = SignalEmitter()
        self.signal_emitter.update_rgb_signal.connect(self.update_rgb_display)
        self.signal_emitter.update_depth_signal.connect(self.update_depth_display)
        self.signal_emitter.update_fps_signal.connect(self.update_fps_display)
        self.signal_emitter.update_point_count_signal.connect(self.update_point_count_display)
        self.signal_emitter.motion_generation_complete.connect(self.on_motion_generated)
        self.signal_emitter.robot_status_update.connect(self.update_robot_status)
        self.signal_emitter.scan_complete.connect(self.on_scan_complete)
        self.signal_emitter.scan_error.connect(self.on_scan_error)
        self.signal_emitter.scan_progress.connect(self.update_scan_progress)
        
        # ROS2 Node
        self.ros_node = None
        
        # Current data
        self.current_rgb_image = None
        self.current_depth_image = None
        self.last_captured_image = None
        self.detection_results = []
        
        # Motion and robot control
        self.motion_generator = MotionGenerator()
        self.robot_executor = RobotMotionExecutor()
        self.current_yaml_file = None
        self.current_json_file = None
        self.surface_files = []
        
        # Motion execution tab 관련 속성들 초기화
        self.yaml_file_label = None
        self.motion_info_text = None
        self.method_individual = None
        self.method_pb_add = None
        self.method_itpl = None
        self.acceleration_spinbox = None
        self.blend_type_combo = None
        self.robot_status_label = None
        self.connect_robot_btn = None
        self.disconnect_robot_btn = None
        self.execute_motion_btn = None
        self.stop_motion_btn = None
        self.emergency_stop_btn = None
        self.execution_log = None
        self.load_yaml_btn = None
        self.robot_executor = None
        self.robot_executor_initialized = False
        
        # Camera tab 관련 속성들 초기화
        self.rgb_label = None
        self.depth_label = None
        self.fps_label = None
        self.point_count_label = None
        self.skip_spinner = None
        self.limit_spinner = None
        self.capture_btn = None
        self.save_rgb_btn = None
        self.save_depth_btn = None
        self.save_pc_btn = None
        self.save_all_btn = None
        
        # Detection tab 관련 속성들 초기화
        self.captured_image_label = None
        self.use_captured_btn = None
        self.load_image_btn = None
        self.selected_image_label = None
        self.start_scan_btn = None
        self.scan_progress_bar = None
        self.scan_output_text = None
        self.detection_image_label = None
        self.detection_json_list = None
        self.copy_to_test_btn = None
        
        # Motion tab 관련 속성들 초기화
        self.surface_file_list = None
        self.add_surface_btn = None
        self.remove_surface_btn = None
        self.clear_surfaces_btn = None
        self.use_detection_btn = None
        self.approach_dist_spin = None
        self.path_width_spin = None
        self.speed_spin = None
        self.generate_motion_btn = None
        self.motion_visualizer = None
        self.generated_files_text = None
        
        if not hasattr(self, 'use_generated_btn'):
            self.use_generated_btn = None
        
        # Setup UI
        self.setup_ui()
        
        # Start ROS2 in separate thread
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
    def initialize_robot_executor_safe(self):
        """안전하게 robot executor 초기화"""
        try:
            # 이미 초기화되어 있는지 확인
            if hasattr(self, 'robot_executor_initialized') and self.robot_executor_initialized:
                print("Robot executor already initialized")
                return True
            
            print("Creating RobotMotionExecutor...")
            
            # Import 시도
            try:
                from realsense_robot_control.robot_controller import RobotMotionExecutor
                self.robot_executor = RobotMotionExecutor()
            except ImportError:
                # 패키지 import 실패 시 상대 import 시도
                try:
                    from .robot_controller import RobotMotionExecutor
                    self.robot_executor = RobotMotionExecutor()
                except ImportError:
                    print("Robot controller module not found, creating dummy executor")
                    # 더미 생성
                    self.robot_executor = type('obj', (object,), {
                        'initialize': lambda: None,
                        'connect_robot': lambda: False,
                        'disconnect_robot': lambda: None,
                        'load_motion_yaml': lambda x: True,
                        'execute_motion': lambda x=None: False,
                        'stop_motion': lambda: None,
                        'set_gripper': lambda x: None,
                        'shutdown': lambda: None
                    })()
            
            print("Initializing RobotMotionExecutor...")
            self.robot_executor.initialize()
            
            self.robot_executor_initialized = True
            print("Robot executor initialized successfully")
            return True
            
        except Exception as e:
            print(f"Failed to initialize robot executor: {e}")
            import traceback
            traceback.print_exc()
            self.robot_executor = None
            self.robot_executor_initialized = False
            return False
    
    def create_dated_directory(self):
        """Create directory with today's date"""
        date_str = datetime.now().strftime("%Y%m%d")
        base_dir = os.path.join("output", date_str)
        os.makedirs(base_dir, exist_ok=True)
        return base_dir
        
    def setup_ui(self):
        """Setup the enhanced user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Create tab widget
        self.tab_widget = QTabWidget()
        
        # Tab 1: Camera View
        self.tab_widget.addTab(self.create_camera_tab(), "Camera")
        
        # Tab 2: Surface Detection  
        self.tab_widget.addTab(self.create_detection_tab(), "Surface Detection")
        
        # Tab 3: Motion Generation
        self.tab_widget.addTab(self.create_motion_tab(), "Motion Generation")
        
        # Tab 4: Motion Execution
        self.tab_widget.addTab(self.create_motion_execution_tab(), "Motion Execution")
        
        # Add tab widget to main layout
        main_layout.addWidget(self.tab_widget)
        
        # Status bar
        self.statusBar().showMessage("Ready")
        
    def create_camera_tab(self):
        """Create camera view tab (1단계)"""
        tab = QWidget()
        layout = QVBoxLayout()
        
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
        layout.addLayout(control_layout)
        
        # Display area
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
        
        layout.addLayout(image_layout)
        
        # Control buttons
        button_layout = QHBoxLayout()
        
        self.capture_btn = QPushButton("Capture Current Frame")
        self.capture_btn.clicked.connect(self.capture_current_frame)
        self.capture_btn.setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; }")
        
        self.save_rgb_btn = QPushButton("Save RGB Image")
        self.save_rgb_btn.clicked.connect(self.save_rgb_image)
        
        self.save_depth_btn = QPushButton("Save Depth Image")
        self.save_depth_btn.clicked.connect(self.save_depth_image)
        
        self.save_pc_btn = QPushButton("Save Point Cloud")
        self.save_pc_btn.clicked.connect(self.save_pointcloud)
        
        self.save_all_btn = QPushButton("Save All")
        self.save_all_btn.clicked.connect(self.save_all)
        
        button_layout.addWidget(self.capture_btn)
        button_layout.addWidget(self.save_rgb_btn)
        button_layout.addWidget(self.save_depth_btn)
        button_layout.addWidget(self.save_pc_btn)
        button_layout.addWidget(self.save_all_btn)
        
        layout.addLayout(button_layout)
        
        tab.setLayout(layout)
        return tab

    def create_detection_tab(self):
        """Create surface detection tab (2단계)"""
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Image selection group
        image_group = QGroupBox("Image Selection")
        image_layout = QVBoxLayout()
        
        # Current captured image display
        self.captured_image_label = QLabel("No image captured")
        self.captured_image_label.setMinimumSize(320, 240)
        self.captured_image_label.setMaximumSize(640, 480)
        self.captured_image_label.setScaledContents(True)
        self.captured_image_label.setStyleSheet("border: 1px solid black;")
        image_layout.addWidget(self.captured_image_label)
        
        # Image selection buttons
        img_btn_layout = QHBoxLayout()
        
        self.use_captured_btn = QPushButton("Use Captured Image")
        self.use_captured_btn.clicked.connect(self.use_captured_image)
        self.use_captured_btn.setEnabled(False)
        
        self.load_image_btn = QPushButton("Load Image from File")
        self.load_image_btn.clicked.connect(self.load_image_file)
        
        img_btn_layout.addWidget(self.use_captured_btn)
        img_btn_layout.addWidget(self.load_image_btn)
        image_layout.addLayout(img_btn_layout)
        
        # Selected image path
        self.selected_image_label = QLabel("No image selected")
        self.selected_image_label.setStyleSheet("padding: 5px; background-color: #f0f0f0;")
        image_layout.addWidget(self.selected_image_label)
        
        image_group.setLayout(image_layout)
        layout.addWidget(image_group)
        
        # Detection control group
        detection_group = QGroupBox("Surface Detection")
        detection_layout = QVBoxLayout()
        
        # Start scan button
        self.start_scan_btn = QPushButton("Start Scan")
        self.start_scan_btn.clicked.connect(self.start_surface_scan)
        self.start_scan_btn.setEnabled(False)
        self.start_scan_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }"
                                        "QPushButton:hover { background-color: #45a049; }")
        detection_layout.addWidget(self.start_scan_btn)
        
        # Progress bar
        self.scan_progress_bar = QProgressBar()
        self.scan_progress_bar.setVisible(False)
        detection_layout.addWidget(self.scan_progress_bar)
        
        # Scan output
        self.scan_output_text = QTextEdit()
        self.scan_output_text.setReadOnly(True)
        self.scan_output_text.setMaximumHeight(200)
        detection_layout.addWidget(self.scan_output_text)
        
        detection_group.setLayout(detection_layout)
        layout.addWidget(detection_group)
        
        # Detection results group
        results_group = QGroupBox("Detection Results")
        results_layout = QVBoxLayout()
        
        # Results display area with horizontal layout
        results_display_layout = QHBoxLayout()
        
        # Detection result image
        self.detection_image_label = QLabel("No detection results")
        self.detection_image_label.setMinimumSize(320, 240)
        self.detection_image_label.setMaximumSize(640, 480)
        self.detection_image_label.setScaledContents(True)
        self.detection_image_label.setStyleSheet("border: 1px solid black;")
        results_display_layout.addWidget(self.detection_image_label)
        
        # JSON files list
        json_list_layout = QVBoxLayout()
        json_list_label = QLabel("Detected Surface Files:")
        json_list_layout.addWidget(json_list_label)
        
        self.detection_json_list = QListWidget()
        self.detection_json_list.setMaximumHeight(200)
        json_list_layout.addWidget(self.detection_json_list)
        
        # Copy to test data button
        self.copy_to_test_btn = QPushButton("Copy to Test Data")
        self.copy_to_test_btn.clicked.connect(self.copy_to_test_data)
        self.copy_to_test_btn.setEnabled(False)
        json_list_layout.addWidget(self.copy_to_test_btn)
        
        results_display_layout.addLayout(json_list_layout)
        results_layout.addLayout(results_display_layout)
        
        results_group.setLayout(results_layout)
        layout.addWidget(results_group)
        
        layout.addStretch()
        tab.setLayout(layout)
        return tab

    def create_motion_tab(self):
        """Create motion generation tab (3단계)"""
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Surface data selection
        surface_group = QGroupBox("Surface Data Selection")
        surface_layout = QVBoxLayout()
        
        # Info
        info_label = QLabel("Select surface data files for motion generation.\n"
                        "At least 2 surface files are required.")
        surface_layout.addWidget(info_label)
        
        # File list
        self.surface_file_list = QListWidget()
        self.surface_file_list.setMaximumHeight(150)
        surface_layout.addWidget(self.surface_file_list)
        
        # File buttons
        file_btn_layout = QHBoxLayout()
        
        self.add_surface_btn = QPushButton("Add Surface File")
        self.add_surface_btn.clicked.connect(self.add_surface_file)
        
        self.remove_surface_btn = QPushButton("Remove Selected")
        self.remove_surface_btn.clicked.connect(self.remove_surface_file)
        
        self.clear_surfaces_btn = QPushButton("Clear All")
        self.clear_surfaces_btn.clicked.connect(self.clear_surface_files)
        
        self.use_detection_btn = QPushButton("Use Detection Results")
        self.use_detection_btn.clicked.connect(self.use_detection_results)
        self.use_detection_btn.setEnabled(False)
        
        file_btn_layout.addWidget(self.add_surface_btn)
        file_btn_layout.addWidget(self.remove_surface_btn)
        file_btn_layout.addWidget(self.clear_surfaces_btn)
        file_btn_layout.addWidget(self.use_detection_btn)
        surface_layout.addLayout(file_btn_layout)
        
        surface_group.setLayout(surface_layout)
        layout.addWidget(surface_group)
        
        # Motion parameters
        param_group = QGroupBox("Motion Parameters")
        param_layout = QVBoxLayout()
        
        # Parameter inputs
        param_grid = QHBoxLayout()
        
        param_grid.addWidget(QLabel("Approach Distance (mm):"))
        self.approach_dist_spin = QSpinBox()
        self.approach_dist_spin.setRange(10, 100)
        self.approach_dist_spin.setValue(50)
        param_grid.addWidget(self.approach_dist_spin)
        
        param_grid.addWidget(QLabel("Path Width (mm):"))
        self.path_width_spin = QSpinBox()
        self.path_width_spin.setRange(50, 200)
        self.path_width_spin.setValue(100)
        param_grid.addWidget(self.path_width_spin)
        
        param_grid.addWidget(QLabel("Speed (mm/s):"))
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(10, 250)
        self.speed_spin.setValue(100)
        param_grid.addWidget(self.speed_spin)
        
        param_layout.addLayout(param_grid)
        
        # Generate button
        self.generate_motion_btn = QPushButton("Generate Motion (ㄷ-shaped path)")
        self.generate_motion_btn.clicked.connect(self.generate_motion)
        self.generate_motion_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }"
                                            "QPushButton:hover { background-color: #45a049; }")
        param_layout.addWidget(self.generate_motion_btn)
        
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        # Motion visualization
        vis_group = QGroupBox("Motion Visualization")
        vis_layout = QVBoxLayout()
        
        self.motion_visualizer = MotionVisualizerWidget()
        vis_layout.addWidget(self.motion_visualizer)
        
        vis_group.setLayout(vis_layout)
        layout.addWidget(vis_group)
        
        # Generated files info
        files_group = QGroupBox("Generated Files")
        files_layout = QVBoxLayout()
        
        self.generated_files_text = QTextEdit()
        self.generated_files_text.setReadOnly(True)
        self.generated_files_text.setMaximumHeight(100)
        files_layout.addWidget(self.generated_files_text)
        
        files_group.setLayout(files_layout)
        layout.addWidget(files_group)
        
        tab.setLayout(layout)
        return tab
        
    def setup_camera_tab(self, parent):
        """Setup camera view tab"""
        layout = QVBoxLayout()
        parent.setLayout(layout)
        
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
        layout.addLayout(control_layout)
        
        # Display area
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
        
        layout.addLayout(image_layout)
        
        # Control buttons
        button_layout = QHBoxLayout()
        
        self.capture_btn = QPushButton("Capture Current Frame")
        self.capture_btn.clicked.connect(self.capture_current_frame)
        self.capture_btn.setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; }")
        
        self.save_rgb_btn = QPushButton("Save RGB Image")
        self.save_rgb_btn.clicked.connect(self.save_rgb_image)
        
        self.save_depth_btn = QPushButton("Save Depth Image")
        self.save_depth_btn.clicked.connect(self.save_depth_image)
        
        self.save_pc_btn = QPushButton("Save Point Cloud")
        self.save_pc_btn.clicked.connect(self.save_pointcloud)
        
        self.save_all_btn = QPushButton("Save All")
        self.save_all_btn.clicked.connect(self.save_all)
        
        button_layout.addWidget(self.capture_btn)
        button_layout.addWidget(self.save_rgb_btn)
        button_layout.addWidget(self.save_depth_btn)
        button_layout.addWidget(self.save_pc_btn)
        button_layout.addWidget(self.save_all_btn)
        
        layout.addLayout(button_layout)
    
    def setup_detection_tab(self, parent):
        """Setup surface detection tab (2단계)"""
        layout = QVBoxLayout()
        parent.setLayout(layout)
        
        # Image selection group
        image_group = QGroupBox("Image Selection")
        image_layout = QVBoxLayout()
        
        # Current captured image display
        self.captured_image_label = QLabel("No image captured")
        self.captured_image_label.setMinimumSize(320, 240)
        self.captured_image_label.setMaximumSize(640, 480)
        self.captured_image_label.setScaledContents(True)
        self.captured_image_label.setStyleSheet("border: 1px solid black;")
        image_layout.addWidget(self.captured_image_label)
        
        # Image selection buttons
        img_btn_layout = QHBoxLayout()
        
        self.use_captured_btn = QPushButton("Use Captured Image")
        self.use_captured_btn.clicked.connect(self.use_captured_image)
        self.use_captured_btn.setEnabled(False)
        
        self.load_image_btn = QPushButton("Load Image from File")
        self.load_image_btn.clicked.connect(self.load_image_file)
        
        img_btn_layout.addWidget(self.use_captured_btn)
        img_btn_layout.addWidget(self.load_image_btn)
        image_layout.addLayout(img_btn_layout)
        
        # Selected image path
        self.selected_image_label = QLabel("No image selected")
        self.selected_image_label.setStyleSheet("padding: 5px; background-color: #f0f0f0;")
        image_layout.addWidget(self.selected_image_label)
        
        image_group.setLayout(image_layout)
        layout.addWidget(image_group)
        
        # Detection control group
        detection_group = QGroupBox("Surface Detection")
        detection_layout = QVBoxLayout()
        
        # Start scan button
        self.start_scan_btn = QPushButton("Start Scan")
        self.start_scan_btn.clicked.connect(self.start_surface_scan)
        self.start_scan_btn.setEnabled(False)
        self.start_scan_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }"
                                         "QPushButton:hover { background-color: #45a049; }")
        detection_layout.addWidget(self.start_scan_btn)
        
        # Progress bar
        self.scan_progress_bar = QProgressBar()
        self.scan_progress_bar.setVisible(False)
        detection_layout.addWidget(self.scan_progress_bar)
        
        # Scan output
        self.scan_output_text = QTextEdit()
        self.scan_output_text.setReadOnly(True)
        self.scan_output_text.setMaximumHeight(200)
        detection_layout.addWidget(self.scan_output_text)
        
        detection_group.setLayout(detection_layout)
        layout.addWidget(detection_group)
        
        # Detection results group
        results_group = QGroupBox("Detection Results")
        results_layout = QVBoxLayout()
        
        # Results display area with horizontal layout
        results_display_layout = QHBoxLayout()
        
        # Detection result image
        self.detection_image_label = QLabel("No detection results")
        self.detection_image_label.setMinimumSize(320, 240)
        self.detection_image_label.setMaximumSize(640, 480)
        self.detection_image_label.setScaledContents(True)
        self.detection_image_label.setStyleSheet("border: 1px solid black;")
        results_display_layout.addWidget(self.detection_image_label)
        
        # JSON files list
        json_list_layout = QVBoxLayout()
        json_list_label = QLabel("Detected Surface Files:")
        json_list_layout.addWidget(json_list_label)
        
        self.detection_json_list = QListWidget()
        self.detection_json_list.setMaximumHeight(200)
        json_list_layout.addWidget(self.detection_json_list)
        
        # Copy to test data button
        self.copy_to_test_btn = QPushButton("Copy to Test Data")
        self.copy_to_test_btn.clicked.connect(self.copy_to_test_data)
        self.copy_to_test_btn.setEnabled(False)
        json_list_layout.addWidget(self.copy_to_test_btn)
        
        results_display_layout.addLayout(json_list_layout)
        results_layout.addLayout(results_display_layout)
        
        results_group.setLayout(results_layout)
        layout.addWidget(results_group)
        
        layout.addStretch()
    
    def setup_motion_generation_tab(self, parent):
        """Setup motion generation tab (3단계)"""
        layout = QVBoxLayout()
        parent.setLayout(layout)
        
        # Surface data selection
        surface_group = QGroupBox("Surface Data Selection")
        surface_layout = QVBoxLayout()
        
        # Info
        info_label = QLabel("Select surface data files for motion generation.\n"
                           "At least 2 surface files are required.")
        surface_layout.addWidget(info_label)
        
        # File list
        self.surface_file_list = QListWidget()
        self.surface_file_list.setMaximumHeight(150)
        surface_layout.addWidget(self.surface_file_list)
        
        # File buttons
        file_btn_layout = QHBoxLayout()
        
        self.add_surface_btn = QPushButton("Add Surface File")
        self.add_surface_btn.clicked.connect(self.add_surface_file)
        
        self.remove_surface_btn = QPushButton("Remove Selected")
        self.remove_surface_btn.clicked.connect(self.remove_surface_file)
        
        self.clear_surfaces_btn = QPushButton("Clear All")
        self.clear_surfaces_btn.clicked.connect(self.clear_surface_files)
        
        self.use_detection_btn = QPushButton("Use Detection Results")
        self.use_detection_btn.clicked.connect(self.use_detection_results)
        self.use_detection_btn.setEnabled(False)
        
        file_btn_layout.addWidget(self.add_surface_btn)
        file_btn_layout.addWidget(self.remove_surface_btn)
        file_btn_layout.addWidget(self.clear_surfaces_btn)
        file_btn_layout.addWidget(self.use_detection_btn)
        surface_layout.addLayout(file_btn_layout)
        
        surface_group.setLayout(surface_layout)
        layout.addWidget(surface_group)
        
        # Motion parameters
        param_group = QGroupBox("Motion Parameters")
        param_layout = QVBoxLayout()
        
        # Parameter inputs
        param_grid = QHBoxLayout()
        
        param_grid.addWidget(QLabel("Approach Distance (mm):"))
        self.approach_dist_spin = QSpinBox()
        self.approach_dist_spin.setRange(10, 100)
        self.approach_dist_spin.setValue(50)
        param_grid.addWidget(self.approach_dist_spin)
        
        param_grid.addWidget(QLabel("Path Width (mm):"))
        self.path_width_spin = QSpinBox()
        self.path_width_spin.setRange(50, 200)
        self.path_width_spin.setValue(100)
        param_grid.addWidget(self.path_width_spin)
        
        param_grid.addWidget(QLabel("Speed (mm/s):"))
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(10, 250)
        self.speed_spin.setValue(100)
        param_grid.addWidget(self.speed_spin)
        
        param_layout.addLayout(param_grid)
        
        # Generate button
        self.generate_motion_btn = QPushButton("Generate Motion (ㄷ-shaped path)")
        self.generate_motion_btn.clicked.connect(self.generate_motion)
        self.generate_motion_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }"
                                              "QPushButton:hover { background-color: #45a049; }")
        param_layout.addWidget(self.generate_motion_btn)
        
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        # Motion visualization
        vis_group = QGroupBox("Motion Visualization")
        vis_layout = QVBoxLayout()
        
        self.motion_visualizer = MotionVisualizerWidget()
        vis_layout.addWidget(self.motion_visualizer)
        
        vis_group.setLayout(vis_layout)
        layout.addWidget(vis_group)
        
        # Generated files info
        files_group = QGroupBox("Generated Files")
        files_layout = QVBoxLayout()
        
        self.generated_files_text = QTextEdit()
        self.generated_files_text.setReadOnly(True)
        self.generated_files_text.setMaximumHeight(100)
        files_layout.addWidget(self.generated_files_text)
        
        files_group.setLayout(files_layout)
        layout.addWidget(files_group)
    
    def setup_motion_execution_tab(self, parent):
        """Setup motion execution tab (4단계)"""
        layout = QVBoxLayout()
        parent.setLayout(layout)
        
        # Motion file selection
        file_group = QGroupBox("Motion File Selection")
        file_layout = QVBoxLayout()
        
        file_btn_layout = QHBoxLayout()
        
        self.load_yaml_btn = QPushButton("Load Motion YAML File")
        self.load_yaml_btn.clicked.connect(self.load_motion_yaml)
        
        self.use_generated_btn = QPushButton("Use Last Generated Motion")
        self.use_generated_btn.clicked.connect(self.use_generated_motion)
        self.use_generated_btn.setEnabled(False)
        
        file_btn_layout.addWidget(self.load_yaml_btn)
        file_btn_layout.addWidget(self.use_generated_btn)
        file_layout.addLayout(file_btn_layout)
        
        # Loaded file info
        self.loaded_yaml_label = QLabel("No motion file loaded")
        self.loaded_yaml_label.setStyleSheet("padding: 5px; background-color: #f0f0f0;")
        file_layout.addWidget(self.loaded_yaml_label)
        
        # Motion info display
        self.motion_info_text = QTextEdit()
        self.motion_info_text.setReadOnly(True)
        self.motion_info_text.setMaximumHeight(150)
        file_layout.addWidget(self.motion_info_text)
        
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        # Robot connection
        connection_group = QGroupBox("Robot Connection")
        connection_layout = QVBoxLayout()
        
        conn_btn_layout = QHBoxLayout()
        self.connect_robot_btn = QPushButton("Connect to Robot")
        self.connect_robot_btn.clicked.connect(self.connect_robot)
        self.connect_robot_btn.setStyleSheet("QPushButton { background-color: #2196F3; color: white; }")
        
        self.disconnect_robot_btn = QPushButton("Disconnect")
        self.disconnect_robot_btn.clicked.connect(self.disconnect_robot)
        self.disconnect_robot_btn.setEnabled(False)
        
        conn_btn_layout.addWidget(self.connect_robot_btn)
        conn_btn_layout.addWidget(self.disconnect_robot_btn)
        connection_layout.addLayout(conn_btn_layout)
        
        # Robot status
        self.robot_status_label = QLabel("Robot Status: Disconnected")
        self.robot_status_label.setStyleSheet("font-weight: bold; padding: 5px;")
        connection_layout.addWidget(self.robot_status_label)
        
        connection_group.setLayout(connection_layout)
        layout.addWidget(connection_group)
        
        # Motion execution
        execution_group = QGroupBox("Motion Execution")
        execution_layout = QVBoxLayout()
        
        # Execution mode
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Execution Mode:"))
        
        self.execution_mode_combo = QComboBox()
        self.execution_mode_combo.addItems(["Interpolated (Smooth)", "Point-to-Point"])
        mode_layout.addWidget(self.execution_mode_combo)
        execution_layout.addLayout(mode_layout)
        
        # Execution controls
        exec_btn_layout = QHBoxLayout()
        
        self.execute_motion_btn = QPushButton("Execute Motion")
        self.execute_motion_btn.clicked.connect(self.execute_motion)
        self.execute_motion_btn.setEnabled(False)
        self.execute_motion_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }"
                                            "QPushButton:hover { background-color: #45a049; }")
        
        self.stop_motion_btn = QPushButton("Stop Motion")
        self.stop_motion_btn.clicked.connect(self.stop_motion)
        self.stop_motion_btn.setEnabled(False)
        self.stop_motion_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; }"
                                          "QPushButton:hover { background-color: #da190b; }")
        
        exec_btn_layout.addWidget(self.execute_motion_btn)
        exec_btn_layout.addWidget(self.stop_motion_btn)
        execution_layout.addLayout(exec_btn_layout)
        
        # Gripper control
        gripper_layout = QHBoxLayout()
        gripper_layout.addWidget(QLabel("Gripper Control:"))
        
        self.gripper_open_btn = QPushButton("Open")
        self.gripper_open_btn.clicked.connect(lambda: self.control_gripper(True))
        self.gripper_open_btn.setEnabled(False)
        
        self.gripper_close_btn = QPushButton("Close")
        self.gripper_close_btn.clicked.connect(lambda: self.control_gripper(False))
        self.gripper_close_btn.setEnabled(False)
        
        gripper_layout.addWidget(self.gripper_open_btn)
        gripper_layout.addWidget(self.gripper_close_btn)
        gripper_layout.addStretch()
        execution_layout.addLayout(gripper_layout)
        
        execution_group.setLayout(execution_layout)
        layout.addWidget(execution_group)
        
        # Execution log
        log_group = QGroupBox("Execution Log")
        log_layout = QVBoxLayout()
        
        self.execution_log = QTextEdit()
        self.execution_log.setReadOnly(True)
        log_layout.addWidget(self.execution_log)
        
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
    
    # Surface Detection Methods (2단계)
    def capture_current_frame(self):
        """Capture current camera frame"""
        if self.current_rgb_image is not None:
            # Save captured image
            timestamp = datetime.now().strftime("%H%M%S")
            capture_path = os.path.join(self.base_output_dir, f"capture_{timestamp}.png")
            cv2.imwrite(capture_path, self.current_rgb_image)
            
            self.last_captured_image = capture_path
            
            # Update UI
            pixmap = QPixmap(capture_path)
            scaled_pixmap = pixmap.scaled(self.captured_image_label.size(), 
                                         Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.captured_image_label.setPixmap(scaled_pixmap)
            
            self.use_captured_btn.setEnabled(True)
            self.statusBar().showMessage(f"Frame captured: {capture_path}")
        else:
            QMessageBox.warning(self, "Warning", "No camera image available!")
    
    def use_captured_image(self):
        """Use the captured image for detection"""
        if self.last_captured_image and os.path.exists(self.last_captured_image):
            self.selected_image_label.setText(f"Selected: {os.path.basename(self.last_captured_image)}")
            self.start_scan_btn.setEnabled(True)
        else:
            QMessageBox.warning(self, "Warning", "No captured image available!")
    
    def load_image_file(self):
        """Load an image file for detection (수정된 버전)"""
        try:
            if PYQT5_AVAILABLE:
                file_path, _ = QFileDialog.getOpenFileName(
                    self, "Load Image File", "", "Image Files (*.png *.jpg *.jpeg)"
                )
            else:
                file_path = self.get_file_via_terminal("Load Image File", "*.png")
        except Exception as e:
            print(f"File dialog error: {e}")
            file_path = self.get_file_via_terminal("Load Image File", "*.png")
        
        if file_path and os.path.exists(file_path):
            self.last_captured_image = file_path
            if hasattr(self, 'selected_image_label') and self.selected_image_label:
                self.selected_image_label.setText(f"Selected: {os.path.basename(file_path)}")
            
            # Show preview
            try:
                if PYQT5_AVAILABLE and hasattr(self, 'captured_image_label') and self.captured_image_label:
                    pixmap = QPixmap(file_path)
                    scaled_pixmap = pixmap.scaled(self.captured_image_label.size(), 
                                                Qt.KeepAspectRatio, Qt.SmoothTransformation)
                    self.captured_image_label.setPixmap(scaled_pixmap)
            except Exception as e:
                print(f"Error showing preview: {e}")
            
            if hasattr(self, 'start_scan_btn') and self.start_scan_btn:
                self.start_scan_btn.setEnabled(True)
            
            print(f"Image loaded: {file_path}")

    
    def start_surface_scan(self):
        """Start surface detection scan with improved responsiveness"""
        if not self.last_captured_image or not os.path.exists(self.last_captured_image):
            safe_message_box_warning(self, "Warning", "No image selected for scanning!")
            return
        
        # 기본 검증들...
        script_paths = [
            "sd.py",
            os.path.join(os.path.dirname(__file__), "sd.py"),
            os.path.join(os.path.dirname(__file__), "..", "scripts", "sd.py"),
            "/root/slc_ws/src/realsense_robot_control/sd.py",
            "/root/slc_ws/install/realsense_robot_control/share/realsense_robot_control/scripts/sd.py"
        ]
        
        sd_script_path = None
        for path in script_paths:
            if os.path.exists(path):
                sd_script_path = path
                break
        
        if not sd_script_path:
            safe_message_box_error(self, "Error", "Surface detection script (sd.py) not found!")
            return
        
        # UI 업데이트
        self.start_scan_btn.setEnabled(False)
        self.start_scan_btn.setText("Scanning...")
        self.scan_progress_bar.setVisible(True)
        self.scan_progress_bar.setRange(0, 0)  # Indeterminate progress
        self.scan_output_text.clear()
        
        # Add stop button
        if not hasattr(self, 'stop_scan_btn'):
            self.stop_scan_btn = QPushButton("Stop Scan")
            self.stop_scan_btn.clicked.connect(self.stop_surface_scan)
            # Add to layout (you'll need to adjust this based on your UI layout)
        
        self.stop_scan_btn.setVisible(True)
        self.stop_scan_btn.setEnabled(True)
        
        # Start scan in separate thread with improved monitoring
        self.scan_thread = ScanThread(self.last_captured_image, self.base_output_dir, sd_script_path)
        self.scan_thread.progress.connect(self.update_scan_progress)
        self.scan_thread.finished.connect(self.on_scan_complete)
        self.scan_thread.error.connect(self.on_scan_error)
        
        # Add progress timer for GUI responsiveness
        self.scan_progress_timer = QTimer()
        self.scan_progress_timer.timeout.connect(self.update_scan_ui)
        self.scan_progress_timer.start(100)  # Update every 100ms
        
        self.statusBar().showMessage("Starting surface detection...")
        self.scan_thread.start()

    def stop_surface_scan(self):
        """Stop the current scan"""
        if hasattr(self, 'scan_thread') and self.scan_thread.isRunning():
            self.update_scan_progress("Stopping scan...")
            self.scan_thread.stop()
            self.scan_thread.wait(5000)  # Wait up to 5 seconds
            
        self.scan_cleanup_ui()

    
    def update_scan_progress(self, message):
        """Update scan progress display"""
        self.scan_output_text.append(message)
        # Auto-scroll to bottom
        scrollbar = self.scan_output_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def update_scan_ui(self):
        """Update UI during scan to maintain responsiveness"""
        if hasattr(self, 'scan_thread') and not self.scan_thread.isRunning():
            self.scan_cleanup_ui()
        
        # Process events to keep UI responsive
        QApplication.processEvents()

    def scan_cleanup_ui(self):
        """Clean up UI after scan completion or cancellation"""
        if hasattr(self, 'scan_progress_timer'):
            self.scan_progress_timer.stop()
        
        self.start_scan_btn.setEnabled(True)
        self.start_scan_btn.setText("Start Scan")
        self.scan_progress_bar.setVisible(False)
        
        if hasattr(self, 'stop_scan_btn'):
            self.stop_scan_btn.setVisible(False)

    def on_scan_complete(self, output_dir, json_files):
        """Handle scan completion with UI cleanup and result display"""
        self.scan_cleanup_ui()
        
        self.detection_results = json_files
        
        # UI의 리스트 위젯에 감지된 JSON 파일 목록을 채웁니다.
        self.detection_json_list.clear()
        for json_file in json_files:
            self.detection_json_list.addItem(os.path.basename(json_file))
        
        # 버튼 활성화 로직
        if json_files:
            self.copy_to_test_btn.setEnabled(True)
            if hasattr(self, 'use_detection_btn') and self.use_detection_btn:
                self.use_detection_btn.setEnabled(True)
        
         # ---▼▼▼▼▼ [핵심 수정 부분] ▼▼▼▼▼---
        
        # 결과 이미지(selected_parts.jpg)를 찾아서 UI에 표시합니다.
        result_image_path = os.path.join(output_dir, "selected_parts.jpg")
        
        if os.path.exists(result_image_path):
            try:
                pixmap = QPixmap(result_image_path)
                # 라벨 크기에 맞게 이미지 스케일 조정 (가로세로 비율 유지)
                scaled_pixmap = pixmap.scaled(
                    self.detection_image_label.size(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                )
                self.detection_image_label.setPixmap(scaled_pixmap)
                self.update_scan_progress(f"Displaying result image: {os.path.basename(result_image_path)}")
            except Exception as e:
                error_msg = f"Error displaying result image: {e}"
                self.update_scan_progress(error_msg)
                self.detection_image_label.setText(error_msg)
        else:
            # selected_parts.jpg가 없을 경우 roi_detection.jpg를 대신 표시
            fallback_image_path = os.path.join(output_dir, "roi_detection.jpg")
            if os.path.exists(fallback_image_path):
                pixmap = QPixmap(fallback_image_path)
                scaled_pixmap = pixmap.scaled(self.detection_image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.detection_image_label.setPixmap(scaled_pixmap)
                self.update_scan_progress("Warning: 'selected_parts.jpg' not found. Displaying ROI detection image instead.")
            else:
                self.update_scan_progress("Result image not found.")
                self.detection_image_label.setText("Result image not found")

        # ---▲▲▲▲▲ [수정 종료] ▲▲▲▲▲---
        
        self.statusBar().showMessage(f"Detection complete: {len(json_files)} surfaces found")
        self.update_scan_progress(f"\nDetection completed successfully!")

    def on_scan_error(self, error_message):
        """Handle scan error with UI cleanup"""
        self.scan_cleanup_ui()
        
        safe_message_box_error(self, "Scan Error", f"Surface detection failed:\n{error_message}")
        self.statusBar().showMessage("Surface detection failed")
        self.update_scan_progress(f"\nERROR: {error_message}")
    
    def copy_to_test_data(self):
        """Copy detection results to test_data folder"""
        if not self.detection_results:
            QMessageBox.warning(self, "Warning", "No detection results to copy!")
            return
        
        # Create test_data directory
        test_data_dir = os.path.join("test_data", datetime.now().strftime("%Y%m%d_%H%M%S"))
        os.makedirs(test_data_dir, exist_ok=True)
        
        # Copy JSON files
        copied_files = []
        for json_file in self.detection_results:
            dest_file = os.path.join(test_data_dir, os.path.basename(json_file))
            shutil.copy2(json_file, dest_file)
            copied_files.append(dest_file)
        
        QMessageBox.information(self, "Success", 
                               f"Copied {len(copied_files)} files to:\n{test_data_dir}")
        self.statusBar().showMessage(f"Detection results copied to test_data")
    
    def use_detection_results(self):
        """Use detection results for motion generation"""
        if not self.detection_results:
            QMessageBox.warning(self, "Warning", "No detection results available!")
            return
        
        # Clear current surface files
        self.surface_files.clear()
        self.surface_file_list.clear()
        
        # Add detection results
        for json_file in self.detection_results:
            self.surface_files.append(json_file)
            self.surface_file_list.addItem(os.path.basename(json_file))
        
        self.statusBar().showMessage(f"Loaded {len(self.surface_files)} surface files from detection")
        
        # Switch to motion generation tab
        self.tab_widget.setCurrentIndex(2)
    
    # Motion Generation Methods (3단계)
    def add_surface_file(self):
        """Add surface data file (수정된 버전)"""
        try:
            if PYQT5_AVAILABLE:
                file_path, _ = QFileDialog.getOpenFileName(
                    self, "Add Surface Data File", "", "JSON Files (*.json)"
                )
            else:
                file_path = self.get_file_via_terminal("Add Surface Data File", "*.json")
        except Exception as e:
            print(f"File dialog error: {e}")
            file_path = self.get_file_via_terminal("Add Surface Data File", "*.json")
        
        if file_path and file_path not in self.surface_files and os.path.exists(file_path):
            self.surface_files.append(file_path)
            if hasattr(self, 'surface_file_list') and self.surface_file_list:
                self.surface_file_list.addItem(os.path.basename(file_path))
            self.statusBar().showMessage(f"Added: {os.path.basename(file_path)}")
            print(f"Surface file added: {file_path}")
    
    def remove_surface_file(self):
        """Remove selected surface file"""
        current_row = self.surface_file_list.currentRow()
        if current_row >= 0:
            self.surface_files.pop(current_row)
            self.surface_file_list.takeItem(current_row)
    
    def clear_surface_files(self):
        """Clear all surface files"""
        self.surface_files.clear()
        self.surface_file_list.clear()
    
    def generate_motion(self):
        """Generate motion from surface data (3단계)"""
        if len(self.surface_files) < 2:
            QMessageBox.warning(self, "Warning", "Please add at least 2 surface data files!")
            return
        
        # Update motion generator parameters
        self.motion_generator.approach_distance = self.approach_dist_spin.value() / 1000.0
        self.motion_generator.path_width = self.path_width_spin.value() / 1000.0
        self.motion_generator.linear_speed = self.speed_spin.value()
        
        # Create output directory with timestamp
        motion_dir = os.path.join(self.base_output_dir, f"motion_{datetime.now().strftime('%H%M%S')}")
        os.makedirs(motion_dir, exist_ok=True)
        
        # Generate motion in separate thread
        self.motion_thread = MotionGenerationThread(
            self.surface_files, motion_dir, self.motion_generator
        )
        self.motion_thread.finished.connect(self.on_motion_generated)
        self.motion_thread.error.connect(self.on_motion_error)
        
        self.generate_motion_btn.setEnabled(False)
        self.statusBar().showMessage("Generating motion...")
        
        self.motion_thread.start()
    
    def on_motion_generated(self, yaml_file, json_file):
        """Handle motion generation completion"""
        self.current_yaml_file = yaml_file
        self.current_json_file = json_file
        self.generate_motion_btn.setEnabled(True)
        
        # Update generated files info
        files_info = f"Generated Files:\n"
        files_info += f"Robot Motion (YAML): {os.path.basename(yaml_file)}\n"
        files_info += f"Visualization (JSON): {os.path.basename(json_file)}\n"
        files_info += f"Location: {os.path.dirname(yaml_file)}"
        self.generated_files_text.setText(files_info)
        
        # Load and visualize motion
        try:
            with open(json_file, 'r') as f:
                motion_data = json.load(f)
            
            self.motion_visualizer.load_motion(motion_data)
            
            # Enable use in execution tab
            self.use_generated_btn.setEnabled(True)
            
            self.statusBar().showMessage(f"Motion generated successfully")
            self.log_message(f"Motion generated with {len(motion_data['waypoints'])} waypoints")
            
        except Exception as e:
            self.on_motion_error(str(e))
    
    def on_motion_error(self, error_msg):
        """Handle motion generation error"""
        self.generate_motion_btn.setEnabled(True)
        QMessageBox.error(self, "Motion Generation Error", error_msg)
        self.statusBar().showMessage("Motion generation failed")
    
    # Motion Execution Methods (4단계)
    def load_motion_yaml(self):
        """Load motion YAML file for execution"""
        try:
            # load_yaml_for_execution 메서드가 있으면 호출
            if hasattr(self, 'load_yaml_for_execution'):
                self.load_yaml_for_execution()
            else:
                # 없으면 직접 구현
                file_path = None
                
                if PYQT5_AVAILABLE:
                    try:
                        file_path, _ = QFileDialog.getOpenFileName(
                            self, "Load Motion YAML File", 
                            getattr(self, 'base_output_dir', ''), 
                            "YAML Files (*.yaml *.yml)"
                        )
                    except Exception as e:
                        print(f"File dialog error: {e}")
                        file_path = self.get_file_via_terminal("Load Motion YAML File", "*.yaml")
                else:
                    file_path = self.get_file_via_terminal("Load Motion YAML File", "*.yaml")
                
                if file_path:
                    self.current_yaml_file = file_path
                    self.load_yaml_for_execution(file_path)
                    print(f"Motion YAML loaded: {file_path}")
                    
        except Exception as e:
            print(f"Error in load_motion_yaml: {e}")
            import traceback
            traceback.print_exc()
            safe_message_box_error(self, "Error", f"Failed to load YAML file: {str(e)}")
    
    def use_generated_motion(self):
        """Use the last generated motion for execution"""
        if self.current_yaml_file and os.path.exists(self.current_yaml_file):
            self.load_yaml_for_execution(self.current_yaml_file)
        else:
            QMessageBox.warning(self, "Warning", "No generated motion available")
    
    def load_yaml_for_execution(self, yaml_file=None):
        """Load YAML file for execution with improved error handling and PyQt5 safety"""
        try:
            # 파일 선택 로직 개선
            if yaml_file is None:
                try:
                    # PyQt5가 사용 가능한 경우
                    if PYQT5_AVAILABLE and hasattr(self, 'isVisible') and self.isVisible():
                        # 파일 다이얼로그 전에 이벤트 처리
                        QApplication.processEvents()
                        
                        yaml_file, _ = QFileDialog.getOpenFileName(
                            self, "Load Motion YAML File", 
                            self.base_output_dir if hasattr(self, 'base_output_dir') else "", 
                            "YAML files (*.yaml *.yml)"
                        )
                        
                        # 파일 다이얼로그 후 이벤트 처리
                        QApplication.processEvents()
                    else:
                        # 터미널 기반 파일 선택
                        yaml_file = self.get_file_via_terminal("Load Motion YAML File", "*.yaml")
                except Exception as e:
                    print(f"File dialog error: {e}")
                    # 에러 시 터미널 방식으로 폴백
                    yaml_file = self.get_file_via_terminal("Load Motion YAML File", "*.yaml")
            
            # 파일 유효성 검사
            if not yaml_file:
                print("No file selected")
                return
                
            if not os.path.exists(yaml_file):
                print(f"File does not exist: {yaml_file}")
                safe_message_box_error(self, "Error", f"File not found: {yaml_file}")
                return
            
            print(f"Loading YAML file: {yaml_file}")
            
            # YAML 파일 읽기 및 파싱
            try:
                with open(yaml_file, 'r') as f:
                    motion_data = yaml.safe_load(f)
            except yaml.YAMLError as e:
                print(f"YAML parsing error: {e}")
                safe_message_box_error(self, "YAML Error", f"Invalid YAML format: {str(e)}")
                return
            except Exception as e:
                print(f"File reading error: {e}")
                safe_message_box_error(self, "Error", f"Cannot read file: {str(e)}")
                return
            
            if motion_data is None:
                safe_message_box_error(self, "Error", "YAML file is empty")
                return
            
            # 파일 경로 저장
            self.current_yaml_file = yaml_file
            filename = os.path.basename(yaml_file)
            
            # UI 업데이트 - 안전하게 처리
            try:
                # 파일 라벨 업데이트
                if hasattr(self, 'yaml_file_label') and self.yaml_file_label is not None:
                    self.yaml_file_label.setText(f"Loaded: {filename}")
                    self.yaml_file_label.setStyleSheet("color: green; font-weight: bold;")
                
                # 모션 정보 추출
                motion_info = motion_data.get('motion_info', {})
                waypoints = motion_data.get('waypoints', [])
                motion_params = motion_data.get('motion_parameters', {})
                
                # 정보 텍스트 생성
                info_text = f"""Motion Name: {motion_info.get('name', 'Unknown')}
    Motion Type: {motion_info.get('motion_type', 'Unknown')}
    Execution Mode: {motion_info.get('execution_mode', 'auto')}
    Total Waypoints: {len(waypoints)}
    Speed: {motion_params.get('linear_speed_mm_s', 100)} mm/s
    Acceleration: {motion_params.get('acceleration_mm_s2', 200)} mm/s²
    Coordinate Frame: {motion_info.get('coordinate_frame', 'robot_base_frame')}"""
                
                # 모션 정보 텍스트 업데이트
                if hasattr(self, 'motion_info_text') and self.motion_info_text is not None:
                    self.motion_info_text.setPlainText(info_text)
                
                # 실행 모드 라디오 버튼 업데이트
                execution_mode = motion_info.get('execution_mode', 'auto')
                if hasattr(self, 'method_pb_add') and self.method_pb_add is not None:
                    if execution_mode == 'pb_add':
                        self.method_pb_add.setChecked(True)
                    elif execution_mode == 'itpl' and hasattr(self, 'method_itpl') and self.method_itpl is not None:
                        self.method_itpl.setChecked(True)
                    elif execution_mode == 'individual' and hasattr(self, 'method_individual') and self.method_individual is not None:
                        self.method_individual.setChecked(True)
                
                # 파라미터 업데이트
                if hasattr(self, 'acceleration_spinbox') and self.acceleration_spinbox is not None:
                    if 'acceleration_mm_s2' in motion_params:
                        self.acceleration_spinbox.setValue(int(motion_params['acceleration_mm_s2']))
                
                if hasattr(self, 'blend_type_combo') and self.blend_type_combo is not None:
                    if 'blend_type' in motion_params:
                        blend_type = motion_params['blend_type']
                        index = self.blend_type_combo.findText(blend_type)
                        if index >= 0:
                            self.blend_type_combo.setCurrentIndex(index)
                
            except Exception as e:
                print(f"UI update error: {e}")
                import traceback
                traceback.print_exc()
            
            # Robot executor 초기화 및 모션 로드
            try:
                # Robot executor 초기화 확인
                if not hasattr(self, 'robot_executor') or self.robot_executor is None:
                    print("Initializing robot executor...")
                    if not self.initialize_robot_executor_safe():
                        self.log_message("Failed to initialize robot executor")
                        return
                
                # 모션 파일 로드
                if self.robot_executor:
                    success = self.robot_executor.load_motion_yaml(yaml_file)
                    if success:
                        self.log_message(f"Motion file loaded successfully: {filename}")
                        
                        # 로봇이 연결되어 있으면 실행 버튼 활성화
                        if (hasattr(self.robot_executor, 'controller') and 
                            self.robot_executor.controller and 
                            hasattr(self.robot_executor.controller, 'robot_connected') and
                            self.robot_executor.controller.robot_connected):
                            
                            if hasattr(self, 'execute_motion_btn') and self.execute_motion_btn is not None:
                                self.execute_motion_btn.setEnabled(True)
                    else:
                        self.log_message(f"Failed to load motion file: {filename}")
                
            except Exception as e:
                print(f"Robot executor error: {e}")
                import traceback
                traceback.print_exc()
                self.log_message(f"Error loading motion to robot: {str(e)}")
            
            print(f"YAML file loading completed: {filename}")
            
            # 이벤트 처리로 UI 응답성 유지
            if PYQT5_AVAILABLE:
                try:
                    QApplication.processEvents()
                except:
                    pass
                    
        except Exception as e:
            print(f"Unexpected error in load_yaml_for_execution: {e}")
            import traceback
            traceback.print_exc()
            
            # 안전한 에러 메시지 표시
            try:
                safe_message_box_error(self, "Error", f"Failed to load YAML file: {str(e)}")
            except:
                print(f"ERROR: Failed to load YAML file: {str(e)}")
            
    def initialize_robot_executor_safe(self):
        """안전하게 robot executor 초기화"""
        try:
            # 이미 초기화되어 있는지 확인
            if hasattr(self, 'robot_executor_initialized') and self.robot_executor_initialized:
                print("Robot executor already initialized")
                return True
            
            print("Creating RobotMotionExecutor...")
            from realsense_robot_control.robot_controller import RobotMotionExecutor
            self.robot_executor = RobotMotionExecutor()
            
            print("Initializing RobotMotionExecutor...")
            self.robot_executor.initialize()
            
            self.robot_executor_initialized = True
            print("Robot executor initialized successfully")
            return True
            
        except ImportError as e:
            print(f"Import error: {e}")
            print("Creating dummy robot executor...")
            
            # 더미 robot executor 생성
            class DummyRobotExecutor:
                def __init__(self):
                    self.controller = None
                    
                def initialize(self):
                    pass
                    
                def connect_robot(self):
                    return False
                    
                def disconnect_robot(self):
                    pass
                    
                def load_motion_yaml(self, yaml_file):
                    print(f"Dummy: Loading {yaml_file}")
                    return True
                    
                def execute_motion(self, method=None):
                    print(f"Dummy: Executing motion with method {method}")
                    return False
                    
                def stop_motion(self):
                    pass
                    
                def shutdown(self):
                    pass
            
            self.robot_executor = DummyRobotExecutor()
            self.robot_executor_initialized = True
            return True
            
        except Exception as e:
            print(f"Failed to initialize robot executor: {e}")
            import traceback
            traceback.print_exc()
            self.robot_executor = None
            self.robot_executor_initialized = False
            return False
            
    def get_file_via_terminal(self, title, file_pattern):
        """터미널 기반 파일 선택 (대체 방법)"""
        print(f"\n=== {title} ===")
        
        # 현재 디렉토리의 해당 패턴 파일들 찾기
        current_dir = os.getcwd()
        base_output_dir = getattr(self, 'base_output_dir', 'output')
        
        search_dirs = [
            current_dir,
            base_output_dir,
            os.path.join(current_dir, "output"),
            os.path.join(current_dir, "test_data"),
            "/root/slc_ws/src/realsense_robot_control",
            "/tmp"
        ]
        
        found_files = []
        
        for search_dir in search_dirs:
            if os.path.exists(search_dir):
                try:
                    # *.yaml, *.yml 파일 찾기
                    import glob
                    if "yaml" in file_pattern:
                        patterns = ["*.yaml", "*.yml"]
                    else:
                        patterns = [file_pattern]
                    
                    for pattern in patterns:
                        files = glob.glob(os.path.join(search_dir, pattern))
                        for file in files:
                            if file not in found_files:
                                found_files.append(file)
                except Exception as e:
                    print(f"Error searching in {search_dir}: {e}")
        
        if not found_files:
            print("No matching files found in common directories.")
            print("Please enter the full path to your YAML file:")
            file_path = input("File path: ").strip()
            return file_path if file_path and os.path.exists(file_path) else None
        
        print("Found files:")
        for i, file in enumerate(found_files):
            print(f"{i+1}. {file}")
        
        print("0. Enter custom path")
        
        try:
            choice = input(f"Select file (1-{len(found_files)}) or 0 for custom path: ").strip()
            
            if choice == "0":
                file_path = input("Enter full file path: ").strip()
                return file_path if file_path and os.path.exists(file_path) else None
            else:
                idx = int(choice) - 1
                if 0 <= idx < len(found_files):
                    return found_files[idx]
                else:
                    print("Invalid selection")
                    return None
        except (ValueError, KeyboardInterrupt):
            print("Selection cancelled")
            return None
    
    def create_motion_execution_tab(self):
        """Motion execution tab 생성 (이벤트 연결 수정)"""
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Motion file loading section
        file_group = QGroupBox("Motion File")
        file_layout = QVBoxLayout()
        
        file_button_layout = QHBoxLayout()
        self.load_yaml_btn = QPushButton("Load YAML Motion File")
        
        # 버튼 클릭 이벤트 연결 - 올바른 메서드 이름 사용
        print("Connecting Load YAML button...")
        
        # 방법 1: 일반적인 연결 - load_yaml_for_execution이 아닌 load_motion_yaml 사용
        try:
            self.load_yaml_btn.clicked.connect(self.load_motion_yaml)
            print("✓ Standard clicked.connect() successful")
        except Exception as e:
            print(f"✗ Standard connection failed: {e}")
        
        file_button_layout.addWidget(self.load_yaml_btn)
        
        # 나머지 UI 구성...
        self.yaml_file_label = QLabel("No file loaded - Click 'Load YAML Motion File' button")
        self.yaml_file_label.setWordWrap(True)
        self.yaml_file_label.setStyleSheet("color: gray; font-style: italic;")
        
        file_layout.addLayout(file_button_layout)
        file_layout.addWidget(self.yaml_file_label)
        
        file_group.setLayout(file_layout)
        layout.addWidget(file_group)
        
        # Motion information display
        info_group = QGroupBox("Motion Information")
        info_layout = QVBoxLayout()
        
        self.motion_info_text = QTextEdit()
        self.motion_info_text.setMaximumHeight(120)
        self.motion_info_text.setReadOnly(True)
        self.motion_info_text.setPlainText("No motion file loaded")
        info_layout.addWidget(self.motion_info_text)
        
        info_group.setLayout(info_layout)
        layout.addWidget(info_group)
        
        # Execution method selection
        method_group = QGroupBox("Execution Method")
        method_layout = QVBoxLayout()
        
        # Radio buttons for execution method
        method_button_layout = QHBoxLayout()
        
        self.method_individual = QRadioButton("Individual Move_L")
        self.method_individual.setToolTip("Execute waypoints one by one using move_l commands")
        
        self.method_pb_add = QRadioButton("PB Add (Point Buffer)")
        self.method_pb_add.setToolTip("Add all points to buffer and execute as smooth path")
        self.method_pb_add.setChecked(True)  # Default to PB Add
        
        self.method_itpl = QRadioButton("ITPL (Interpolated)")
        self.method_itpl.setToolTip("Use interpolated motion (requires at least 2 points)")
        
        method_button_layout.addWidget(self.method_individual)
        method_button_layout.addWidget(self.method_pb_add)
        method_button_layout.addWidget(self.method_itpl)
        method_layout.addLayout(method_button_layout)
        
        # Method parameters
        param_layout = QHBoxLayout()
        
        param_layout.addWidget(QLabel("Acceleration:"))
        self.acceleration_spinbox = QSpinBox()
        self.acceleration_spinbox.setRange(50, 500)
        self.acceleration_spinbox.setValue(200)
        self.acceleration_spinbox.setSuffix(" mm/s²")
        param_layout.addWidget(self.acceleration_spinbox)
        
        param_layout.addWidget(QLabel("Blend Type:"))
        self.blend_type_combo = QComboBox()
        self.blend_type_combo.addItems(["INTENDED", "CONSTANT"])
        param_layout.addWidget(self.blend_type_combo)
        
        param_layout.addStretch()
        method_layout.addLayout(param_layout)
        
        method_group.setLayout(method_layout)
        layout.addWidget(method_group)
        
        # Robot connection and control
        robot_group = QGroupBox("Robot Control")
        robot_layout = QVBoxLayout()
        
        # Connection status and controls
        connection_layout = QHBoxLayout()
        
        self.robot_status_label = QLabel("Status: Disconnected")
        self.robot_status_label.setStyleSheet("color: red; font-weight: bold;")
        connection_layout.addWidget(self.robot_status_label)
        
        connection_layout.addStretch()
        
        self.connect_robot_btn = QPushButton("Connect Robot")
        self.connect_robot_btn.clicked.connect(self.connect_robot)
        connection_layout.addWidget(self.connect_robot_btn)
        
        self.disconnect_robot_btn = QPushButton("Disconnect")
        self.disconnect_robot_btn.clicked.connect(self.disconnect_robot)
        self.disconnect_robot_btn.setEnabled(False)
        connection_layout.addWidget(self.disconnect_robot_btn)
        
        robot_layout.addLayout(connection_layout)
        
        # Execution controls
        execution_layout = QHBoxLayout()
        
        self.execute_motion_btn = QPushButton("Execute Motion")
        self.execute_motion_btn.clicked.connect(self.execute_motion)
        self.execute_motion_btn.setEnabled(False)
        execution_layout.addWidget(self.execute_motion_btn)
        
        self.stop_motion_btn = QPushButton("Stop Motion")
        self.stop_motion_btn.clicked.connect(self.stop_motion)
        self.stop_motion_btn.setEnabled(False)
        execution_layout.addWidget(self.stop_motion_btn)
        
        self.emergency_stop_btn = QPushButton("EMERGENCY STOP")
        self.emergency_stop_btn.clicked.connect(self.emergency_stop)
        self.emergency_stop_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        execution_layout.addWidget(self.emergency_stop_btn)
        
        robot_layout.addLayout(execution_layout)
        
        robot_group.setLayout(robot_layout)
        layout.addWidget(robot_group)
        
        # Execution log
        log_group = QGroupBox("Execution Log")
        log_layout = QVBoxLayout()
        
        self.execution_log = QTextEdit()
        self.execution_log.setMaximumHeight(150)
        self.execution_log.setReadOnly(True)
        self.execution_log.setPlainText("System ready")
        log_layout.addWidget(self.execution_log)
        
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        # "Use Last Generated Motion" 버튼 추가
        self.use_generated_btn = QPushButton("Use Last Generated Motion")
        self.use_generated_btn.clicked.connect(self.use_generated_motion)
        self.use_generated_btn.setEnabled(False)
        file_button_layout.addWidget(self.use_generated_btn)
        
        layout.addStretch()
        tab.setLayout(layout)
        return tab

    def handle_load_yaml_click(self):
        """Load YAML 버튼 클릭 처리 (수정된 버전)"""
        print("=== Load YAML Button Clicked ===")
        try:
            # 즉시 상태 업데이트
            if hasattr(self, 'yaml_file_label') and self.yaml_file_label:
                self.yaml_file_label.setText("Processing file selection...")
                self.yaml_file_label.setStyleSheet("color: blue;")
            
            # QApplication 이벤트 처리
            if PYQT5_AVAILABLE:
                QApplication.processEvents()
            
            # 올바른 메서드 호출
            self.load_motion_yaml()  # load_yaml_for_execution이 아닌 load_motion_yaml
            
        except Exception as e:
            print(f"Error in handle_load_yaml_click: {e}")
            import traceback
            traceback.print_exc()
            
            # 에러 상태 표시
            if hasattr(self, 'yaml_file_label') and self.yaml_file_label:
                self.yaml_file_label.setText(f"Error: {str(e)}")
                self.yaml_file_label.setStyleSheet("color: red;")

    def debug_event_system(self):
        """이벤트 시스템 디버깅"""
        print("=== Event System Debug ===")
        print(f"PYQT5_AVAILABLE: {PYQT5_AVAILABLE}")
        print(f"QApplication instance: {QApplication.instance()}")
        
        if hasattr(self, 'load_yaml_btn'):
            print(f"Load YAML button exists: {self.load_yaml_btn}")
            print(f"Button type: {type(self.load_yaml_btn)}")
            
            # 버튼 상태 확인
            if hasattr(self.load_yaml_btn, 'isEnabled'):
                print(f"Button enabled: {self.load_yaml_btn.isEnabled()}")
            
            # 시그널 연결 확인
            if hasattr(self.load_yaml_btn, 'clicked'):
                print(f"Clicked signal: {self.load_yaml_btn.clicked}")
        
        print("==========================")

    def connect_robot(self):
        """Connect to robot with safe initialization"""
        try:
            # Ensure robot executor is initialized
            if not self.robot_executor_initialized:
                if not self.initialize_robot_executor_safe():
                    safe_message_box_error(self, "Error", "Failed to initialize robot executor")
                    return
            
            self.log_message("Connecting to robot...")
            success = self.robot_executor.connect_robot()
            
            if success:
                self.robot_status_label.setText("Status: Connected")
                self.robot_status_label.setStyleSheet("color: green; font-weight: bold;")
                self.connect_robot_btn.setEnabled(False)
                self.disconnect_robot_btn.setEnabled(True)
                
                # Enable execution if motion file is loaded
                if self.current_yaml_file:
                    self.execute_motion_btn.setEnabled(True)
                
                self.log_message("Robot connected successfully")
            else:
                self.log_message("Failed to connect to robot")
                
        except Exception as e:
            safe_message_box_error(self, "Connection Error", f"Failed to connect to robot: {str(e)}")
            self.log_message(f"Connection error: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def disconnect_robot(self):
        """Disconnect from robot"""
        try:
            if self.robot_executor:
                self.robot_executor.disconnect_robot()
            
            self.robot_status_label.setText("Status: Disconnected")
            self.robot_status_label.setStyleSheet("color: red; font-weight: bold;")
            self.connect_robot_btn.setEnabled(True)
            self.disconnect_robot_btn.setEnabled(False)
            self.execute_motion_btn.setEnabled(False)
            self.stop_motion_btn.setEnabled(False)
            
            self.log_message("Robot disconnected")
            
        except Exception as e:
            self.log_message(f"Disconnect error: {str(e)}")
    
    def execute_motion(self):
        """Execute motion with selected method"""
        if not self.current_yaml_file:
            safe_message_box_warning(self, "Warning", "No motion file loaded!")
            return
        
        if not hasattr(self, 'robot_executor') or not self.robot_executor:
            safe_message_box_error(self, "Error", "Robot not connected!")
            return
        
        # Determine execution method
        if self.method_pb_add.isChecked():
            method = 'pb_add'
        elif self.method_itpl.isChecked():
            method = 'itpl'
        else:
            method = 'individual'
        
        try:
            self.execute_motion_btn.setEnabled(False)
            self.stop_motion_btn.setEnabled(True)
            
            # Set parameters
            acceleration = self.acceleration_spinbox.value()
            blend_type = self.blend_type_combo.currentText()
            
            # Update robot controller parameters
            if hasattr(self.robot_executor, 'controller') and self.robot_executor.controller:
                controller = self.robot_executor.controller
                controller.max_acceleration = acceleration
                
                # Update motion parameters
                if controller.motion_data:
                    if 'motion_parameters' not in controller.motion_data:
                        controller.motion_data['motion_parameters'] = {}
                    controller.motion_data['motion_parameters']['acceleration_mm_s2'] = acceleration
                    controller.motion_data['motion_parameters']['blend_type'] = blend_type
            
            self.log_message(f"Starting {method.upper()} motion execution...")
            self.log_message(f"Acceleration: {acceleration} mm/s², Blend: {blend_type}")
            
            # Execute motion
            success = self.robot_executor.execute_motion(method)
            
            if success:
                self.log_message(f"Motion execution started successfully")
                self.robot_status_label.setText("Status: Executing Motion")
                self.robot_status_label.setStyleSheet("color: blue;")
            else:
                self.log_message(f"Failed to start motion execution")
                self.execute_motion_btn.setEnabled(True)
                self.stop_motion_btn.setEnabled(False)
            
        except Exception as e:
            safe_message_box_error(self, "Execution Error", f"Failed to execute motion: {str(e)}")
            self.log_message(f"Execution error: {str(e)}")
    
    def stop_motion(self):
        """Stop current motion"""
        try:
            if self.robot_executor:
                self.robot_executor.stop_motion()
            self.log_message("Motion stopped")
        except Exception as e:
            self.log_message(f"Stop error: {str(e)}")
            
    def emergency_stop(self):
        """Emergency stop"""
        try:
            if hasattr(self, 'robot_executor') and self.robot_executor:
                self.robot_executor.stop_motion()
            self.log_message("EMERGENCY STOP activated")
            
            # 버튼 상태 업데이트
            if hasattr(self, 'execute_motion_btn') and self.execute_motion_btn:
                self.execute_motion_btn.setEnabled(True)
            if hasattr(self, 'stop_motion_btn') and self.stop_motion_btn:
                self.stop_motion_btn.setEnabled(False)
            
            # 로봇 상태 업데이트
            if hasattr(self, 'robot_status_label') and self.robot_status_label:
                self.robot_status_label.setText("Status: Emergency Stop")
                self.robot_status_label.setStyleSheet("color: red; font-weight: bold;")
                
        except Exception as e:
            self.log_message(f"Emergency stop error: {str(e)}")
            print(f"Emergency stop error: {e}")
    
    def control_gripper(self, open_gripper):
        """Control robot gripper"""
        self.robot_executor.set_gripper(open_gripper)
        state = "opened" if open_gripper else "closed"
        self.log_message(f"Gripper {state}")
    
    def update_robot_status(self, status):
        """Update robot status display"""
        # This would be called by robot status callbacks
        pass
    
    def log_message(self, message):
        """Add message to execution log"""
        if hasattr(self, 'execution_log') and self.execution_log:
            timestamp = datetime.now().strftime("%H:%M:%S")
            log_entry = f"[{timestamp}] {message}"
            self.execution_log.append(log_entry)
            
            # Auto scroll to bottom
            scrollbar = self.execution_log.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())
        else:
            # Fallback to print if log widget not available
            print(f"LOG: {message}")
    
    # Original methods from realsense_viewer.py
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
    
    def save_rgb_image(self):
        """Save RGB image as PNG"""
        if self.current_rgb_image is None:
            QMessageBox.warning(self, "Warning", "No RGB image available to save!")
            return
        
        timestamp = datetime.now().strftime("%H%M%S")
        filename = f"rgb_image_{timestamp}.png"
        file_path = os.path.join(self.base_output_dir, filename)
        
        cv2.imwrite(file_path, self.current_rgb_image)
        self.statusBar().showMessage(f"RGB image saved: {file_path}")
    
    def save_depth_image(self):
        """Save depth image as PNG"""
        if self.ros_node is None or self.ros_node.latest_depth_image is None:
            QMessageBox.warning(self, "Warning", "No depth image available to save!")
            return
        
        timestamp = datetime.now().strftime("%H%M%S")
        filename = f"depth_image_{timestamp}.png"
        file_path = os.path.join(self.base_output_dir, filename)
        
        cv2.imwrite(file_path, self.ros_node.latest_depth_image.astype(np.uint16))
        self.statusBar().showMessage(f"Depth image saved: {file_path}")
    
    def save_pointcloud(self):
        """Save point cloud with RGB as PLY"""
        if self.ros_node is None or self.ros_node.latest_points is None:
            QMessageBox.warning(self, "Warning", "No point cloud available to save!")
            return
        
        timestamp = datetime.now().strftime("%H%M%S")
        filename = f"pointcloud_rgb_{timestamp}.ply"
        file_path = os.path.join(self.base_output_dir, filename)
        
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
        timestamp = datetime.now().strftime("%H%M%S")
        
        saved_files = []
        
        if self.current_rgb_image is not None:
            rgb_path = os.path.join(self.base_output_dir, f"rgb_image_{timestamp}.png")
            cv2.imwrite(rgb_path, self.current_rgb_image)
            saved_files.append("RGB image")
        
        if self.ros_node and self.ros_node.latest_depth_image is not None:
            depth_path = os.path.join(self.base_output_dir, f"depth_image_{timestamp}.png")
            cv2.imwrite(depth_path, self.ros_node.latest_depth_image.astype(np.uint16))
            saved_files.append("Depth image")
        
        if self.ros_node and self.ros_node.latest_points is not None:
            pc_path = os.path.join(self.base_output_dir, f"pointcloud_rgb_{timestamp}.ply")
            try:
                self.save_ply_file(pc_path, 
                                 self.ros_node.latest_points, 
                                 self.ros_node.latest_colors)
                saved_files.append(f"Point cloud ({len(self.ros_node.latest_points):,} points)")
            except Exception as e:
                self.statusBar().showMessage(f"Point cloud save error: {str(e)}")
        
        if saved_files:
            self.statusBar().showMessage(f"Saved: {', '.join(saved_files)} to {self.base_output_dir}")
        else:
            QMessageBox.warning(self, "Warning", "No data available to save!")
    
    def ros_spin(self):
        """ROS2 node를 별도 스레드에서 실행 - 이벤트 루프 블로킹 방지"""
        import rclpy
        from rclpy.node import Node
        
        try:
            # RCL이 이미 초기화되었는지 확인
            if not rclpy.ok():
                rclpy.init()
            
            # 간단한 더미 노드로 시작 (GUI 블로킹 방지)
            class SimpleRealSenseNode(Node):
                def __init__(self, signal_emitter):
                    super().__init__('simple_realsense_node')
                    self.signal_emitter = signal_emitter
                    self.latest_rgb_image = None
                    self.latest_depth_image = None
                    self.latest_points = None
                    self.latest_colors = None
                    
                    # RGB 이미지 구독
                    self.rgb_sub = self.create_subscription(
                        Image,
                        '/camera/camera/color/image_raw',
                        self.rgb_callback,
                        10
                    )
                    
                    # Depth 이미지 구독
                    self.depth_sub = self.create_subscription(
                        Image,
                        '/camera/camera/depth/image_rect_raw',
                        self.depth_callback,
                        10
                    )
                    
                    # 상태 확인 타이머
                    self.status_timer = self.create_timer(5.0, self.check_topics)
                    
                    # CvBridge
                    from cv_bridge import CvBridge
                    self.bridge = CvBridge()
                    
                    self.get_logger().info("Simple RealSense node initialized")
                
                def rgb_callback(self, msg):
                    """RGB 이미지 콜백"""
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                        self.latest_rgb_image = cv_image
                        self.signal_emitter.update_rgb_signal.emit(cv_image)
                    except Exception as e:
                        self.get_logger().error(f'RGB callback error: {str(e)}')
                
                def depth_callback(self, msg):
                    """Depth 이미지 콜백"""
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                        self.latest_depth_image = cv_image
                        
                        # Normalize for visualization
                        import cv2
                        depth_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
                        depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
                        
                        self.signal_emitter.update_depth_signal.emit(depth_colored)
                    except Exception as e:
                        self.get_logger().error(f'Depth callback error: {str(e)}')
                
                def check_topics(self):
                    """토픽 상태 확인"""
                    topic_names = self.get_topic_names_and_types()
                    camera_topics = [name for name, _ in topic_names if 'camera' in name]
                    
                    if camera_topics:
                        self.get_logger().info(f"Available camera topics: {camera_topics[:5]}...")
                        self.signal_emitter.update_fps_signal.emit("FPS: Camera Connected")
                    else:
                        self.get_logger().warn("No camera topics found")
                        self.signal_emitter.update_fps_signal.emit("FPS: No Camera")
                
                def cleanup(self):
                    """정리"""
                    self.get_logger().info("Cleaning up simple RealSense node")
            
            print("Creating Simple RealSense Node...")
            self.ros_node = SimpleRealSenseNode(self.signal_emitter)
            print("Simple RealSense Node created successfully")
            
            # 스피너 실행 (논블로킹)
            print("Starting ROS spin...")
            
            # SingleThreadedExecutor 사용하여 논블로킹 실행
            from rclpy.executors import SingleThreadedExecutor
            executor = SingleThreadedExecutor()
            executor.add_node(self.ros_node)
            
            # 짧은 주기로 spin_once 실행하여 GUI 블로킹 방지
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
                time.sleep(0.01)  # GUI 이벤트 처리 시간 확보
                
        except Exception as e:
            print(f"ROS spinning error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # 안전한 정리
            try:
                if hasattr(self, 'ros_node') and self.ros_node:
                    print("Cleaning up ROS node...")
                    self.ros_node.cleanup()
                    self.ros_node.destroy_node()
                    print("ROS node cleanup completed")
            except Exception as e:
                print(f"Node cleanup error: {e}")
    
    def closeEvent(self, event):
        """Handle window close event with proper cleanup"""
        try:
            print("Cleaning up resources...")
            
            # Cleanup robot executor safely
            if hasattr(self, 'robot_executor') and self.robot_executor:
                try:
                    self.robot_executor.shutdown()
                except Exception as e:
                    print(f"Error shutting down robot executor: {e}")
            
            # Cleanup ROS node
            if hasattr(self, 'ros_node') and self.ros_node:
                try:
                    self.ros_node.cleanup()
                    self.ros_node.destroy_node()
                except Exception as e:
                    print(f"Error destroying node: {e}")
            
            print("Cleanup completed")
            event.accept()
            
        except Exception as e:
            print(f"Error during cleanup: {e}")
            event.accept()

def main():
    """Main entry point for RealSense Viewer V4"""
    import signal
    import sys
    
    def signal_handler(signum, frame):
        print(f"Signal {signum} received, shutting down...")
        if 'app' in locals():
            app.quit()
        sys.exit(0)
    
    # Install signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # QApplication 생성 - ROS2 초기화 전에 수행
    app = QApplication(sys.argv)
    
    # Enable Ctrl+C handling in Qt
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)
    
    # Create the main viewer V4 instance
    try:
        viewer = RealSenseViewerV4()
        viewer.show()
        
        print("RealSense Viewer V4 started - Vision-based Robot Control System")
        print("Available features:")
        print("- Camera View (Step 1)")
        print("- Surface Detection (Step 2)")
        print("- Motion Generation (Step 3)")
        print("- Motion Execution (Step 4)")
        print("\nGUI should be responsive now. Try clicking buttons.")
        
        # 이벤트 루프 시작
        sys.exit(app.exec_())
        
    except KeyboardInterrupt:
        print("Keyboard interrupt in main")
        sys.exit(0)
    except Exception as e:
        print(f"Error creating viewer: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)



if __name__ == '__main__':
    main()