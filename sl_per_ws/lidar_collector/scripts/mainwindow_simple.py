#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dual LiDAR Point Cloud Collector - Web Interface with AUTO START Pipeline
- Left LiDAR: 3GGDN8700225691 (Mid-70)
- Right LiDAR: 3GGDN8700226431 (Mid-70)

Uses subprocess to call ROS2 services (avoids rclpy threading issues with Flask)
Supports AUTO START pipeline for automated point cloud processing
"""

import os
import subprocess
import datetime
import json
import time
import threading
import glob

from flask import Flask, render_template_string, jsonify, send_from_directory, send_file

app = Flask(__name__)

# Get the workspace root directory
# Script is at: lidar_collector/scripts/mainwindow_simple.py
# Workspace root is 2 levels up: lidar_collector/scripts -> lidar_collector -> workspace_root
WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


class DualLidarInterface:
    def __init__(self):
        # State variables
        self.is_collecting = False
        self.log_messages = []

        # PCD save paths
        self.save_path_left = os.path.join(WORKSPACE_ROOT, 'pcd_file/left')
        self.save_path_right = os.path.join(WORKSPACE_ROOT, 'pcd_file/right')

        # Create directories
        os.makedirs(self.save_path_left, exist_ok=True)
        os.makedirs(self.save_path_right, exist_ok=True)

        # Pipeline state
        self.pipeline_progress = 0
        self.current_stage = "idle"
        self.current_message = ""
        self.pipeline_running = False
        self.pipeline_error = None
        self.pipeline_stop_requested = False  # Stop flag for AUTO-STOP

        self.write_log("Dual LiDAR Interface initialized")
        self.write_log(f"Workspace root: {WORKSPACE_ROOT}")
        self.write_log(f"Left save path: {self.save_path_left}")
        self.write_log(f"Right save path: {self.save_path_right}")

    def write_log(self, msg):
        """Add log message"""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {msg}"
        self.log_messages.append(log_entry)
        if len(self.log_messages) > 100:
            self.log_messages = self.log_messages[-100:]
        print(log_entry)

    def update_progress(self, progress, stage, message):
        """Update pipeline progress"""
        self.pipeline_progress = progress
        self.current_stage = stage
        self.current_message = message
        self.write_log(f"[{stage}] {message} ({progress}%)")

    def call_ros2_service(self, service_name, service_type, request_data=""):
        """Call ROS2 service using subprocess"""
        try:
            cmd = ["ros2", "service", "call", service_name, service_type]
            if request_data:
                cmd.append(request_data)

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=10.0
            )

            if result.returncode == 0:
                return {"success": True, "output": result.stdout}
            else:
                return {"success": False, "output": result.stderr}

        except subprocess.TimeoutExpired:
            return {"success": False, "output": "Service call timeout"}
        except Exception as e:
            return {"success": False, "output": str(e)}

    def start_collection(self):
        """Start point cloud collection"""
        self.write_log("Calling start_collection service...")

        result = self.call_ros2_service(
            "/dual_lidar/start_collection",
            "std_srvs/srv/SetBool",
            "{data: true}"
        )

        if result["success"]:
            self.is_collecting = True
            self.write_log("Collection STARTED")
            return {"success": True, "message": "Collection started"}
        else:
            self.write_log(f"Failed to start: {result['output']}")
            return {"success": False, "message": result["output"]}

    def stop_collection(self):
        """Stop point cloud collection without saving"""
        result = self.call_ros2_service(
            "/dual_lidar/start_collection",
            "std_srvs/srv/SetBool",
            "{data: false}"
        )

        if result["success"]:
            self.is_collecting = False
            self.write_log("Collection STOPPED")
            return {"success": True, "message": "Collection stopped"}
        else:
            return {"success": False, "message": result["output"]}

    def save_pointcloud(self):
        """Save collected point clouds to PCD files"""
        self.write_log("Calling save_pointcloud service...")

        result = self.call_ros2_service(
            "/dual_lidar/save_pointcloud",
            "std_srvs/srv/Trigger",
            ""
        )

        self.is_collecting = False

        if result["success"]:
            output = result["output"]
            self.write_log(f"Save completed: {output[:200]}")
            return {"success": True, "message": "Point clouds saved"}
        else:
            self.write_log(f"Save failed: {result['output']}")
            return {"success": False, "message": result["output"]}

    def get_status(self):
        """Get current status"""
        left_files = len([f for f in os.listdir(self.save_path_left) if f.endswith('.pcd')]) if os.path.exists(self.save_path_left) else 0
        right_files = len([f for f in os.listdir(self.save_path_right) if f.endswith('.pcd')]) if os.path.exists(self.save_path_right) else 0

        return {
            "is_collecting": self.is_collecting,
            "left_path": self.save_path_left,
            "right_path": self.save_path_right,
            "left_file_count": left_files,
            "right_file_count": right_files
        }

    def get_recent_files(self, limit=5):
        """Get recent PCD files"""
        files = {"left": [], "right": []}

        for side, path in [("left", self.save_path_left), ("right", self.save_path_right)]:
            if os.path.exists(path):
                pcd_files = [f for f in os.listdir(path) if f.endswith('.pcd')]
                pcd_files.sort(reverse=True)
                files[side] = pcd_files[:limit]

        return files

    def _get_latest_pcd(self, directory):
        """Get the most recent PCD file in a directory"""
        pcd_files = glob.glob(os.path.join(directory, "*.pcd"))
        if not pcd_files:
            raise FileNotFoundError(f"No PCD files in {directory}")
        return max(pcd_files, key=os.path.getmtime)

    # ============================================================
    # AUTO START Pipeline Methods
    # ============================================================

    def reset_pipeline(self):
        """Reset pipeline state for fresh start"""
        self.pipeline_progress = 0
        self.current_stage = "idle"
        self.current_message = ""
        self.pipeline_running = False
        self.pipeline_error = None
        self.pipeline_stop_requested = False
        self.is_collecting = False
        self.write_log("Pipeline state reset")

    def stop_pipeline(self):
        """Stop the running pipeline and reset"""
        if not self.pipeline_running:
            return {"success": False, "message": "Pipeline is not running"}

        self.pipeline_stop_requested = True
        self.write_log("Pipeline stop requested")

        # Stop any ongoing collection
        self.call_ros2_service(
            "/dual_lidar/start_collection",
            "std_srvs/srv/SetBool",
            "{data: false}"
        )

        return {"success": True, "message": "Pipeline stop requested"}

    def _check_stop_requested(self):
        """Check if stop was requested and raise exception if so"""
        if self.pipeline_stop_requested:
            raise Exception("Pipeline stopped by user")

    def run_auto_pipeline(self):
        """Run the full AUTO START pipeline"""
        if self.pipeline_running:
            return {"success": False, "message": "Pipeline already running"}

        # Reset stop flag before starting
        self.pipeline_stop_requested = False

        # 기존 결과 파일 정리 (새로 실행 시 혼란 방지)
        import shutil
        out_dir = os.path.join(WORKSPACE_ROOT, "out_icp_only")
        if os.path.exists(out_dir):
            shutil.rmtree(out_dir)
            self.write_log("Cleaned old out_icp_only folder")

        self.pipeline_running = True
        self.pipeline_error = None

        try:
            # Step 1: Point cloud collection (30 seconds)
            self._check_stop_requested()
            self.update_progress(0, "collect", "Starting point cloud collection")
            self._step1_collect()

            # Step 2: Dual LiDAR registration
            self._check_stop_requested()
            self.update_progress(16, "registration", "Running dual LiDAR registration")
            self._step2_registration()

            # Step 3: Downsampling
            self._check_stop_requested()
            self.update_progress(40, "downsample", "Running downsampling")
            self._step3_downsample()

            # Step 4: ICP Registration
            self._check_stop_requested()
            self.update_progress(55, "icp", "Running ICP registration")
            self._step4_icp()

            # Step 5: Target generation
            self._check_stop_requested()
            self.update_progress(80, "target", "Generating target path")
            self._step5_target()

            # Complete
            self.update_progress(100, "complete", "Pipeline completed successfully")

        except Exception as e:
            error_msg = str(e)
            if "stopped by user" in error_msg.lower():
                self.update_progress(0, "stopped", "Pipeline stopped by user")
                self.write_log("Pipeline stopped by user - resetting state")
                # Reset state after stop
                self.reset_pipeline()
            else:
                self.pipeline_error = error_msg
                self.update_progress(self.pipeline_progress, "error", error_msg)
                self.write_log(f"Pipeline error: {e}")
        finally:
            self.pipeline_running = False
            self.pipeline_stop_requested = False

    def _step1_collect(self):
        """Step 1: Point cloud collection (30 seconds)"""
        # Start collection
        result = self.call_ros2_service(
            "/dual_lidar/start_collection",
            "std_srvs/srv/SetBool",
            "{data: true}"
        )

        if not result["success"]:
            self.write_log(f"Warning: Could not start ROS2 collection service: {result['output']}")
            self.write_log("Simulating collection for testing...")

        # Wait for 30 seconds with progress updates
        for i in range(30):
            # Check for stop request during collection
            if self.pipeline_stop_requested:
                # Stop collection immediately
                self.call_ros2_service(
                    "/dual_lidar/start_collection",
                    "std_srvs/srv/SetBool",
                    "{data: false}"
                )
                raise Exception("Pipeline stopped by user")
            time.sleep(1)
            progress = int((i + 1) / 30 * 16)
            self.update_progress(progress, "collect", f"Collecting... {i+1}/30 seconds")

        # Save collected point clouds
        result = self.call_ros2_service(
            "/dual_lidar/save_pointcloud",
            "std_srvs/srv/Trigger",
            ""
        )

        if not result["success"]:
            self.write_log(f"Warning: Could not save via ROS2 service: {result['output']}")

    def _step2_registration(self):
        """Step 2: Dual LiDAR registration (view.py with headless mode)"""
        try:
            # Find latest PCD files
            left_file = self._get_latest_pcd(self.save_path_left)
            right_file = self._get_latest_pcd(self.save_path_right)

            self.write_log(f"Left PCD: {left_file}")
            self.write_log(f"Right PCD: {right_file}")

            # Run view.py with headless mode (no GUI visualization)
            # Auto-loads latest PCD files from left/right directories
            result = subprocess.run(
                [
                    "python3", "view.py",
                    "--headless",
                    "--left-dir", self.save_path_left,
                    "--right-dir", self.save_path_right
                ],
                capture_output=True,
                text=True,
                timeout=300,  # 5 minutes timeout
                cwd=WORKSPACE_ROOT
            )

            # Log last 10 lines of output
            if result.stdout:
                for line in result.stdout.split('\n')[-10:]:
                    if line.strip():
                        self.write_log(f"[view.py] {line}")

            if result.returncode != 0:
                self.write_log(f"view.py warning: {result.stderr[:500]}")
                # Continue anyway if output files exist

            # Check if output files were created
            merged_pcd = os.path.join(WORKSPACE_ROOT, "merged_world.pcd")
            cropped_pcd = os.path.join(WORKSPACE_ROOT, "cropped_world.pcd")

            if os.path.exists(cropped_pcd):
                self.write_log("Registration completed: cropped_world.pcd created")
            elif os.path.exists(merged_pcd):
                self.write_log("Registration completed: merged_world.pcd created")
            else:
                self.write_log("Warning: Registration output not found, continuing...")

        except FileNotFoundError as e:
            self.write_log(f"Warning: {e}")
            self.write_log("Skipping registration step - no PCD files found")
        except subprocess.TimeoutExpired:
            self.write_log("Warning: view.py timed out")
        except Exception as e:
            self.write_log(f"Registration warning: {e}")

    def _step3_downsample(self):
        """Step 3: Downsampling (downsample.py)"""
        try:
            result = subprocess.run(
                ["python3", "downsample.py"],
                capture_output=True,
                text=True,
                timeout=300,
                cwd=WORKSPACE_ROOT
            )

            if result.returncode != 0:
                self.write_log(f"downsample.py warning: {result.stderr[:500]}")

            downsampled = os.path.join(WORKSPACE_ROOT, "cropped_world_downsampled.pcd")
            if os.path.exists(downsampled):
                self.write_log("Downsampling completed")
            else:
                self.write_log("Warning: Downsampled output not found")

        except subprocess.TimeoutExpired:
            self.write_log("Warning: downsample.py timed out")
        except Exception as e:
            self.write_log(f"Downsample warning: {e}")

    def _step4_icp(self):
        """Step 4: ICP Registration (final.py)"""
        try:
            result = subprocess.run(
                ["python3", "final.py"],
                capture_output=True,
                text=True,
                timeout=3600,  # 1 hour timeout for ICP
                cwd=WORKSPACE_ROOT
            )

            if result.returncode != 0:
                self.write_log(f"final.py info: May have display errors (normal on server)")

            # Check for transformation matrix (cloudcompy 폴더 우선 검색)
            xform_paths = [
                os.path.join(WORKSPACE_ROOT, "global/out_icp_only/T_ms_overall.txt"),
                os.path.join(WORKSPACE_ROOT, "out_icp_only/T_ms_overall.txt"),
                os.path.join(WORKSPACE_ROOT, "T_ms_overall.txt"),
            ]

            xform_found = False
            for xform_file in xform_paths:
                if os.path.exists(xform_file):
                    self.write_log(f"ICP completed: Transform matrix at {xform_file}")
                    xform_found = True
                    break

            if not xform_found:
                self.write_log("Warning: ICP transform matrix not found")

        except subprocess.TimeoutExpired:
            self.write_log("Warning: final.py timed out")
        except Exception as e:
            self.write_log(f"ICP warning: {e}")

    def _step5_target(self):
        """Step 5: Target path generation (paview.py)"""
        try:
            # Find transform matrix file (cloudcompy 폴더 우선 검색)
            xform_paths = [
                os.path.join(WORKSPACE_ROOT, "global/out_icp_only/T_ms_overall.txt"),
                os.path.join(WORKSPACE_ROOT, "out_icp_only/T_ms_overall.txt"),
                os.path.join(WORKSPACE_ROOT, "T_ms_overall.txt"),
            ]

            transform_file = None
            for path in xform_paths:
                if os.path.exists(path):
                    transform_file = path
                    self.write_log(f"Transform file found: {path}")
                    break

            if transform_file is None:
                self.write_log("ERROR: Transform matrix file not found!")
                self.write_log("Searched paths: " + ", ".join(xform_paths))
                return

            # Build command with transform file path
            cmd = [
                "python3", "paview.py",
                "--transform", transform_file
            ]

            self.write_log(f"Running: {' '.join(cmd)}")

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=300,
                cwd=WORKSPACE_ROOT
            )

            # Log output for debugging
            if result.stdout:
                for line in result.stdout.split('\n')[-15:]:
                    if line.strip():
                        self.write_log(f"[paview] {line}")

            if result.returncode == 0:
                self.write_log("Target generation completed")
                # Extract output path from result (with microseconds: YYMMDD_HHMMSS_ffffff)
                output = result.stdout
                if "robot_path_mm_" in output:
                    import re
                    # Match both old (YYMMDD_HHMMSS) and new (YYMMDD_HHMMSS_ffffff) formats
                    match = re.search(r'robot_path_mm_\d{6}_\d{6}(_\d{6})?\.json', output)
                    if match:
                        self.write_log(f"Generated: {match.group()}")
            else:
                self.write_log(f"Target generation error (returncode={result.returncode})")
                if result.stderr:
                    self.write_log(f"stderr: {result.stderr[:500]}")

        except subprocess.TimeoutExpired:
            self.write_log("Warning: paview.py timed out")
        except Exception as e:
            self.write_log(f"Target generation warning: {e}")


# Global interface instance
iface = None


# ============================================================
# HTML Template (Fallback if index.html not found)
# ============================================================
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8"/>
    <title>Dual LiDAR Controller</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: 'Segoe UI', 'Malgun Gothic', sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            padding: 20px;
            color: #eee;
        }
        .container { max-width: 900px; margin: 0 auto; }
        h1 {
            text-align: center;
            color: #00d9ff;
            margin-bottom: 25px;
            font-size: 26px;
        }
        .panel {
            background: rgba(255, 255, 255, 0.08);
            border-radius: 12px;
            padding: 20px;
            margin-bottom: 15px;
        }
        .btn-container {
            display: flex;
            justify-content: center;
            gap: 20px;
            padding: 10px 0;
        }
        button {
            padding: 15px 40px;
            font-size: 16px;
            font-weight: bold;
            border: none;
            border-radius: 10px;
            cursor: pointer;
        }
        .btn-start { background: linear-gradient(135deg, #00d9ff 0%, #00ff88 100%); color: #1a1a2e; }
        .btn-auto { background: linear-gradient(135deg, #ff00ff 0%, #cc00cc 100%); color: white; }
        .log-area {
            background: #0a0a0a;
            color: #00ff88;
            padding: 12px;
            height: 200px;
            overflow-y: auto;
            font-family: monospace;
            font-size: 12px;
        }
        .progress-bar {
            width: 100%; height: 30px;
            background: #333; border-radius: 15px;
            overflow: hidden; margin: 20px 0;
        }
        .progress-fill {
            height: 100%; background: linear-gradient(90deg, #ff00ff, #00ffff);
            transition: width 0.3s;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Dual LiDAR Controller</h1>
        <div class="panel">
            <p style="text-align:center; margin-bottom:20px;">New UI available at: <a href="/new" style="color:#00ffff;">/new</a></p>
            <div class="btn-container">
                <button class="btn-start" onclick="startCollection()">Start Collection</button>
                <button class="btn-auto" onclick="autoStart()">AUTO START</button>
            </div>
            <div class="progress-bar"><div id="progress" class="progress-fill" style="width:0%"></div></div>
            <div id="status" style="text-align:center; margin:10px 0;">Ready</div>
        </div>
        <div class="panel">
            <div id="logArea" class="log-area"></div>
        </div>
    </div>
    <script>
        async function startCollection() {
            const response = await fetch('/api/start', { method: 'POST' });
            const data = await response.json();
            addLog(data.message);
        }
        async function autoStart() {
            addLog('Starting AUTO pipeline...');
            const response = await fetch('/api/auto_start', { method: 'POST' });
            const data = await response.json();
            addLog(data.message);
            if (data.success) pollProgress();
        }
        async function pollProgress() {
            const response = await fetch('/api/progress');
            const data = await response.json();
            document.getElementById('progress').style.width = data.progress + '%';
            document.getElementById('status').textContent = data.message;
            addLog('[' + data.stage + '] ' + data.message);
            if (data.stage !== 'complete' && data.stage !== 'error') {
                setTimeout(pollProgress, 1000);
            }
        }
        function addLog(msg) {
            const log = document.getElementById('logArea');
            const time = new Date().toLocaleTimeString();
            log.innerHTML += '[' + time + '] ' + msg + '<br>';
            log.scrollTop = log.scrollHeight;
        }
        setInterval(async () => {
            const response = await fetch('/api/logs');
            const data = await response.json();
            document.getElementById('logArea').innerHTML = data.logs.join('<br>');
        }, 2000);
    </script>
</body>
</html>
"""


# ============================================================
# Routes
# ============================================================

@app.route('/')
def index():
    """Serve the main page - try index.html first, fallback to template"""
    index_path = os.path.join(WORKSPACE_ROOT, 'index.html')
    if os.path.exists(index_path):
        return send_file(index_path)
    return render_template_string(HTML_TEMPLATE)


@app.route('/new')
def new_ui():
    """Serve the new UI explicitly"""
    index_path = os.path.join(WORKSPACE_ROOT, 'index.html')
    if os.path.exists(index_path):
        return send_file(index_path)
    return "index.html not found", 404


@app.route('/old')
def old_ui():
    """Serve the old UI"""
    return render_template_string(HTML_TEMPLATE)


@app.route('/<path:filename>')
def serve_static(filename):
    """Serve static files from workspace root"""
    return send_from_directory(WORKSPACE_ROOT, filename)


# ============================================================
# API Endpoints
# ============================================================

@app.route('/api/start', methods=['POST'])
def api_start():
    return jsonify(iface.start_collection())


@app.route('/api/stop', methods=['POST'])
def api_stop():
    return jsonify(iface.stop_collection())


@app.route('/api/save', methods=['POST'])
def api_save():
    return jsonify(iface.save_pointcloud())


@app.route('/api/status')
def api_status():
    return jsonify(iface.get_status())


@app.route('/api/logs')
def api_logs():
    return jsonify({"logs": iface.log_messages})


@app.route('/api/files')
def api_files():
    return jsonify(iface.get_recent_files())


# ============================================================
# AUTO START Pipeline API
# ============================================================

@app.route('/api/auto_start', methods=['POST'])
def api_auto_start():
    """Start the AUTO START pipeline"""
    if iface.pipeline_running:
        return jsonify({"success": False, "message": "Pipeline already running"})

    # Run pipeline in background thread
    thread = threading.Thread(target=iface.run_auto_pipeline)
    thread.daemon = True
    thread.start()

    return jsonify({"success": True, "message": "Auto pipeline started"})


@app.route('/api/auto_stop', methods=['POST'])
def api_auto_stop():
    """Stop the AUTO START pipeline and reset state"""
    result = iface.stop_pipeline()
    return jsonify(result)


@app.route('/api/progress')
def api_progress():
    """Get current pipeline progress"""
    return jsonify({
        "progress": iface.pipeline_progress,
        "stage": iface.current_stage,
        "message": iface.current_message,
        "running": iface.pipeline_running,
        "error": iface.pipeline_error
    })


@app.route('/api/visualization/<viz_type>')
def api_visualization(viz_type):
    """Get visualization data"""
    if viz_type == 'registration':
        # Return registration result PCD info
        merged_pcd = os.path.join(WORKSPACE_ROOT, "merged_world.pcd")
        cropped_pcd = os.path.join(WORKSPACE_ROOT, "cropped_world.pcd")

        if os.path.exists(cropped_pcd):
            return jsonify({
                "success": True,
                "file": "cropped_world.pcd",
                "path": cropped_pcd,
                "exists": True
            })
        elif os.path.exists(merged_pcd):
            return jsonify({
                "success": True,
                "file": "merged_world.pcd",
                "path": merged_pcd,
                "exists": True
            })
        else:
            return jsonify({"success": False, "message": "No registration output found"})

    elif viz_type == 'target':
        # Return latest target JSON
        iktarget_dir = os.path.join(WORKSPACE_ROOT, "iktarget")
        if os.path.exists(iktarget_dir):
            json_files = glob.glob(os.path.join(iktarget_dir, "robot_path_mm_*.json"))
            if json_files:
                latest = max(json_files, key=os.path.getmtime)
                try:
                    with open(latest, 'r') as f:
                        data = json.load(f)
                    return jsonify({
                        "success": True,
                        "file": os.path.basename(latest),
                        "data": data
                    })
                except Exception as e:
                    return jsonify({"success": False, "message": str(e)})

        return jsonify({"success": False, "message": "No target JSON found"})

    else:
        return jsonify({"error": "Unknown visualization type"}), 404


@app.route('/api/pcd_data')
def api_pcd_data():
    """Get point cloud data as JSON for Three.js rendering (view.py style)

    Returns dual lidar point clouds with colors:
    - Lidar1 (Left): Red color
    - Lidar2 (Right): Blue color
    """
    try:
        import open3d as o3d
        import numpy as np

        # Find the merged or cropped PCD file
        cropped_pcd = os.path.join(WORKSPACE_ROOT, "cropped_world.pcd")
        merged_pcd = os.path.join(WORKSPACE_ROOT, "merged_world.pcd")
        downsampled_pcd = os.path.join(WORKSPACE_ROOT, "cropped_world_downsampled.pcd")

        pcd_path = None
        if os.path.exists(downsampled_pcd):
            pcd_path = downsampled_pcd
        elif os.path.exists(cropped_pcd):
            pcd_path = cropped_pcd
        elif os.path.exists(merged_pcd):
            pcd_path = merged_pcd

        if pcd_path is None:
            return jsonify({"success": False, "message": "No PCD file found"})

        # Load point cloud
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points)
        has_colors = pcd.has_colors()

        if has_colors:
            colors = np.asarray(pcd.colors)
        else:
            colors = None

        # Downsample for web if too many points (max 100k points for performance)
        max_points = 100000
        if len(points) > max_points:
            indices = np.random.choice(len(points), max_points, replace=False)
            points = points[indices]
            if colors is not None:
                colors = colors[indices]

        # Compute bounding box for normalization
        center = points.mean(axis=0)
        points_centered = points - center
        scale = np.abs(points_centered).max()
        if scale > 0:
            points_normalized = points_centered / scale
        else:
            points_normalized = points_centered

        # If has colors (from view.py: red for Lidar1, blue for Lidar2)
        # The colors should already be set by view.py (red=[1,0.3,0.3], blue=[0.3,0.5,1])
        # If no colors, assign based on some heuristic or use gradient
        if colors is None:
            # Generate gradient colors based on position
            colors = np.zeros((len(points), 3))
            for i in range(len(points)):
                z_norm = (points_normalized[i, 2] + 1) / 2
                # Magenta to cyan gradient
                colors[i] = [1.0 - z_norm * 0.5, z_norm * 0.5, 1.0]

        # Prepare response data
        response_data = {
            "success": True,
            "file": os.path.basename(pcd_path),
            "num_points": len(points),
            "positions": points_normalized.flatten().tolist(),
            "colors": colors.flatten().tolist(),
            "center": center.tolist(),
            "scale": float(scale)
        }

        return jsonify(response_data)

    except ImportError:
        return jsonify({"success": False, "message": "Open3D not installed"})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route('/api/target_data')
def api_target_data():
    """Get target path data as JSON for Three.js rendering (paview.py style)

    Returns path data with:
    - robot_positions: Robot positions [robot_x, robot_y, robot_z]
    - surface_points: Surface contact points [surface_x, surface_y, surface_z]
    - line_indices: Array of line data with start/end indices for drawing connected lines

    Data format from paview.py: [robot_x, robot_y, robot_z, surface_x, surface_y, surface_z]
    """
    try:
        iktarget_dir = os.path.join(WORKSPACE_ROOT, "iktarget")
        if not os.path.exists(iktarget_dir):
            return jsonify({"success": False, "message": "iktarget directory not found"})

        json_files = glob.glob(os.path.join(iktarget_dir, "robot_path_mm_*.json"))
        if not json_files:
            return jsonify({"success": False, "message": "No target JSON found"})

        latest = max(json_files, key=os.path.getmtime)

        with open(latest, 'r') as f:
            data = json.load(f)

        # Extract robot positions and surface points (like paview.py)
        all_robot_positions = []
        all_surface_points = []
        line_indices = []  # [(start, end), ...] for each line

        for line in data.get('lines', []):
            start_idx = len(all_robot_positions)
            for pt in line:
                # Point format: [robot_x, robot_y, robot_z, surface_x, surface_y, surface_z, ...]
                if len(pt) >= 6:
                    all_robot_positions.append(pt[:3])    # robot position
                    all_surface_points.append(pt[3:6])    # surface contact point
            end_idx = len(all_robot_positions)
            if end_idx > start_idx:
                line_indices.append([start_idx, end_idx])

        if not all_robot_positions:
            return jsonify({"success": False, "message": "No points in target data"})

        import numpy as np
        robot_positions = np.array(all_robot_positions)
        surface_points = np.array(all_surface_points)

        # Normalize coordinates for visualization (use surface points as reference)
        # Combine both for proper centering
        all_points = np.vstack([robot_positions, surface_points])
        center = all_points.mean(axis=0)
        scale = np.abs(all_points - center).max()
        if scale == 0:
            scale = 1.0

        robot_normalized = (robot_positions - center) / scale
        surface_normalized = (surface_points - center) / scale

        return jsonify({
            "success": True,
            "file": os.path.basename(latest),
            "num_lines": data.get('num_lines', 0),
            "total_points": len(all_robot_positions),
            "robot_positions": robot_normalized.flatten().tolist(),
            "surface_points": surface_normalized.flatten().tolist(),
            "line_indices": line_indices,  # For drawing connected lines
            "center": center.tolist(),
            "scale": float(scale)
        })

    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


def main():
    global iface

    print("=" * 60)
    print("Dual LiDAR Web Interface with AUTO START Pipeline")
    print("=" * 60)

    # Initialize interface
    iface = DualLidarInterface()

    print(f"Workspace root: {WORKSPACE_ROOT}")
    print("Starting web server on http://0.0.0.0:5020")
    print("")
    print("Available URLs:")
    print("  - http://localhost:5020      (New UI if index.html exists)")
    print("  - http://localhost:5020/new  (New UI)")
    print("  - http://localhost:5020/old  (Old UI)")
    print("=" * 60)

    # Run Flask app
    app.run(host='0.0.0.0', port=5020, debug=False, threaded=True)


if __name__ == '__main__':
    main()
