#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import std_msgs.msg as std_msgs
import numpy as np
import yaml
import time
import threading
from typing import List, Dict, Optional

# gt3_rainbow의 cobot.py 임포트
try:
    from gt3_rainbow.cobot import *
except ImportError:
    print("Warning: gt3_rainbow.cobot not found, trying direct import")
    from cobot import *

class RobotMotionController(Node):
    """ROS2 node for controlling Rainbow Robotics RB10-1300 robot from YAML files"""
    
    def __init__(self, node_name='robot_motion_controller_v2'):
        try:
            super().__init__(node_name)
            print(f"Creating RobotMotionController with name: {node_name}")
            
            # Robot state
            self.robot_connected = False
            self.current_tcp_pose = None
            self.current_joint_angles = None
            self.robot_state = None
            self.is_executing_motion = False
            
            # Motion data
            self.motion_data = None
            self.waypoints = []
            self.current_waypoint_idx = 0
            
            # Safety parameters
            self.max_linear_speed = 250.0  # mm/s
            self.max_acceleration = 500.0   # mm/s^2
            self.safety_distance = 0.01     # 10mm minimum distance
            
            # cobot.py 연결 상태
            self.cobot_connected = False
            
            # Publishers
            self.robot_cmd_pub = self.create_publisher(
                std_msgs.String, 
                '/robot_pose', 
                10
            )
            
            # Subscribers
            self.robot_status_sub = self.create_subscription(
                std_msgs.String,
                '/current_pose',
                self.robot_status_callback,
                10
            )
            
            # Timer for connection check
            self.connection_timer = self.create_timer(1.0, self.check_connection)
            
            self.get_logger().info(f'{node_name} initialized')
            
        except Exception as e:
            print(f"Error in RobotMotionController.__init__: {e}")
            import traceback
            traceback.print_exc()
            raise
    
    def check_connection(self):
        """Check cobot connection status"""
        if self.cobot_connected:
            # Check if cobot is still connected
            if IsCommandSockConnect() and IsDataSockConnect():
                # Update robot state
                pose_data = GetCurreJP()
                if pose_data:
                    msg = std_msgs.String()
                    msg.data = pose_data
                    self.robot_status_pub.publish(msg)
            else:
                self.get_logger().warn("Cobot connection lost")
                self.cobot_connected = False
    
    def robot_status_callback(self, msg):
        """Handle robot status updates"""
        try:
            # Parse status from cobot.py format
            parts = msg.data.split(';')
            if len(parts) >= 3:
                self.current_tcp_pose = eval(parts[0])
                self.current_joint_angles = eval(parts[1])
                self.robot_state = int(parts[2])
                
                # Check if robot is idle (state == 1)
                if self.robot_state == 1 and self.is_executing_motion:
                    # Robot finished current motion
                    self.execute_next_waypoint()
                    
        except Exception as e:
            self.get_logger().error(f'Error parsing robot status: {e}')
    
    def execute_next_waypoint(self):
        """Execute next waypoint in sequence"""
        if self.current_waypoint_idx < len(self.waypoints):
            self.send_move_command(self.waypoints[self.current_waypoint_idx])
            self.current_waypoint_idx += 1
        else:
            self.is_executing_motion = False
            self.get_logger().info("Motion sequence completed")
            
    def connect_robot(self):
        """Connect to Rainbow Robotics robot using cobot.py"""
        try:
            # IP 주소는 환경변수나 파라미터로 받아야 하지만, 일단 하드코딩
            robot_ip = '192.168.1.13'  # 실제 로봇 IP로 변경
            
            self.get_logger().info(f'Connecting to robot at {robot_ip}...')
            
            # cobot.py의 ConnectToCB 함수 사용
            result = ConnectToCB(robot_ip)
            
            if result:
                self.cobot_connected = True
                self.robot_connected = True
                
                # 프로그램 모드를 REAL로 설정
                SetProgramMode(PG_MODE.REAL)
                
                self.get_logger().info('Successfully connected to Rainbow robot')
                time.sleep(2)  # 연결 안정화 대기
                return True
            else:
                self.get_logger().error('Failed to connect to robot')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Connection error: {e}')
            import traceback
            traceback.print_exc()
            return False
    
    def disconnect_robot(self):
        """Disconnect from robot"""
        try:
            if self.cobot_connected:
                DisConnectToCB()
                self.cobot_connected = False
                self.robot_connected = False
                self.get_logger().info('Disconnected from robot')
        except Exception as e:
            self.get_logger().error(f'Disconnect error: {e}')
            
    def execute_pb_add_motion(self) -> bool:
        """Execute motion using PB Add method with cobot.py"""
        if not self.cobot_connected:
            self.get_logger().error('Robot not connected')
            return False
        
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded')
            return False
        
        # Validate waypoints
        valid_waypoints = []
        for wp in self.waypoints:
            if self.validate_waypoint(wp):
                valid_waypoints.append(wp)
            else:
                self.get_logger().error(f"Skipping invalid waypoint: {wp.get('name', 'unknown')}")
        
        if len(valid_waypoints) < 2:
            self.get_logger().error(f'PB Add motion requires at least 2 valid waypoints, got {len(valid_waypoints)}')
            return False
        
        self.get_logger().info(f'Starting PB Add motion with {len(valid_waypoints)} waypoints')
        
        try:
            # Step 1: Clear previous point buffer
            MovePB_Clear()
            time.sleep(0.1)
            
            # Step 2: Add all waypoints to point buffer
            for i, wp in enumerate(valid_waypoints):
                success = self.add_waypoint_to_pb(wp, i)
                if not success:
                    self.get_logger().error(f"Failed to add waypoint {i}: {wp.get('name', 'unknown')}")
                    return False
                time.sleep(0.05)  # Small delay between adds
            
            # Step 3: Execute point buffer motion
            motion_params = self.motion_data.get('motion_parameters', {})
            acceleration = motion_params.get('acceleration_mm_s2', 200.0)
            blend_type = motion_params.get('blend_type', 'INTENDED')
            
            # cobot.py의 BLEND_RTYPE enum 사용
            rtype = BLEND_RTYPE.INTENDED if blend_type == 'INTENDED' else BLEND_RTYPE.CONSTANT
            
            # Execute PB motion
            result = MovePB_Run(acceleration, rtype)
            
            if result:
                self.get_logger().info(f'PB Add motion started with {len(valid_waypoints)} waypoints')
                self.is_executing_motion = True
                return True
            else:
                self.get_logger().error('Failed to start PB Add motion')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error in PB Add motion execution: {e}')
            import traceback
            traceback.print_exc()
            return False

    def add_waypoint_to_pb(self, waypoint: Dict, index: int) -> bool:
        """Add waypoint to point buffer using cobot.py"""
        try:
            pose = waypoint.get('pose', {})
            position = pose.get('position', {})
            orientation = pose.get('orientation', {})
            
            # Extract values
            x = position.get('x', 0)
            y = position.get('y', 0)
            z = position.get('z', 0)
            rx = orientation.get('rx', 0)
            ry = orientation.get('ry', 0)
            rz = orientation.get('rz', 0)
            
            speed = waypoint.get('speed', 100)
            blend_radius = waypoint.get('blend_radius', 5.0)
            blend_option = waypoint.get('blend_option', 'DISTANCE')
            
            # Determine blend option using cobot.py enum
            b_option = BLEND_OPTION.DISTANCE if blend_option == 'DISTANCE' else BLEND_OPTION.RATIO
            
            # Use cobot.py's MovePB_Add function
            result = MovePB_Add(x, y, z, rx, ry, rz, speed, b_option, blend_radius)
            
            self.get_logger().info(f"Added waypoint {index+1}/{len(self.waypoints)} to PB: "
                                f"({x:.1f}, {y:.1f}, {z:.1f}) mm at {speed} mm/s, "
                                f"blend: {blend_radius}")
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error adding waypoint to PB: {e}")
            import traceback
            traceback.print_exc()
            return False

    def execute_motion_with_method(self, method: str = 'pb_add') -> bool:
        """Execute motion with specified method"""
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded')
            return False
        
        waypoint_count = len(self.waypoints)
        
        if method == 'pb_add' and waypoint_count >= 2:
            self.get_logger().info(f"Executing PB Add motion with {waypoint_count} waypoints")
            return self.execute_pb_add_motion()
        elif method == 'itpl' and waypoint_count >= 2:
            self.get_logger().info(f"Executing ITPL motion with {waypoint_count} waypoints")
            return self.execute_interpolated_motion()
        else:
            self.get_logger().info(f"Executing individual motion with {waypoint_count} waypoints")
            return self.execute_motion()
    
    def execute_motion(self) -> bool:
        """Execute the loaded motion using individual move commands"""
        if not self.cobot_connected:
            self.get_logger().error('Robot not connected')
            return False
        
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded')
            return False
        
        # Validate all waypoints
        valid_waypoints = []
        for wp in self.waypoints:
            if self.validate_waypoint(wp):
                valid_waypoints.append(wp)
            else:
                self.get_logger().error(f"Skipping invalid waypoint: {wp.get('name', 'unknown')}")
        
        if not valid_waypoints:
            self.get_logger().error('No valid waypoints to execute')
            return False
        
        self.waypoints = valid_waypoints
        self.is_executing_motion = True
        self.current_waypoint_idx = 0
        
        # Start with first waypoint
        self.execute_next_waypoint()
        
        return True

    def execute_interpolated_motion(self) -> bool:
        """Execute motion using interpolated path with cobot.py"""
        if not self.cobot_connected:
            self.get_logger().error('Robot not connected')
            return False
        
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded')
            return False
        
        # Validate waypoints
        valid_waypoints = []
        for wp in self.waypoints:
            if self.validate_waypoint(wp):
                valid_waypoints.append(wp)
        
        if len(valid_waypoints) < 2:
            self.get_logger().error(f'Interpolated motion requires at least 2 valid waypoints, got {len(valid_waypoints)}')
            return self.execute_motion()
        
        self.get_logger().info(f'Starting interpolated motion with {len(valid_waypoints)} waypoints')
        
        # Clear previous motion
        MoveITPL_Clear()
        time.sleep(0.2)
        
        # Add all waypoints to interpolated motion
        for i, wp in enumerate(valid_waypoints):
            pose = wp.get('pose', {})
            position = pose.get('position', {})
            orientation = pose.get('orientation', {})
            
            x = position.get('x', 0)
            y = position.get('y', 0)
            z = position.get('z', 0)
            rx = orientation.get('rx', 0)
            ry = orientation.get('ry', 0)
            rz = orientation.get('rz', 0)
            speed = wp.get('speed', 100)
            
            # Use cobot.py's MoveITPL_Add
            MoveITPL_Add(x, y, z, rx, ry, rz, speed, i)
            time.sleep(0.05)
            
            self.get_logger().info(f"Added waypoint {i+1}/{len(valid_waypoints)} to interpolated motion")
        
        # Execute interpolated motion
        time.sleep(0.2)
        
        # Use cobot.py's MoveITPL_Run
        result = MoveITPL_Run(self.max_acceleration, ITPL_RTYPE.CA_INTENDED, False)
        
        self.get_logger().info(f'Executing interpolated motion with {len(valid_waypoints)} waypoints')
        return result

    def send_move_command(self, waypoint: Dict):
        """Send individual move command to robot using cobot.py"""
        try:
            pose = waypoint.get('pose', {})
            position = pose.get('position', {})
            orientation = pose.get('orientation', {})
            
            # Extract values
            x = position.get('x', 0)
            y = position.get('y', 0)
            z = position.get('z', 0)
            rx = orientation.get('rx', 0)
            ry = orientation.get('ry', 0)
            rz = orientation.get('rz', 0)
            
            speed = waypoint.get('speed', 100)
            acc = min(self.max_acceleration, 300)
            
            # Use cobot.py's MoveL function
            result = MoveL(x, y, z, rx, ry, rz, speed, acc)
            
            self.get_logger().info(f"Sent move_l: ({x:.1f}, {y:.1f}, {z:.1f}) mm at {speed} mm/s")
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error sending move command: {e}")
            import traceback
            traceback.print_exc()
            return False

    def load_motion_from_yaml(self, yaml_file_path: str) -> bool:
        """Load motion data from YAML file with execution mode detection"""
        try:
            with open(yaml_file_path, 'r') as f:
                self.motion_data = yaml.safe_load(f)
            
            # Determine execution mode
            motion_info = self.motion_data.get('motion_info', {})
            self.execution_mode = motion_info.get('execution_mode', 'individual')
            
            # Load waypoints
            if 'waypoints' in self.motion_data:
                self.waypoints = self.motion_data.get('waypoints', [])
                self.get_logger().info(f"Loaded structured YAML format")
            else:
                self.get_logger().error("Unknown YAML format")
                return False
            
            # Log motion info
            self.get_logger().info(f"Loaded motion: {motion_info.get('name', 'Unknown')}")
            self.get_logger().info(f"Total waypoints: {len(self.waypoints)}")
            self.get_logger().info(f"Motion type: {motion_info.get('motion_type', 'Unknown')}")
            self.get_logger().info(f"Execution mode: {self.execution_mode}")
            
            if len(self.waypoints) == 0:
                self.get_logger().error("No valid waypoints found in YAML file")
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to load motion YAML: {e}')
            import traceback
            traceback.print_exc()
            return False

    def validate_waypoint(self, waypoint: Dict) -> bool:
        """Validate waypoint for safety"""
        try:
            pose = waypoint.get('pose', {})
            pos = pose.get('position', {})
            
            # Extract position values in mm
            x = pos.get('x', 0) / 1000.0  # Convert to meters for validation
            y = pos.get('y', 0) / 1000.0
            z = pos.get('z', 0) / 1000.0
            
            # Check workspace limits (example values, adjust for your robot)
            if not all(-1.3 <= p <= 1.3 for p in [x, y]):  # X, Y limits
                self.get_logger().warn(f"Waypoint {waypoint.get('name', 'unknown')} outside XY workspace")
                return False
                
            if not (0.0 <= z <= 1.5):  # Z limit
                self.get_logger().warn(f"Waypoint {waypoint.get('name', 'unknown')} outside Z workspace")
                return False
            
            # Check distance from robot base
            distance = np.sqrt(x**2 + y**2)
            if distance < 0.3 or distance > 1.3:
                self.get_logger().warn(f"Waypoint {waypoint.get('name', 'unknown')} too close/far from base")
                return False
            
            # Check speed limits
            speed = waypoint.get('speed', 100)
            if speed > self.max_linear_speed:
                self.get_logger().warn(f"Waypoint speed {speed} exceeds maximum {self.max_linear_speed}")
                waypoint['speed'] = self.max_linear_speed
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error validating waypoint: {e}")
            return False

    def stop_motion(self):
        """Stop current motion execution"""
        self.is_executing_motion = False
        self.current_waypoint_idx = 0
        
        # Use cobot.py's MotionHalt
        if self.cobot_connected:
            MotionHalt()
            
        self.get_logger().info('Motion stopped')
    
    def set_gripper(self, state: bool):
        """Control gripper state"""
        if self.cobot_connected:
            if state:
                CBDigitalOut(0.0, DOUT_SET.HIGH)
            else:
                CBDigitalOut(0.0, DOUT_SET.LOW)
            
            self.get_logger().info(f'Gripper {"opened" if state else "closed"}')


class RobotMotionExecutor:
    """High-level robot motion executor for YAML-based motion"""
    
    def __init__(self):
        self.controller = None
        self.executor = None
        self.thread = None
        self.initialized = False
        
    def initialize(self):
        """Initialize ROS2 and controller"""
        if self.initialized:
            print("RobotMotionExecutor already initialized")
            return
            
        try:
            # ROS2가 이미 초기화되었는지 확인
            if not rclpy.ok():
                print("Initializing rclpy...")
                rclpy.init()
            else:
                print("rclpy already initialized")
            
            # 새로운 노드 생성 시 고유한 이름 사용
            import time
            node_name = f'robot_motion_controller_v2_{int(time.time() * 1000) % 100000}'
            self.controller = RobotMotionController(node_name)
            
            self.executor = rclpy.executors.SingleThreadedExecutor()
            self.executor.add_node(self.controller)
            
            # Run executor in separate thread
            self.thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.thread.start()
            
            time.sleep(0.5)  # Allow time for initialization
            self.initialized = True
            print(f"RobotMotionExecutor initialized with node: {node_name}")
            
        except Exception as e:
            print(f"Error initializing RobotMotionExecutor: {e}")
            import traceback
            traceback.print_exc()
            self.initialized = False
    
    def connect_robot(self) -> bool:
        """Connect to robot"""
        if not self.initialized:
            print("RobotMotionExecutor not initialized")
            return False
            
        if self.controller:
            return self.controller.connect_robot()
        return False
    
    def disconnect_robot(self):
        """Disconnect from robot"""
        if self.controller:
            self.controller.disconnect_robot()
    
    def load_motion_yaml(self, yaml_file: str) -> bool:
        """Load motion from YAML file"""
        if not self.initialized:
            print("RobotMotionExecutor not initialized. Initializing now...")
            self.initialize()
            
        if self.controller:
            try:
                return self.controller.load_motion_from_yaml(yaml_file)
            except Exception as e:
                print(f"Error loading YAML file: {e}")
                import traceback
                traceback.print_exc()
                return False
        return False
    
    def execute_motion(self, method: str = None) -> bool:
        """Execute loaded motion with specified or auto-detected method"""
        if self.controller:
            # Auto-detect method if not specified
            if method is None:
                motion_info = self.controller.motion_data.get('motion_info', {}) if self.controller.motion_data else {}
                method = motion_info.get('execution_mode', 'individual')
            
            return self.controller.execute_motion_with_method(method)
        return False
    
    def stop_motion(self):
        """Stop motion execution"""
        if self.controller:
            self.controller.stop_motion()
    
    def set_gripper(self, state: bool):
        """Set gripper state"""
        if self.controller:
            self.controller.set_gripper(state)
    
    def shutdown(self):
        """Shutdown executor"""
        try:
            if self.controller:
                self.controller.disconnect_robot()
                
            if self.executor:
                self.executor.shutdown()
                
            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=2.0)
                
            self.initialized = False
        except Exception as e:
            print(f"Error during shutdown: {e}")


# Test execution
if __name__ == "__main__":
    executor = RobotMotionExecutor()
    executor.initialize()
    
    try:
        # Connect to robot
        print("Connecting to robot...")
        if executor.connect_robot():
            print("Robot connected successfully")
            time.sleep(2)
            
            # Load motion from YAML
            yaml_file = "robot_motion_20250611_120000.yaml"  # Example filename
            print(f"Loading motion from {yaml_file}...")
            
            if executor.load_motion_yaml(yaml_file):
                print("Motion loaded successfully")
                
                # Execute motion
                print("Executing motion...")
                if executor.execute_motion():
                    print("Motion execution started")
                else:
                    print("Failed to execute motion")
            else:
                print("Failed to load motion")
        else:
            print("Failed to connect to robot")
        
        # Keep running
        input("Press Enter to stop...")
        
    finally:
        executor.stop_motion()
        executor.disconnect_robot()
        executor.shutdown()
        rclpy.shutdown()