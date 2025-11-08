#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # RealSense camera node with fixed parameters
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'serial_no': '',
            'usb_port_id': '',
            'device_type': 'd455',
            
            # Point cloud settings
            'pointcloud.enable': True,
            'pointcloud.stream_filter': 2,
            'pointcloud.stream_index_filter': 0,
            'pointcloud.ordered_pc': False,
            'pointcloud.allow_no_texture_points': False,
            
            # Stream settings
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_sync': True,
            'align_depth.enable': True,
            
            # Filter settings
            'colorizer.enable': False,
            'decimation_filter.enable': False,
            'spatial_filter.enable': False,
            'temporal_filter.enable': False,
            'hole_filling_filter.enable': False,
            
            # Resolution settings
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '1280x720x30',
            
            # Additional stability settings
            'depth_module.emitter_enabled': 1,
            'depth_module.exposure': 8500,
            'depth_module.gain': 16,
            'depth_module.enable_auto_exposure': True,
            
            'use_sim_time': use_sim_time,
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # Vision-based robot control system V4 (메인 GUI) - realsense_viewerv2.py 사용
    vision_robot_gui_v4 = TimerAction(
        period=5.0,  # 카메라 노드가 안정화된 후 시작
        actions=[
            Node(
                package='realsense_robot_control',
                executable='realsense_viewer_v4',  # V4 실행파일 사용
                name='vision_robot_control_gui_v4',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'max_points': 100000,
                }],
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                # X11 환경변수 설정
                additional_env={
                    'DISPLAY': os.environ.get('DISPLAY', ':0'),
                    'XAUTHORITY': os.environ.get('XAUTHORITY', '/root/.Xauthority'),
                    'QT_QPA_PLATFORM': 'xcb',
                    'QT_X11_NO_MITSHM': '1',
                    'XDG_RUNTIME_DIR': '/tmp/runtime-root',
                }
            )
        ]
    )
    
    # Robot interface node
    robot_interface = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='realsense_robot_control',
                executable='cobot_interface',
                name='rainbow_robot_interface',
                parameters=[{
                    'robot_ip': '192.168.1.13',
                    'use_sim_time': use_sim_time,
                }],
                output='screen',
                respawn=True,
                respawn_delay=2.0
            )
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        realsense_node,
        vision_robot_gui_v4,
        robot_interface,
    ])