#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Optimized launch file for RealSense D455 camera and viewer
    포인트 클라우드 성능 최적화 설정 - 포인트 수 제한 없음
    """
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # RealSense camera node - 최적화된 설정
    camera_params = {
        'device_type': 'd455',
        'serial_no': '',
        
        # 스트림 동기화 활성화
        'enable_sync': True,
        
        # 스트림 활성화
        'enable_color': True,
        'enable_depth': True,
        'enable_infra': False,
        'enable_infra1': False,
        'enable_infra2': False,
        'enable_gyro': False,
        'enable_accel': False,
        
        # 해상도 설정 - 성능과 품질의 균형
        'depth_module.depth_profile': '640x480x15',  # 더 높은 해상도와 프레임레이트
        'rgb_camera.color_profile': '640x480x15',
        
        # 포인트클라우드 최적화
        'pointcloud.enable': True,
        'pointcloud.ordered_pc': True,
        'pointcloud.allow_no_texture_points': True,  # 일단 true로 변경하여 모든 포인트 표시
        'pointcloud.stream_filter': 2,  # 0: No filter, 1: Depth, 2: Color
        'pointcloud.stream_index_filter': 0,
        'align_depth.enable': True,  # Depth를 Color에 정렬
        
        # 필터 설정 - 품질 향상
        'temporal_filter.enable': True,
        'spatial_filter.enable': True,
        'decimation_filter.enable': False,
        'decimation_filter.filter_magnitude': 2,  # 2x decimation for performance
        
        # 성능 최적화
        'depth_module.exposure': 8500,
        'depth_module.gain': 16,
        'depth_module.enable_auto_exposure': True,
        
        # 추가 최적화
        'hole_filling_filter.enable': False,  # 성능을 위해 비활성화
        'threshold_filter.enable': False,
        
        # 깊이 범위 설정 (mm)
        'depth_module.depth_units': 0.001,  # 1mm 단위
        'clip_distance': 10.0,  # 10미터까지 표시 (확장됨)
        
        'use_sim_time': use_sim_time,
    }
    
    # RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[camera_params],
        output='screen',
        emulate_tty=True,
        # 추가 ROS 파라미터
        remappings=[
            # 토픽 리매핑 (필요시)
        ]
    )
    
    # Viewer node with environment variables
    viewer_node = Node(
        package='realsense_viewer',
        executable='realsense_viewer_node',
        name='realsense_viewer',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_points': 0,  # 0 = 제한 없음 (무제한)
            'skip_frames': 1,  # 프레임 스킵 설정
        }],
        # Qt 환경 변수
        additional_env={
            'QT_QPA_PLATFORM': 'xcb',
            'QT_QPA_PLATFORM_PLUGIN_PATH': '/usr/lib/x86_64-linux-gnu/qt5/plugins',
            'OPENCV_GUI_BACKEND': 'gtk',
            # OpenGL 환경변수 추가
            'QT_OPENGL': 'desktop',
            'MESA_GL_VERSION_OVERRIDE': '3.3',
            # 성능 최적화를 위한 환경변수
            # 'OMP_NUM_THREADS': '4',  # OpenMP 스레드 수
        }
    )
    
    # 추가 노드 (선택사항) - 포인트클라우드 다운샘플링
    # downsample_node = Node(
    #     package='pcl_ros',
    #     executable='voxel_grid_filter',
    #     name='voxel_grid_filter',
    #     parameters=[{
    #         'leaf_size': 0.01,  # 1cm voxel size
    #         'use_sim_time': use_sim_time
    #     }],
    #     remappings=[
    #         ('input', '/camera/camera/depth/color/points'),
    #         ('output', '/camera/camera/depth/color/points_downsampled')
    #     ]
    # )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add actions
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(realsense_node)
    ld.add_action(viewer_node)
    # ld.add_action(downsample_node)  # 필요시 활성화
    
    return ld