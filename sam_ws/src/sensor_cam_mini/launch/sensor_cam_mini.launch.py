#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 카메라 노드가 사용할 포트 번호 인수
    tcp_server_port_arg = DeclareLaunchArgument(
        'tcp_server_port',
        default_value='9100',
        description='Port for the Camera MJPEG streaming server'
    )

    # 카메라 노드 (이제 포트 번호만 인수로 받음)
    camera_node = Node(
        package='sensor_cam_mini',
        executable='camera_node',
        name='camera_node',
        output='screen',
        additional_env={'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'},
        arguments=[
            LaunchConfiguration('tcp_server_port') # IP 인수를 제거하고 포트만 전달
        ]
    )
    
    # 라이다 노드는 파일 공유 방식으로 변경되어 더 이상 인수가 필요 없음
    lidar_node = Node(
        package='sensor_cam_mini',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        additional_env={'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}
        # arguments 리스트가 필요 없으므로 완전히 제거
    )

    return LaunchDescription([
        tcp_server_port_arg,
        camera_node,
        lidar_node
    ])