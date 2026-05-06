#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # 새로운 인수 선언
    # tcp_server_ip_arg = DeclareLaunchArgument(
    #     'tcp_server_ip',
    #     default_value='127.0.0.1',
    #     description='IP address of the TCP server'
    # )
    
    # tcp_server_port_arg = DeclareLaunchArgument(
    #     'tcp_server_port',
    #     default_value='9100',
    #     description='Port of the TCP server'
    # )

    # # Flask 앱 노드 (포트 인수 전달)
    # flaskapp_node = Node(
    #     package='sensor_cam_mini',
    #     executable='flaskapp.py',
    #     name='flaskapp',
    #     output='screen',
    #     additional_env={'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'},
    #     arguments=[LaunchConfiguration('tcp_server_port')]
    # )

    return LaunchDescription([
        # tcp_server_ip_arg,
        # tcp_server_port_arg,
        # flaskapp_node
    ])