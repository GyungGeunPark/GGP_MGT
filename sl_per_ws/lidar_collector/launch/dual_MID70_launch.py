#!/usr/bin/env python3
"""
Dual Livox LiDAR Mid-70 Launch File
- Left LiDAR:  3GGDN8700225691
- Right LiDAR: 3GGDN8700226431
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

################### User Configuration ###################
# Livox driver parameters
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 1    # 1-Each LiDAR publishes to separate topic (IMPORTANT!)
data_src      = 0    # 0-lidar
publish_freq  = 10.0 # Publish frequency in Hz
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

# Get config file path
cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'dual_MID70_config.json')

# PCD save paths (relative to working directory)
save_path_left = './pcd_file/left'
save_path_right = './pcd_file/right'

# LiDAR topic names (based on broadcast code)
# When multi_topic=1, topics are named: /livox/lidar_1_<broadcast_code>
left_topic = '/livox/lidar_1_3GGDN8700225691'
right_topic = '/livox/lidar_1_3GGDN8700226431'
##########################################################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]

dual_lidar_params = [
    {"left_topic": left_topic},
    {"right_topic": right_topic},
    {"save_path_left": save_path_left},
    {"save_path_right": save_path_right},
    {"accumulate_duration": 5.0},
    {"use_mm_scale": True},           # Convert to mm (bundle program format)
    {"correct_right_yaw": False},     # Yaw correction (disabled)
    # {"right_yaw_correction": 180.0},  # [COMMENTED OUT] Not used when correct_right_yaw=False
    # [COMMENTED OUT] Bounding box filter - disabled
    {"use_bounding_box": False},
    # {"left_x_max": 3000.0},           # mm
    # {"left_y_min": -500.0},           # mm
    # {"left_y_max": 1000.0},           # mm
    # {"right_x_max": 3000.0},          # mm
    # {"right_y_min": -1000.0},         # mm
    # {"right_y_max": 500.0}            # mm
]


def generate_launch_description():
    # Livox ROS2 Driver Node (Changed from livox_ros_driver2!)
    livox_driver = Node(
        package='livox_ros2_driver',         # << CHANGED for Mid-70
        executable='livox_ros2_driver_node', # << CHANGED for Mid-70
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    # Dual LiDAR Collector Node (unchanged)
    dual_lidar_node = Node(
        package='lidar_collector',
        executable='dual_lidar_node',
        name='dual_lidar_collector',
        output='screen',
        parameters=dual_lidar_params
    )

    return LaunchDescription([
        livox_driver,
        dual_lidar_node
    ])
