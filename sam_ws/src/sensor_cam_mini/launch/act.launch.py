from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='sensor_cam_mini', executable='MultiActuator.py', output='screen', additional_env={'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'})
    ])
