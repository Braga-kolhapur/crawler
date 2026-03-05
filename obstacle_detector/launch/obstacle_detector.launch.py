"""
Obstacle Detector Launch File
==============================
Usage:
  ros2 launch obstacle_detector obstacle_detector.launch.py
  ros2 launch obstacle_detector obstacle_detector.launch.py auto_pause:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir    = get_package_share_directory('obstacle_detector')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )

    node = Node(
        package='obstacle_detector',
        executable='obstacle_detector_node',
        name='obstacle_detector_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        node,
    ])
