from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir     = get_package_share_directory('cus_nav2_config')
    obs_pkg_dir = get_package_share_directory('obstacle_detector')

    map_file = os.path.join(pkg_dir, 'maps', 'map123.yaml')
    params_file = os.path.join(pkg_dir, 'params', 'localization.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[params_file, {'yaml_filename': map_file}],
            output='screen'
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),
        Node(
            package='obstacle_detector',
            executable='obstacle_detector_node',
            name='obstacle_detector_node',
            output='screen',
            parameters=[os.path.join(obs_pkg_dir, 'config', 'params.yaml')],
        ),
    ])

