"""
Auto Mode Launch File
=====================
Launches the full autonomous coverage stack:
  - map_server        (serves the pre-built map)
  - amcl              (Monte-Carlo localisation)
  - lifecycle_manager (manages map_server + amcl lifecycle)
  - controller_server (Regulated Pure Pursuit)
  - lifecycle_manager (manages controller_server lifecycle)
  - ros2_plan_publisher  (publishes coverage path from JSON plan)
  - coverage_path_follower (bridges plan path → /follow_path action)

Usage:
  ros2 launch cus_nav2_config auto_mode.launch.py
  ros2 launch cus_nav2_config auto_mode.launch.py map:=/path/to/map.yaml
  ros2 launch cus_nav2_config auto_mode.launch.py plan_file:=/path/to/plan.json
  ros2 launch cus_nav2_config auto_mode.launch.py use_sim_time:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('cus_nav2_config')

    # ── Launch arguments ──────────────────────────────────────────────────────

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'map123.yaml'),
        description='Full path to the map yaml file',
    )

    plan_file_arg = DeclareLaunchArgument(
        'plan_file',
        default_value=os.path.join(pkg_dir, 'paths', 'coverage_plan.json'),
        description='Full path to the coverage plan JSON file',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set true when running with Gazebo)',
    )

    stamped_cmd_vel_arg = DeclareLaunchArgument(
        'stamped_cmd_vel',
        default_value='false',
        description='Publish cmd_vel as TwistStamped (required by some simulators)',
    )

    # ── Params files ──────────────────────────────────────────────────────────
    localization_params = os.path.join(pkg_dir, 'params', 'localization.yaml')
    nav_params          = os.path.join(pkg_dir, 'params', 'pure_pus.yaml')

    # ── Localisation nodes ────────────────────────────────────────────────────

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            localization_params,
            {
                'yaml_filename':  LaunchConfiguration('map'),
                'use_sim_time':   LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            localization_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart':    True,
            'node_names':   ['map_server', 'amcl'],
        }],
    )

    # ── Navigation / controller nodes ─────────────────────────────────────────

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            nav_params,
            {
                'use_sim_time':          LaunchConfiguration('use_sim_time'),
                'enable_stamped_cmd_vel': LaunchConfiguration('stamped_cmd_vel'),
            },
        ],
        remappings=[('cmd_vel', 'cmd_vel')],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart':    True,
            'node_names':   ['controller_server'],
        }],
    )

    # ── Coverage plan publisher ───────────────────────────────────────────────

    plan_publisher_node = Node(
        package='cus_nav2_config',
        executable='ros2_plan_publisher',
        name='coverage_plan_publisher',
        output='screen',
        parameters=[{
            'plan_file':   LaunchConfiguration('plan_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_hz':  1.0,
        }],
    )

    # ── Coverage path follower (bridge → /follow_path action) ─────────────────

    path_follower_node = Node(
        package='cus_nav2_config',
        executable='coverage_path_follower',
        name='coverage_path_follower',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ── Static TF: base_link → laser (real robot only, not needed in sim) ──────
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        output='screen',
        arguments=['0.25', '0', '0.15', '0', '0', '0', 'base_link', 'laser'],
        condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
    )

    return LaunchDescription([
        map_arg,
        plan_file_arg,
        use_sim_time_arg,
        stamped_cmd_vel_arg,
        # Localisation
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
        # Navigation
        controller_server_node,
        lifecycle_manager_navigation,
        # Coverage
        plan_publisher_node,
        path_follower_node,
        # TF (real robot only)
        static_tf_node,
    ])
