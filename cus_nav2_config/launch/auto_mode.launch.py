"""
Auto Mode Launch File
=====================
Launches the autonomous coverage stack:
  - map_server           (serves the pre-built map)
  - amcl                 (Monte-Carlo localisation)
  - lifecycle_manager    (manages map_server + amcl)
  - ros2_plan_publisher  (publishes interpolated coverage path from JSON)
  - coverage_path_follower (generates /updated_path for cus_pure_pursuit)

NOTE: The nav2 controller_server (Regulated Pure Pursuit) is intentionally
      NOT launched here.  cus_pose_compute and cus_pure_pursuit are started
      by the HMI on the "Start Robot" and "Start Coverage" button presses
      respectively, giving per-button control over the custom stack.

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
    pkg_dir = get_package_share_directory("cus_nav2_config")

    # ── Launch arguments ──────────────────────────────────────────────────────

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_dir, "maps", "map123.yaml"),
        description="Full path to the map yaml file",
    )

    plan_file_arg = DeclareLaunchArgument(
        "plan_file",
        default_value=os.path.join(pkg_dir, "paths", "coverage_plan.json"),
        description="Full path to the coverage plan JSON file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time (set true when running with Gazebo)",
    )

    # ── Params files ──────────────────────────────────────────────────────────
    localization_params = os.path.join(pkg_dir, "params", "localization.yaml")

    # ── Localisation nodes ────────────────────────────────────────────────────

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            localization_params,
            {
                "yaml_filename": LaunchConfiguration("map"),
                "use_sim_time":  LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            localization_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart":    True,
            "node_names":   ["map_server", "amcl"],
        }],
    )

    # ── Coverage plan publisher ───────────────────────────────────────────────

    plan_publisher_node = Node(
        package="cus_nav2_config",
        executable="ros2_plan_publisher",
        name="coverage_plan_publisher",
        output="screen",
        parameters=[{
            "plan_file":       LaunchConfiguration("plan_file"),
            "use_sim_time":    LaunchConfiguration("use_sim_time"),
            "max_interp_dist": 0.2,
        }],
    )

    # ── Coverage path follower (generates /updated_path for cus_pure_pursuit) ─

    path_follower_node = Node(
        package="cus_nav2_config",
        executable="coverage_path_follower",
        name="coverage_path_follower",
        output="screen",
        parameters=[{
            "use_sim_time":    LaunchConfiguration("use_sim_time"),
            "max_interp_dist": 0.2,
        }],
    )

    # ── Static TF: base_link → laser (real robot only) ───────────────────────
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_laser",
        output="screen",
        arguments=["0.11", "0", "0.15", "0", "0", "3.14159", "base_link", "laser"],
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
    )

    return LaunchDescription([
        map_arg,
        plan_file_arg,
        use_sim_time_arg,
        # Localisation
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
        # Coverage
        plan_publisher_node,
        path_follower_node,
        # TF (real robot only)
        static_tf_node,
    ])
