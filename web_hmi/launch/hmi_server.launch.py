"""
Launch file for the Web HMI server.

Usage:
    ros2 launch web_hmi hmi_server.launch.py
    ros2 launch web_hmi hmi_server.launch.py plan_file:=/path/to/coverage_plan.json
    ros2 launch web_hmi hmi_server.launch.py port:=8080 host:=0.0.0.0
    ros2 launch web_hmi hmi_server.launch.py stamped:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    plan_file_arg = DeclareLaunchArgument(
        'plan_file',
        default_value='coverage_plan.json',
        description='Path to coverage plan JSON file'
    )

    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='Host address to bind the server to'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='5002',
        description='Port number for the web server'
    )

    stamped_arg = DeclareLaunchArgument(
        'stamped',
        default_value='false',
        description='Publish cmd_vel as TwistStamped instead of Twist'
    )

    _common_args = [
        LaunchConfiguration('plan_file'),
        '--host', LaunchConfiguration('host'),
        '--port', LaunchConfiguration('port'),
    ]

    # HMI server node — TwistStamped variant
    hmi_server_stamped = Node(
        package='web_hmi',
        executable='hmi_server',
        name='hmi_server',
        output='screen',
        arguments=_common_args + ['--stamped'],
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('stamped')),
    )

    # HMI server node — plain Twist variant (default)
    hmi_server_node = Node(
        package='web_hmi',
        executable='hmi_server',
        name='hmi_server',
        output='screen',
        arguments=_common_args,
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('stamped')),
    )

    return LaunchDescription([
        plan_file_arg,
        host_arg,
        port_arg,
        stamped_arg,
        hmi_server_stamped,
        hmi_server_node,
    ])
