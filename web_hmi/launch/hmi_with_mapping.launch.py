"""
Launch file for Web HMI with SLAM mapping support.

This launch file starts:
- HMI web server
- (Optional) SLAM toolbox for mapping

Usage:
    ros2 launch web_hmi hmi_with_mapping.launch.py
    ros2 launch web_hmi hmi_with_mapping.launch.py start_slam:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    plan_file_arg = DeclareLaunchArgument(
        'plan_file',
        default_value='coverage_plan.json',
        description='Path to coverage plan JSON file'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='5002',
        description='Port number for the web server'
    )

    start_slam_arg = DeclareLaunchArgument(
        'start_slam',
        default_value='false',
        description='Whether to start SLAM toolbox automatically'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # HMI server node
    hmi_server_node = Node(
        package='web_hmi',
        executable='hmi_server',
        name='hmi_server',
        output='screen',
        arguments=[
            '--port', LaunchConfiguration('port'),
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        emulate_tty=True,
    )

    # Static TF: base_link → laser (real robot only, not needed in sim)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        output='screen',
        arguments=['0.11', '0', '0.15', '0', '0', '0', 'base_link', 'laser'],
        condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
    )

    # Optional: SLAM toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': PathJoinSubstitution([
                FindPackageShare('cus_nav2_config'),
                'params',
                'mapper_params_online_async.yaml'
            ]),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_slam'))
    )

    return LaunchDescription([
        plan_file_arg,
        port_arg,
        start_slam_arg,
        use_sim_time_arg,
        hmi_server_node,
        static_tf_node,
        slam_toolbox_launch,
    ])
