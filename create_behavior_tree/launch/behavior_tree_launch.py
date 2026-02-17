"""Launch file for behavior tree node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description."""

    # Declare arguments
    tree_update_hz_arg = DeclareLaunchArgument(
        'tree_update_hz',
        default_value='10.0',
        description='Behavior tree update frequency in Hz'
    )

    # Behavior tree node
    behavior_tree_node = Node(
        package='create_behavior_tree',
        executable='behavior_tree_node',
        name='behavior_tree_node',
        output='screen',
        parameters=[
            {'tree_update_hz': LaunchConfiguration('tree_update_hz')},
        ],
        remappings=[
            ('/create/bumper', '/create/bumper'),
            ('/create/cliff', '/create/cliff'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        tree_update_hz_arg,
        behavior_tree_node,
    ])
