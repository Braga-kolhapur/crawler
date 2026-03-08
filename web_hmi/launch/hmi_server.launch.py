from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='web_hmi',
            executable='hmi_server',
            name='hmi_server',
            output='screen',
            emulate_tty=True,
        ),
    ])
