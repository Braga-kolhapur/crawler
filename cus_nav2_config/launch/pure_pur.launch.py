import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Path to your params
    params_file = os.path.join(
        get_package_share_directory('custom_nav2_config'),
        'params',
        'pure_pus.yaml'
    )

    # Nav2 bringup launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false'
        }.items(),
    )

    return LaunchDescription([
        nav2_launch
    ])

