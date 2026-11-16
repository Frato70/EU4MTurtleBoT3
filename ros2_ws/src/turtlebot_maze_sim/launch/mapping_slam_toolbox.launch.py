"""Online asynchronous SLAM Toolbox mapping."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)


def generate_launch_description():

    pkg_sim = get_package_share_directory(
        'turtlebot_maze_sim'
    )

    pkg_slam = get_package_share_directory(
        'slam_toolbox'
    )

    params_file = os.path.join(
        pkg_sim,
        'config',
        'slam_toolbox',
        'mapper_maze.yaml',
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_slam,
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        slam_toolbox,
    ])