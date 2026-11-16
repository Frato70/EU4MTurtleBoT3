"""Autonomous frontier exploration using explore_lite."""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_sim = get_package_share_directory(
        'turtlebot_maze_sim'
    )

    params_file = os.path.join(
        pkg_sim,
        'config',
        'exploration',
        'explore.yaml',
    )

    explorer = Node(

        package='explore_lite',

        executable='explore',

        name='explore',

        output='screen',

        parameters=[
            params_file,
        ],
    )

    return LaunchDescription([
        explorer,
    ])