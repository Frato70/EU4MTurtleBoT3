"""Nav2 navigation for TurtleBot3 maze simulation.

Localization is provided by SLAM Toolbox.
"""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_sim = get_package_share_directory(
        'turtlebot_maze_sim'
    )

    pkg_nav2 = get_package_share_directory(
        'nav2_bringup'
    )

    params_file = os.path.join(
        pkg_sim,
        'config',
        'nav2',
        'burger_maze.yaml',
    )

    use_sim_time = LaunchConfiguration(
        'use_sim_time'
    )

    autostart = LaunchConfiguration(
        'autostart'
    )

    nav2 = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_nav2,
                'launch',
                'navigation_launch.py',
            )
        ),

        launch_arguments={

            'use_sim_time': use_sim_time,

            'params_file': params_file,

            'autostart': autostart,

            'use_composition': 'False',

        }.items(),
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
        ),

        nav2,

    ])