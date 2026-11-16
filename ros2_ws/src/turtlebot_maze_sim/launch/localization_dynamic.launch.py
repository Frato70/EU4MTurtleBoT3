
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory(
        'turtlebot_maze_sim'
    )

    default_params_file = os.path.join(
        pkg_share,
        'config',
        'nav2',
        'burger_maze_dynamic.yaml'
    )

    default_map_file = os.path.join(
        pkg_share,
        'maps',
        'ground_truth',
        'maze_gt.yaml'
    )

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Navigation parameters file'
    )

    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Path to the ground truth map'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='false'
    )

    # ------------------------------------------
    # Map Server
    # ------------------------------------------

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file
            }
        ]
    )

    # ------------------------------------------
    # AMCL
    # ------------------------------------------

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time
            }
        ]
    )

    # ------------------------------------------
    # Lifecycle Manager
    # ------------------------------------------

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'map_server',
                    'amcl'
                ],
                'bond_timeout': 0.0,
                'attempt_respawn_reconnection': False
            }
        ]
    )

    return LaunchDescription([
        declare_params_file,
        declare_map_file,
        declare_use_sim_time,
        declare_autostart,

        map_server,
        amcl,
        lifecycle_manager
    ])
