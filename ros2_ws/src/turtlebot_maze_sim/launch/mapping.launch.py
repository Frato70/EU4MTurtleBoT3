
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_sim = get_package_share_directory('turtlebot_maze_sim')

    configuration_directory = os.path.join(
        pkg_sim,
        'config',
        'cartographer'
    )

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory',
            configuration_directory,
            '-configuration_basename',
            'turtlebot3_lds_2d.lua',
        ],
    )

    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-resolution',
            LaunchConfiguration('resolution'),
            '-publish_period_sec',
            '1.0',
        ],
    )

    cartographer_odometry = Node(
    package='turtlebot_maze_sim',
    executable='cartographer_odometry.py',
    name='cartographer_odometry',
    output='screen',
    parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'resolution',
            default_value='0.03',
            description='Occupancy grid resolution in meters'
        ),
        cartographer,
        occupancy_grid,
        cartographer_odometry,
    ])
