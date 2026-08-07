"""Launch the obstacle-aware scalar reference velocity commander."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'ros2',
        'custom',
        'reference_velocity_commander.yaml',
    )
    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_bringup',
            executable='reference_velocity_commander.py',
            name='reference_velocity_commander',
            parameters=[config, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
