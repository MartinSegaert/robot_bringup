from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        get_package_share_directory('robot_bringup'),
        'config',
        'ros2',
        'custom',
        'custom_global_planner.yaml',
    ])
    arguments = [
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the planner parameter YAML file',
        ),
    ]

    planner = Node(
        package='robot_bringup',
        executable='straight_line_global_planner.py',
        name='straight_line_global_planner',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription(arguments + [planner])
