from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('odometry_topic', default_value='/rmf/odom'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose'),
        DeclareLaunchArgument('path_topic', default_value='/gbplanner_path'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('replan_interval', default_value='2.0'),
    ]

    planner = Node(
        package='robot_bringup',
        executable='straight_line_global_planner.py',
        name='straight_line_global_planner',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'goal_topic': LaunchConfiguration('goal_topic'),
            'path_topic': LaunchConfiguration('path_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'replan_interval': LaunchConfiguration('replan_interval'),
        }],
    )

    return LaunchDescription(arguments + [planner])
