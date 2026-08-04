from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_waypoint_file = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config',
        'ros2',
        'waypoint_follower',
        'waypoints.yaml',
    ])

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'waypoint_file', default_value=default_waypoint_file
        ),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose'),
        DeclareLaunchArgument('odometry_topic', default_value='/rmf/odom'),
        DeclareLaunchArgument('reached_distance', default_value='2.0'),
    ]

    follower = Node(
        package='robot_bringup',
        executable='waypoint_follower.py',
        name='waypoint_follower',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool
            ),
            'waypoint_file': LaunchConfiguration('waypoint_file'),
            'autostart': ParameterValue(
                LaunchConfiguration('autostart'), value_type=bool
            ),
            'goal_topic': LaunchConfiguration('goal_topic'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'reached_distance': ParameterValue(
                LaunchConfiguration('reached_distance'), value_type=float
            ),
        }],
    )

    return LaunchDescription(arguments + [follower])
