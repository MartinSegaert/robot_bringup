from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    ns = LaunchConfiguration('ns')
    input_image = LaunchConfiguration('input_image')
    odometry = LaunchConfiguration('odometry')
    output_topic = LaunchConfiguration('output_topic')
    joystick_topic = LaunchConfiguration('joystick_topic')
    viz_sdf_2d = LaunchConfiguration('viz_sdf_2d')
    viz_sdf_3d = LaunchConfiguration('viz_sdf_3d')
    enable_sdf_nodes = LaunchConfiguration('enable_sdf_nodes')
    cfg_file = LaunchConfiguration('cfg_file')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('ns', default_value='sdf_nmpc'),
        DeclareLaunchArgument('input_image', default_value='/rmf/lidar/range'),
        DeclareLaunchArgument('odometry', default_value='/rmf/odom'),
        DeclareLaunchArgument('output_topic', default_value='/sdf_nmpc/cmd/acc'),
        DeclareLaunchArgument('joystick_topic', default_value='/sdf_nmpc/joystick'),
        DeclareLaunchArgument('enable_sdf_nodes', default_value='true'),
        DeclareLaunchArgument('viz_sdf_2d', default_value='true'),
        DeclareLaunchArgument('viz_sdf_3d', default_value='true'),
        DeclareLaunchArgument(
            'cfg_file',
            default_value='/workspace/src/robot_bringup/config/ros2/nmpc_sim_lidar_goal_velocity.yaml',
        ),
    ]

    common_params = [{
        'cfg': cfg_file,
        'use_sim_time': use_sim_time,
    }]

    group = GroupAction([
        PushRosNamespace(ns),
        SetRemap(src='odometry', dst=odometry),
        SetRemap(src='observation', dst=input_image),
        SetRemap(src='cmd/acc', dst=output_topic),
        SetRemap(src='joystick', dst=joystick_topic),
        Node(
            package='sdf_nmpc_ros',
            executable='vae_node.py',
            name='vae',
            parameters=common_params,
            condition=IfCondition(enable_sdf_nodes),
            output='screen',
        ),
        Node(
            package='sdf_nmpc_ros',
            executable='ref_gen_node.py',
            name='ref_gen',
            parameters=common_params,
            output='screen',
        ),
        Node(
            package='sdf_nmpc_ros',
            executable='sdfnmpc_node.py',
            name='sdfnmpc',
            parameters=common_params,
            output='screen',
        ),
        Node(
            package='sdf_nmpc_ros',
            executable='viz_sdf_3D_node.py',
            name='viz_sdf_3D',
            parameters=common_params,
            condition=IfCondition(viz_sdf_3d),
            output='screen',
        ),
        Node(
            package='sdf_nmpc_ros',
            executable='viz_sdf_2D_node.py',
            name='viz_sdf_2D',
            parameters=common_params,
            condition=IfCondition(viz_sdf_2d),
            output='screen',
        )
    ])

    return LaunchDescription(declare_args + [group])
