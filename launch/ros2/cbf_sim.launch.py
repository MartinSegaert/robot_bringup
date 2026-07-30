from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    declare_cfg_arg = DeclareLaunchArgument(
        'cfg',
        default_value='cbf_sim.yaml',
        description='Config path relative to robot_bringup/config/ros2'
    )
    
    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='rmf',
        description='Robot name to use in topic remappings'
    )
    
    # Create LaunchConfiguration variables
    cfg = LaunchConfiguration('cfg')
    robot_name = LaunchConfiguration('robot_name')
    
    # Create the full path to the config file
    cfg_file = PathJoinSubstitution([
        get_package_share_directory('robot_bringup'),
        'config',
        'ros2',
        cfg
    ])
    
    node = Node(
        package='composite_cbf',
        executable='composite_cbf_node',
        name='composite_cbf',
        parameters=[
            cfg_file,
            {'use_sim_time': True}
        ],
        remappings=[
            ('~/obstacles', '/cbf_pc_selector/output_pc'),
            ('~/odom', [PythonExpression(["'/'", " + '", robot_name, "' + '/odom'"])]),
            ('~/safe_cmd_twist', [PythonExpression(["'/'", " + '", robot_name, "' + '/cmd/acc'"])]),
            # ('~/safe_cmd_twist', '/cbf/cmd/acc'),
            ('~/cmd_in', '/sdf_nmpc/cmd/acc'),
            # ('~/cmd_in', [PythonExpression(["'/'", " + '", robot_name, "' + '/cmd/acc'"])]),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_cfg_arg,
        declare_robot_name_arg,
        node
    ])
