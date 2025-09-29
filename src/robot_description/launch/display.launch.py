import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_path = get_package_share_directory('robot_description')
    
    # Path to xacro file
    xacro_file = os.path.join(pkg_path, 'urdf', 'autocook_robot.urdf.xacro')
    
    # Declare launch argument for model path
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=xacro_file,
        description='Path to robot xacro file'
    )
    
    # Process xacro file
    robot_desc = Command(['xacro ', LaunchConfiguration('model')])
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2
    rviz_config_file = os.path.join(pkg_path, 'config', 'urdf.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])