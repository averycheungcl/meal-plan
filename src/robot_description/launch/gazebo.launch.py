import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_path = get_package_share_directory('robot_description')
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_path, 'urdf', 'autocook_robot.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
        )
    )
    
    # Static transform publisher (replaces tf static_transform_publisher)
    tf_footprint_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=['-file', urdf_file, '-entity', 'autocook_robot'],
        output='screen'
    )
    
    # Fake joint calibration (using ros2 topic pub)
    fake_joint_calibration = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/calibrated', 'std_msgs/msg/Bool', '{data: true}', '--once'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        tf_footprint_base,
        robot_state_publisher,
        spawn_robot,
        fake_joint_calibration
    ])