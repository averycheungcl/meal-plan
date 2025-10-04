# launch/autocook_system.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # URDF via xacro
    urdf_file = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'autocook_robot.urdf.xacro'
    )

    robot_description = Command(['xacro ', urdf_file])

    # Load SRDF
    srdf_file = os.path.join(
        get_package_share_directory('robot_description'),
        'srdf',
        'autocook_robot.srdf'
    )

    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Webcam detection service
        Node(
            package='my_package',
            executable='webcamNode.py',
            name='webcam_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # Recipe planning service
        Node(
            package='my_package',
            executable='planningNode.py',
            name='planning_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        # Motion control node (delayed start)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='my_package',
                    executable='motionControlNode.py',
                    name='motion_control_node',
                    output='screen',
                    parameters=[
                        {'use_sim_time': False},
                        {'robot_description_semantic': robot_description_semantic}
                    ]
                )
            ]
        ),

        # Control node (delayed start)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='my_package',
                    executable='controlNode.py',
                    name='control_node',
                    output='screen',
                    parameters=[{'use_sim_time': False}]
                )
            ]
        ),

        # Camera frame transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_tf',
            arguments=['--x', '0.3', '--y', '0', '--z', '0.5', 
                       '--roll', '0', '--pitch', '1.57', '--yaw', '0',
                       '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
            output='log'
        ),

        # World to base transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_tf',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0', 
                       '--frame-id', 'world', '--child-frame-id', 'base_link'],
            output='log'
        ),
    ])