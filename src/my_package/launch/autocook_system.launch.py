# launch/autocook_system.launch.py

import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command

def generate_launch_description():
    urdf_file=os.path.join(get_package_share_directory('robot_description'),'urdf','autocook_robot.urdf.xacro')
    robot_description=Command(['xacro',urdf_file])


    return LaunchDescription([
        # Launch webcam detection service
        Node(
            package='robot_state_publisher',
            exectuable = 'robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description':robot_description}],
        ),
        Node(
            package='my_package',
            executable='webcamNode.py',
            name='webcam_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
        
        # Launch recipe planning service
        Node(
            package='my_package',
            executable='planningNode.py',
            name='planning_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
        # Delay control node to ensure services are ready
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='my_package',
                    executable='motionControlNode.py',
                    name='motion_control_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                    }]
                )
            ]
        ),
        
        
        # Delay control node to ensure services are ready
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='my_package',
                    executable='controlNode.py',
                    name='control_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                    }]
                )
            ]
        ),
        
        # Launch TF2 static transform publisher for robot base
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
            output='log'
        ),
    ])

