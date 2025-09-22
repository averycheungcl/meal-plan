# launch/autocook_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Launch webcam detection service
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
            output='screen'
        ),
    ])

