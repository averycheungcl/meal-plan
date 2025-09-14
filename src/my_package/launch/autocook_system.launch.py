# launch/autocook_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Launch webcam detection service
        Node(
            package='my_package',
            executable='webcamNode',
            name='webcam_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
        
        # Launch recipe planning service
        Node(
            package='my_package',
            executable='planningNode',
            name='planning_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
        
        # Delay control node to ensure services are ready
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='my_package',
                    executable='controlNode',
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


# launch/individual_nodes.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'node_name',
            default_value='webcamNode',
            description='Name of the node to launch'
        ),
        
        # Launch single node based on argument
        Node(
            package='my_package',
            executable=LaunchConfiguration('node_name'),
            name=LaunchConfiguration('node_name'),
            output='screen',
        ),
    ])


# launch/debug_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch all nodes with debug output
        Node(
            package='my_package',
            executable='webcamNode',
            name='webcam_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        
        Node(
            package='my_package',
            executable='planningNode',
            name='planning_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        
        Node(
            package='my_package',
            executable='controlNode',
            name='control_node',
            output='screen',
            arguments=['--ros-args', '--log-level', 'DEBUG']
        ),
        
        # RQT graph for visualization
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph',
            output='screen'
        ),
    ])

'''

Need add this to  cmakelist

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

At bottom: 
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
'''
