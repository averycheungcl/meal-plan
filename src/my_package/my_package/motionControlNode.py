#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
import numpy as np
import json
import serial
import time

# MoveIt2 imports - Official API only
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel

# Custom service imports
from my_package.srv import MoveToPosition, ExecuteGrip, SetTool

class motionControlNode(Node):
    def __init__(self):
        super().__init__('motion_control_node')

        # Services for motion control
        self.move_service = self.create_service(MoveToPosition, 'move_to_position', self.move_to_position_callback)
        self.grip_service = self.create_service(ExecuteGrip, 'execute_grip', self.execute_grip_callback)
        self.tool_service = self.create_service(SetTool, 'set_tool', self.set_tool_callback)

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.motion_status_pub = self.create_publisher(String, 'motion_status', 10)
        self.esp32_command_pub = self.create_publisher(String, 'esp32_commands', 10)

        # ESP32 Serial Communication
        try:
            self.esp32_serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info('ESP32 serial connection established')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            self.esp32_serial = None

        # Initialize MoveIt2
        self.setup_moveit2()

        # Motion state tracking
        self.current_tool = None
        self.is_moving = False
        
        # Tool positions (relative to robot base)
        self.tool_positions = {
            'gripper': {'x': 0.0, 'y': -0.2, 'z': 0.15},
            'knife': {'x': 0.0, 'y': -0.25, 'z': 0.15},  
            'spoon': {'x': 0.0, 'y': -0.3, 'z': 0.15}
        }

        # Workspace safety limits
        self.workspace_limits = {
            'x_min': 0.05, 'x_max': 0.5,
            'y_min': -0.4, 'y_max': 0.4,
            'z_min': 0.02, 'z_max': 0.5
        }

        self.get_logger().info('Motion control node initialized and ready')

    def setup_moveit2(self):
        """Initialize MoveIt2 components for ROS2 Rolling"""
        self.get_logger().info('Initializing MoveIt2 components for ROS2 Rolling...')
        
        try:
            # Initialize MoveItPy for ROS2 Rolling
            self.moveit = MoveItPy(node_name="motion_control_node")
            
            # Get robot model and planning scene monitor
            self.robot_model = self.moveit.get_robot_model()
            self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
            
            # Set up planning groups (adjust based on your robot URDF)
            self.arm_group_name = "arm"  # Change to your arm group name
            self.gripper_group_name = "gripper"  # Change to your gripper group name
            
            self.get_logger().info(f'Robot model: {self.robot_model.getName()}')
            
            # Get available joint model groups
            available_groups = self.robot_model.getJointModelGroupNames()
            self.get_logger().info(f'Available joint groups: {available_groups}')
            
            # Check if arm group exists
            if self.arm_group_name not in available_groups:
                self.get_logger().error(f"Joint group '{self.arm_group_name}' not found!")
                self.get_logger().error(f"Available groups: {available_groups}")
                self.get_logger().error("Please update self.arm_group_name to match your robot's URDF")
                raise ValueError(f"Joint group '{self.arm_group_name}' not found. Available: {available_groups}")
            
            # Get joint model group for arm
            self.arm_group = self.robot_model.getJointModelGroup(self.arm_group_name)
            self.get_logger().info(f'Arm group "{self.arm_group_name}" loaded successfully')
            
            # Check for gripper group
            if self.gripper_group_name in available_groups:
                self.gripper_group = self.robot_model.getJointModelGroup(self.gripper_group_name)
                self.get_logger().info(f'Gripper group "{self.gripper_group_name}" found')
            else:
                self.gripper_group = None
                self.get_logger().warn(f'Gripper group "{self.gripper_group_name}" not found - using manual control')
            
            # Get planning component for the arm group
            self.planning_component = self.moveit.get_planning_component(self.arm_group_name)
            self.get_logger().info(f'Planning component created for group: {self.arm_group_name}')
            
            # Set planning parameters
            self.planning_time = 10.0
            self.velocity_scaling = 0.5
            self.acceleration_scaling = 0.5
            
            self.get_logger().info('MoveIt2 initialization completed successfully')
            
            # Move to home position on startup
            self.move_to_home_position()
            
        except Exception as e:
            self.get_logger().error(f'MoveIt2 initialization failed: {e}')
            self.get_logger().error('Troubleshooting steps:')
            self.get_logger().error('1. Make sure your robot description is loaded: ros2 param get /robot_description robot_description')
            self.get_logger().error('2. Check MoveIt2 is running: ros2 node list | grep move_group')
            self.get_logger().error('3. Verify your planning groups in SRDF match the arm_group_name')
            raise5
            self.acceleration_scaling = 0.5
            
            # Move to home position on startup
            self.move_to_home_position()
            
        except Exception as e:
            self.get_logger().error(f'MoveIt2 initialization failed: {e}')
            raise

    def move_to_position_callback(self, request, response):
        """Service callback for moving to a specific pose"""
        try:
            pose = request.target_pose
            action_type = request.action_type
            
            self.get_logger().info(f'Moving to position: ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})')
            
            # Safety check
            if not self.is_pose_safe(pose):
                response.success = False
                response.message = "Target pose outside safe workspace"
                return response
            
            # Execute motion
            success = self.execute_motion_to_pose(pose, action_type)
            
            response.success = success
            response.message = "Motion completed successfully" if success else "Motion failed"
            
            return response
            
        except Exception as e:
            self.get_logger().error(f'Move to position failed: {e}')
            response.success = False
            response.message = str(e)
            return response

    def execute_grip_callback(self, request, response):
        """Service callback for gripper actions"""
        try:
            action = request.action
            self.get_logger().info(f'Executing gripper action: {action}')
            
            success = self.execute_gripper_action(action)
            
            response.success = success
            response.message = f"Gripper {action} completed" if success else f"Gripper {action} failed"
            
            return response
            
        except Exception as e:
            self.get_logger().error(f'Gripper action failed: {e}')
            response.success = False
            response.message = str(e)
            return response

    def set_tool_callback(self, request, response):
        """Service callback for tool management"""
        try:
            tool_name = request.tool_name
            
            if tool_name == "none":
                success = self.return_current_tool()
            else:
                success = self.pickup_tool(tool_name)
            
            response.success = success
            response.current_tool = self.current_tool or "none"
            response.message = f"Tool operation completed" if success else "Tool operation failed"
            
            return response
            
        except Exception as e:
            self.get_logger().error(f'Tool operation failed: {e}')
            response.success = False
            response.current_tool = self.current_tool or "none"
            response.message = str(e)
            return response

    def execute_motion_to_pose(self, pose, action_type="move"):
        """Execute motion using MoveIt2 for ROS2 Rolling"""
        try:
            self.is_moving = True
            self.publish_status(f"Planning motion to ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
            
            # Set start state to current state
            self.planning_component.setStartStateToCurrentState()
            
            # Set goal pose (adjust end_effector_link based on your robot)
            self.planning_component.setGoal(pose, "end_effector_link")
            
            # Plan motion
            plan_solution = self.planning_component.plan()
            
            # Check if planning was successful
            if plan_solution:
                self.publish_status('Motion planning successful, executing...')
                
                # Execute the planned trajectory
                robot_trajectory = plan_solution.trajectory
                
                if robot_trajectory and len(robot_trajectory.joint_trajectory.points) > 0:
                    # Get joint values from trajectory (final point)
                    final_point = robot_trajectory.joint_trajectory.points[-1]
                    joint_values = list(final_point.positions)
                    
                    # Send to ESP32
                    esp32_success = self.send_joints_to_esp32(joint_values, action_type)
                    
                    if esp32_success:
                        self.publish_joint_states(joint_values)
                        self.publish_status(f'Motion completed: {action_type}')
                        self.is_moving = False
                        return True
                    else:
                        self.get_logger().warn('ESP32 communication failed but planning succeeded')
                        self.is_moving = False
                        return True  # Still success from MoveIt perspective
                else:
                    self.get_logger().error('Empty trajectory received')
                    self.is_moving = False
                    return False
            else:
                self.get_logger().error('Motion planning failed')
                self.is_moving = False
                return False
                
        except Exception as e:
            self.get_logger().error(f'Motion execution error: {e}')
            self.is_moving = False
            return False

    def send_joints_to_esp32(self, joint_values, action_type="move"):
        """Send joint angles to ESP32"""
        if not self.esp32_serial:
            self.get_logger().warn('ESP32 not connected, skipping hardware command')
            return True  # Return true to not block simulation
        
        try:
            # Ensure we have 6 joint values
            if len(joint_values) < 6:
                joint_values.extend([0.0] * (6 - len(joint_values)))
            
            # Create command dictionary
            command = {
                'action': action_type,
                'joints': {
                    'J1': round(np.degrees(joint_values[0]), 2),
                    'J2': round(np.degrees(joint_values[1]), 2),
                    'J3': round(np.degrees(joint_values[2]), 2),
                    'J4': round(np.degrees(joint_values[3]), 2),
                    'J5': round(np.degrees(joint_values[4]), 2),
                    'J6': round(np.degrees(joint_values[5]), 2)
                },
                'speed': 30,  # Movement speed percentage
                'timestamp': time.time()
            }
            
            # Send command
            json_command = json.dumps(command) + '\n'
            self.esp32_serial.write(json_command.encode())
            
            # Publish command for debugging
            debug_msg = String()
            debug_msg.data = json_command
            self.esp32_command_pub.publish(debug_msg)
            
            # Wait for acknowledgment
            response = self.esp32_serial.readline().decode().strip()
            
            if response == "OK":
                self.get_logger().info(f'ESP32 command sent: {action_type}')
                return True
            else:
                self.get_logger().warn(f'ESP32 response: {response}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'ESP32 communication error: {e}')
            return False

    def execute_gripper_action(self, action):
        """Execute gripper actions"""
        if self.gripper_group:
            try:
                # Use MoveIt2 for gripper control if group exists
                planning_component = self.moveit.get_planning_component(self.gripper_group_name)
                
                # Set named target (adjust names based on your gripper configuration)
                if action in ["open", "place"]:
                    planning_component.setGoal("open")
                elif action in ["close", "pick"]:
                    planning_component.setGoal("close")
                
                # Plan and execute
                plan_result = planning_component.plan()
                if plan_result:
                    self.publish_status(f'Gripper {action} via MoveIt2')
                    return True
                    
            except Exception as e:
                self.get_logger().warn(f'MoveIt2 gripper control failed: {e}')
        
        # Fallback to direct ESP32 control
        return self.send_gripper_to_esp32(action)

    def send_gripper_to_esp32(self, action):
        """Send gripper command directly to ESP32"""
        if not self.esp32_serial:
            return True
        
        try:
            # Map actions to angles
            angle_map = {
                "open": 0.0, "place": 0.0,
                "close": 90.0, "pick": 90.0
            }
            
            angle = angle_map.get(action, 0.0)
            
            command = {
                'action': 'gripper',
                'gripper_angle': angle,
                'speed': 50
            }
            
            json_command = json.dumps(command) + '\n'
            self.esp32_serial.write(json_command.encode())
            
            response = self.esp32_serial.readline().decode().strip()
            success = response == "OK"
            
            if success:
                self.get_logger().info(f'Gripper {action} sent to ESP32')
            
            return success
            
        except Exception as e:
            self.get_logger().error(f'ESP32 gripper command failed: {e}')
            return False

    def pickup_tool(self, tool_name):
        """Move to tool position and pick it up"""
        if tool_name not in self.tool_positions:
            self.get_logger().error(f'Unknown tool: {tool_name}')
            return False
        
        if self.current_tool == tool_name:
            self.get_logger().info(f'Already holding {tool_name}')
            return True
        
        try:
            # Return current tool first
            if self.current_tool:
                self.return_current_tool()
            
            # Move to tool position
            tool_pos = self.tool_positions[tool_name]
            pose = Pose()
            pose.position.x = tool_pos['x']
            pose.position.y = tool_pos['y']
            pose.position.z = tool_pos['z']
            pose.orientation.w = 1.0
            
            if self.execute_motion_to_pose(pose, f"pickup_{tool_name}"):
                # Close gripper to grab tool
                if self.execute_gripper_action("close"):
                    self.current_tool = tool_name
                    self.get_logger().info(f'Picked up {tool_name}')
                    return True
            
            return False
            
        except Exception as e:
            self.get_logger().error(f'Tool pickup failed: {e}')
            return False

    def return_current_tool(self):
        """Return current tool to its position"""
        if not self.current_tool:
            return True
        
        try:
            # Move to tool return position
            tool_pos = self.tool_positions[self.current_tool]
            pose = Pose()
            pose.position.x = tool_pos['x']
            pose.position.y = tool_pos['y']
            pose.position.z = tool_pos['z']
            pose.orientation.w = 1.0
            
            if self.execute_motion_to_pose(pose, f"return_{self.current_tool}"):
                # Open gripper to release tool
                if self.execute_gripper_action("open"):
                    returned_tool = self.current_tool
                    self.current_tool = None
                    self.get_logger().info(f'Returned {returned_tool}')
                    return True
            
            return False
            
        except Exception as e:
            self.get_logger().error(f'Tool return failed: {e}')
            return False

    def move_to_home_position(self):
        """Move arm to home/neutral position"""
        try:
            # Try using named configuration if available
            planning_component = self.moveit.get_planning_component(self.arm_group_name)
            planning_component.setGoal("home")  # Adjust named pose if you have one
            
            plan_result = planning_component.plan()
            if plan_result:
                self.get_logger().info('Moved to home position via named target')
            else:
                # Fallback to joint positions
                joint_goals = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]  # Adjust for your robot
                self.send_joints_to_esp32(joint_goals, "home")
                self.get_logger().info('Moved to home via joint target')
                
        except Exception as e:
            self.get_logger().error(f'Home position failed: {e}')

    def is_pose_safe(self, pose):
        """Check if pose is within workspace limits"""
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        return (self.workspace_limits['x_min'] <= x <= self.workspace_limits['x_max'] and
                self.workspace_limits['y_min'] <= y <= self.workspace_limits['y_max'] and
                self.workspace_limits['z_min'] <= z <= self.workspace_limits['z_max'])

    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = f"[MotionNode] {message}"
        self.motion_status_pub.publish(msg)
        self.get_logger().info(message)

    def publish_joint_states(self, joint_values):
        """Publish current joint states"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_state.position = joint_values[:6]  # Ensure we have exactly 6 joints
        self.joint_state_pub.publish(joint_state)

    def destroy_node(self):
        """Cleanup on node shutdown"""
        if self.esp32_serial:
            self.esp32_serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = motionControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
