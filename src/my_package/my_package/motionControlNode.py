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

# MoveIt2 imports
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import moveit_msgs.msg
from moveit_msgs.msg import DisplayTrajectory, RobotState

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
        self.esp32_command_pub = self.create_publisher(String, 'esp32_commands', 10)  # For debugging

        # ESP32 Serial Communication
        try:
            self.esp32_serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info('ESP32 serial connection established')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            self.esp32_serial = None

        # Initialize MoveIt
        self.setup_moveit()

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

    def setup_moveit(self):
        """Initialize MoveIt components"""
        self.get_logger().info('Initializing MoveIt components...')
        
        try:
            # Initialize MoveIt commander objects
            self.robot = RobotCommander()
            self.scene = PlanningSceneInterface()
            
            # Set up planning groups (adjust names based on your robot URDF)
            self.arm_group_name = "arm"  # Change to your arm group name
            self.gripper_group_name = "gripper"  # Change to your gripper group name
            
            self.arm_group = MoveGroupCommander(self.arm_group_name)
            
            # Configure planning parameters
            self.arm_group.set_planning_time(10.0)
            self.arm_group.set_num_planning_attempts(5)
            self.arm_group.set_max_velocity_scaling_factor(0.5)
            self.arm_group.set_max_acceleration_scaling_factor(0.5)
            self.arm_group.set_pose_reference_frame("base_link")  # Adjust frame name
            
            # Initialize gripper group if available
            try:
                self.gripper_group = MoveGroupCommander(self.gripper_group_name)
                self.get_logger().info('Gripper group initialized')
            except:
                self.gripper_group = None
                self.get_logger().warn('Gripper group not found - using manual control')
            
            # Get robot information
            planning_frame = self.arm_group.get_planning_frame()
            eef_link = self.arm_group.get_end_effector_link()
            group_names = self.robot.get_group_names()
            
            self.get_logger().info(f'Planning frame: {planning_frame}')
            self.get_logger().info(f'End effector: {eef_link}')
            self.get_logger().info(f'Planning groups: {group_names}')
            
            # Move to home position on startup
            self.move_to_home_position()
            
        except Exception as e:
            self.get_logger().error(f'MoveIt initialization failed: {e}')
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
            action = request.action  # "open", "close", "pick", "place"
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
            tool_name = request.tool_name  # "gripper", "knife", "spoon", "none"
            
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
        """Execute motion using MoveIt and send to ESP32"""
        try:
            self.is_moving = True
            self.publish_status(f"Planning motion to ({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})")
            
            # Set pose target
            self.arm_group.set_pose_target(pose)
            
            # Plan motion
            plan = self.arm_group.plan()
            
            # Handle different ROS2 versions
            if isinstance(plan, tuple):
                success, trajectory, planning_time, error_code = plan
            else:
                success = len(plan.joint_trajectory.points) > 0
                trajectory = plan
            
            if not success:
                self.get_logger().error('Motion planning failed')
                self.is_moving = False
                return False
            
            self.publish_status('Executing planned motion')
            
            # Execute motion in MoveIt
            execute_success = self.arm_group.execute(trajectory, wait=True)
            
            if execute_success:
                # Get final joint states
                joint_values = self.arm_group.get_current_joint_values()
                
                # Send joint commands to ESP32
                esp32_success = self.send_joints_to_esp32(joint_values, action_type)
                
                if esp32_success:
                    self.publish_joint_states(joint_values)
                    self.publish_status(f'Motion completed: {action_type}')
                    self.is_moving = False
                    return True
                else:
                    self.get_logger().warn('ESP32 communication failed but MoveIt motion completed')
                    self.is_moving = False
                    return True  # Still return true since MoveIt succeeded
            else:
                self.get_logger().error('Motion execution failed')
                self.is_moving = False
                return False
                
        except Exception as e:
            self.get_logger().error(f'Motion execution error: {e}')
            self.is_moving = False
            return False
        finally:
            self.arm_group.clear_pose_targets()

    def send_joints_to_esp32(self, joint_values, action_type="move"):
        """Send joint angles to ESP32"""
        if not self.esp32_serial:
            self.get_logger().warn('ESP32 not connected, skipping hardware command')
            return True  # Return true to not block simulation
        
        try:
            # Create command dictionary
            command = {
                'action': action_type,
                'joints': {
                    'J1': round(np.degrees(joint_values[0]), 2),
                    'J2': round(np.degrees(joint_values[1]), 2),
                    'J3': round(np.degrees(joint_values[2]), 2),
                    'J4': round(np.degrees(joint_values[3]), 2),
                    'J5': round(np.degrees(joint_values[4]), 2),
                    'J6': round(np.degrees(joint_values[5]), 2) if len(joint_values) > 5 else 0.0
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
        try:
            if self.gripper_group and action in ["open", "close"]:
                # Use MoveIt for gripper control
                target_name = "open" if action == "open" else "close"
                self.gripper_group.set_named_target(target_name)
                
                plan = self.gripper_group.plan()
                if isinstance(plan, tuple):
                    success, trajectory, _, _ = plan
                else:
                    success = len(plan.joint_trajectory.points) > 0
                    trajectory = plan
                
                if success:
                    result = self.gripper_group.execute(trajectory, wait=True)
                    if result:
                        self.publish_status(f'Gripper {action} via MoveIt')
                        return True
            
            # Fallback to direct ESP32 control
            return self.send_gripper_to_esp32(action)
            
        except Exception as e:
            self.get_logger().error(f'Gripper action error: {e}')
            return False

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
            # Try named target first
            self.arm_group.set_named_target("home")
            plan = self.arm_group.plan()
            
            if isinstance(plan, tuple):
                success, trajectory, _, _ = plan
            else:
                success = len(plan.joint_trajectory.points) > 0
                trajectory = plan
            
            if success:
                self.arm_group.execute(trajectory, wait=True)
                self.get_logger().info('Moved to home position')
            else:
                # Fallback to joint target
                joint_goal = self.arm_group.get_current_joint_values()
                joint_goal[0] = 0.0  # Base
                joint_goal[1] = -1.57  # Shoulder  
                joint_goal[2] = 1.57   # Elbow
                joint_goal[3] = 0.0    # Wrist pitch
                joint_goal[4] = 0.0    # Wrist roll
                if len(joint_goal) > 5:
                    joint_goal[5] = 0.0    # End effector
                
                self.arm_group.go(joint_goal, wait=True)
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
