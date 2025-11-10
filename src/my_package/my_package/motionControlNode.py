#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState as RobotStateMsg
import numpy as np
import json
import serial
import time

# MoveIt2 imports for Rolling
from moveit.planning import (
    MoveItPy,
    PlanningComponent,
)
from moveit.core.robot_state import RobotState

# Custom service imports
from my_package.srv import MoveToPosition, ExecuteGrip, SetTool

class motionControlNode(Node):
    def __init__(self):
        super().__init__('motion_control_node')

        # Services for motion control
        self.move_service = self.create_service(
            MoveToPosition, 'move_to_position', self.move_to_position_callback
        )
        self.grip_service = self.create_service(
            ExecuteGrip, 'execute_grip', self.execute_grip_callback
        )
        self.tool_service = self.create_service(
            SetTool, 'set_tool', self.set_tool_callback
        )

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.motion_status_pub = self.create_publisher(String, 'motion_status', 10)
        self.esp32_command_pub = self.create_publisher(String, 'esp32_commands', 10)

        # ESP32 Serial Communication with retry
        self.esp32_serial = None
        self.esp32_max_retries = 3
        self.connect_esp32()

        # Initialize MoveIt2
        self.moveit_available = False
        try:
            self.setup_moveit2()
            self.moveit_available = True
        except Exception as e:
            self.get_logger().error(f'MoveIt2 initialization failed: {e}')
            import traceback
            traceback.print_exc()
            self.get_logger().warn('Continuing without MoveIt2 - using direct control only')

        # Motion state tracking
        self.current_tool = None
        self.is_moving = False
        
        # Tool positions (relative to robot base)
        self.tool_positions = {
            'gripper': {'x': 0.0, 'y': -0.2, 'z': 0.15},
            'knife': {'x': 0.0, 'y': -0.25, 'z': 0.15},  
            'spoon': {'x': 0.0, 'y': -0.3, 'z': 0.15}
        }

        # Workspace safety limits (meters, relative to base_link)
        self.workspace_limits = {
            'x_min': 0.05, 'x_max': 0.5,
            'y_min': -0.4, 'y_max': 0.4,
            'z_min': 0.02, 'z_max': 0.5
        }

        self.get_logger().info('Motion control node initialized and ready')

    def connect_esp32(self):
        """Connect to ESP32 with error handling"""
        ports_to_try = ['/dev/ttyUSB0', '/dev/ttyACM0', '/dev/serial0']
        
        for port in ports_to_try:
            try:
                self.esp32_serial = serial.Serial(port, 115200, timeout=1)
                time.sleep(2)
                self.get_logger().info(f'✓ ESP32 connected on {port}')
                return
            except serial.SerialException:
                continue
        
        self.get_logger().error('Failed to connect to ESP32 on any port')
        self.esp32_serial = None

    def setup_moveit2(self):
        """Initialize MoveIt2 components for ROS2 Rolling"""
        self.get_logger().info('Initializing MoveIt2 components...')
        
        # Initialize MoveItPy with node_name parameter
        self.moveit = MoveItPy(node_name="motion_control_moveit")
        
        # Get robot model
        self.robot_model = self.moveit.get_robot_model()
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()
        
        # Set up planning groups (must match SRDF)
        self.arm_group_name = "arm"
        self.gripper_group_name = "gripper"
        
        self.get_logger().info(f'Robot model: {self.robot_model.getName()}')
        
        # Get available joint model groups
        available_groups = self.robot_model.getJointModelGroupNames()
        self.get_logger().info(f'Available joint groups: {available_groups}')
        
        # Validate arm group exists
        if self.arm_group_name not in available_groups:
            raise ValueError(
                f"Joint group '{self.arm_group_name}' not found. "
                f"Available: {available_groups}. "
                f"Check your SRDF file!"
            )
        
        # Get joint model group for arm
        self.arm_group = self.robot_model.getJointModelGroup(self.arm_group_name)
        
        # Get actual joint names from URDF/SRDF
        self.joint_names = self.arm_group.getVariableNames()
        self.get_logger().info(f'Arm joints: {self.joint_names}')
        
        # Check for gripper group
        if self.gripper_group_name in available_groups:
            self.gripper_group = self.robot_model.getJointModelGroup(self.gripper_group_name)
            self.get_logger().info('Gripper group found')
        else:
            self.gripper_group = None
            self.get_logger().warn('Gripper group not found - using manual control')
        
        # Get planning component
        self.planning_component = self.moveit.get_planning_component(self.arm_group_name)
        self.get_logger().info('Planning component created')
        
        # Set planning parameters
        self.planning_component.set_planning_time(10.0)
        
        # Set velocity and acceleration scaling (0.0 to 1.0)
        self.velocity_scaling = 0.5
        self.acceleration_scaling = 0.5
        
        self.get_logger().info('MoveIt2 initialization completed')
        
        # Move to home position
        self.move_to_home_position()

    def move_to_position_callback(self, request, response):
        """Service callback for moving to a specific pose"""
        try:
            pose = request.target_pose
            action_type = request.action_type
            
            self.get_logger().info(
                f'Move request: ({pose.position.x:.3f}, {pose.position.y:.3f}, '
                f'{pose.position.z:.3f}) for action: {action_type}'
            )
            
            # Safety check
            if not self.is_pose_safe(pose):
                self.get_logger().error(
                    f'Pose outside safe workspace: '
                    f'x:[{self.workspace_limits["x_min"]}, {self.workspace_limits["x_max"]}], '
                    f'y:[{self.workspace_limits["y_min"]}, {self.workspace_limits["y_max"]}], '
                    f'z:[{self.workspace_limits["z_min"]}, {self.workspace_limits["z_max"]}]'
                )
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
            import traceback
            traceback.print_exc()
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
            response.message = "Tool operation completed" if success else "Tool operation failed"
            
            return response
            
        except Exception as e:
            self.get_logger().error(f'Tool operation failed: {e}')
            response.success = False
            response.current_tool = self.current_tool or "none"
            response.message = str(e)
            return response

    def execute_motion_to_pose(self, pose, action_type="move"):
        """Execute motion using MoveIt2"""
        if not self.moveit_available:
            self.get_logger().error('MoveIt2 not available, cannot execute motion')
            return False
            
        try:
            self.is_moving = True
            self.publish_status(
                f"Planning motion to ({pose.position.x:.3f}, "
                f"{pose.position.y:.3f}, {pose.position.z:.3f})"
            )
            
            # Set start state to current state
            self.planning_component.set_start_state_to_current_state()
            
            # Create PoseStamped for goal
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = pose
            
            # Set goal pose - NOTE: Verify end effector link name in your URDF!
            self.planning_component.set_goal_state(
                pose_stamped_msg=pose_stamped,
                pose_link="ee"  # Change this to match your URDF end effector link
            )
            
            # Plan motion
            plan_result = self.planning_component.plan()
            
            # Check if planning was successful
            if plan_result and plan_result.error_code.val == 1:  # SUCCESS = 1
                self.publish_status('Motion planning successful, executing...')
                
                # Get the trajectory
                robot_trajectory = plan_result.trajectory
                
                if robot_trajectory and len(robot_trajectory.joint_trajectory.points) > 0:
                    # Get joint values from trajectory (final point)
                    final_point = robot_trajectory.joint_trajectory.points[-1]
                    joint_values = list(final_point.positions)
                    
                    self.get_logger().info(f'Planned joint values: {joint_values}')
                    
                    # Send to ESP32
                    esp32_success = self.send_joints_to_esp32(joint_values, action_type)
                    
                    if esp32_success:
                        self.publish_joint_states(joint_values)
                        self.publish_status(f'Motion completed: {action_type}')
                        self.is_moving = False
                        return True
                    else:
                        self.get_logger().warn('ESP32 communication failed but planning succeeded')
                        self.publish_joint_states(joint_values)
                        self.is_moving = False
                        return True
                else:
                    self.get_logger().error('Empty trajectory received')
                    self.is_moving = False
                    return False
            else:
                error_code = plan_result.error_code.val if plan_result else -1
                self.get_logger().error(f'Motion planning failed with error code: {error_code}')
                self.is_moving = False
                return False
                
        except Exception as e:
            self.get_logger().error(f'Motion execution error: {e}')
            import traceback
            traceback.print_exc()
            self.is_moving = False
            return False

    def send_joints_to_esp32(self, joint_values, action_type="move"):
        """Send joint angles to ESP32 with retry logic"""
        if not self.esp32_serial:
            self.get_logger().warn('ESP32 not connected, skipping hardware command')
            return True  # Don't block simulation
        
        for attempt in range(self.esp32_max_retries):
            try:
                # Pad to match actual number of joints
                num_joints = len(self.joint_names)
                if len(joint_values) < num_joints:
                    joint_values.extend([0.0] * (num_joints - len(joint_values)))
                
                # Create command dictionary with proper joint mapping
                joints_dict = {}
                for i, (name, value) in enumerate(zip(self.joint_names, joint_values)):
                    # Map to ESP32 expected names (J1, J2, etc)
                    joints_dict[f'J{i+1}'] = round(np.degrees(value), 2)
                
                command = {
                    'action': action_type,
                    'joints': joints_dict,
                    'speed': 30,
                    'timestamp': time.time()
                }
                
                # Send command
                json_command = json.dumps(command) + '\n'
                self.esp32_serial.write(json_command.encode())
                self.esp32_serial.flush()
                
                # Publish for debugging
                debug_msg = String()
                debug_msg.data = json_command
                self.esp32_command_pub.publish(debug_msg)
                
                # Wait for acknowledgment with timeout
                self.esp32_serial.timeout = 2.0
                response = self.esp32_serial.readline().decode().strip()
                
                if response == "OK":
                    self.get_logger().info(f'✓ ESP32 acknowledged: {action_type}')
                    return True
                else:
                    self.get_logger().warn(
                        f'ESP32 unexpected response (attempt {attempt+1}): {response}'
                    )
                    if attempt < self.esp32_max_retries - 1:
                        time.sleep(0.5)
                        continue
                    
            except serial.SerialException as e:
                self.get_logger().error(f'ESP32 serial error (attempt {attempt+1}): {e}')
                if attempt < self.esp32_max_retries - 1:
                    time.sleep(0.5)
                    continue
            except Exception as e:
                self.get_logger().error(f'ESP32 communication error (attempt {attempt+1}): {e}')
                if attempt < self.esp32_max_retries - 1:
                    time.sleep(0.5)
                    continue
        
        # All retries failed
        self.get_logger().error(
            f'ESP32 communication failed after {self.esp32_max_retries} attempts'
        )
        return False

    def execute_gripper_action(self, action):
        """Execute gripper open/close"""
        if not self.esp32_serial:
            self.get_logger().warn('ESP32 not connected, simulating gripper action')
            return True
        
        try:
            command = {
                'action': 'gripper',
                'state': action,
                'timestamp': time.time()
            }
            
            json_command = json.dumps(command) + '\n'
            self.esp32_serial.write(json_command.encode())
            self.esp32_serial.flush()
            
            # Wait for acknowledgment
            self.esp32_serial.timeout = 2.0
            response = self.esp32_serial.readline().decode().strip()
            
            if response == "OK":
                self.get_logger().info(f'✓ Gripper {action} completed')
                return True
            else:
                self.get_logger().warn(f'Gripper {action} failed: {response}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Gripper command failed: {e}')
            return False

    def pickup_tool(self, tool_name):
        """Navigate to tool location and attach it"""
        if tool_name not in self.tool_positions:
            self.get_logger().error(f'Unknown tool: {tool_name}')
            return False
        
        self.publish_status(f'Picking up {tool_name}')
        
        tool_pos = self.tool_positions[tool_name]
        
        # Create approach pose (10cm above tool)
        approach_pose = Pose()
        approach_pose.position.x = tool_pos['x']
        approach_pose.position.y = tool_pos['y']
        approach_pose.position.z = tool_pos['z'] + 0.1
        approach_pose.orientation.w = 1.0
        
        if not self.execute_motion_to_pose(approach_pose, "approach_tool"):
            return False
        
        if not self.execute_gripper_action('open'):
            return False
        
        # Move down to tool
        tool_pose = Pose()
        tool_pose.position.x = tool_pos['x']
        tool_pose.position.y = tool_pos['y']
        tool_pose.position.z = tool_pos['z']
        tool_pose.orientation.w = 1.0
        
        if not self.execute_motion_to_pose(tool_pose, "grasp_tool"):
            return False
        
        if not self.execute_gripper_action('close'):
            return False
        
        if not self.execute_motion_to_pose(approach_pose, "lift_tool"):
            return False
        
        self.current_tool = tool_name
        self.publish_status(f'{tool_name} attached')
        return True

    def return_current_tool(self):
        """Return currently held tool to its storage location"""
        if not self.current_tool:
            return True
        
        tool_name = self.current_tool
        self.publish_status(f'Returning {tool_name}')
        
        tool_pos = self.tool_positions[tool_name]
        
        approach_pose = Pose()
        approach_pose.position.x = tool_pos['x']
        approach_pose.position.y = tool_pos['y']
        approach_pose.position.z = tool_pos['z'] + 0.1
        approach_pose.orientation.w = 1.0
        
        if not self.execute_motion_to_pose(approach_pose, "approach_storage"):
            return False
        
        storage_pose = Pose()
        storage_pose.position.x = tool_pos['x']
        storage_pose.position.y = tool_pos['y']
        storage_pose.position.z = tool_pos['z']
        storage_pose.orientation.w = 1.0
        
        if not self.execute_motion_to_pose(storage_pose, "place_tool"):
            return False
        
        if not self.execute_gripper_action('open'):
            return False
        
        if not self.execute_motion_to_pose(approach_pose, "retract_from_tool"):
            return False
        
        self.current_tool = None
        self.publish_status(f'{tool_name} returned')
        return True

    def move_to_home_position(self):
        """Move arm to predefined home configuration"""
        self.get_logger().info('Moving to home position')
        
        if not self.moveit_available:
            self.get_logger().warn('MoveIt not available, cannot move to home')
            return False
        
        try:
            # Set start state
            self.planning_component.set_start_state_to_current_state()
            
            # Try to use named state from SRDF
            try:
                # Get robot state
                robot_state = RobotState(self.robot_model)
                
                # Set to named state "home" if it exists in SRDF
                if robot_state.setToDefaultValues(self.arm_group, "home"):
                    self.get_logger().info('Using named state "home" from SRDF')
                    self.planning_component.set_goal_state(robot_state=robot_state)
                else:
                    # Fallback to default joint positions
                    self.get_logger().warn('Named state "home" not found, using default positions')
                    home_joints = [0.0] * len(self.joint_names)
                    self.planning_component.set_goal_state(
                        configuration_name="home",
                        joint_positions=home_joints
                    )
            except Exception as e:
                self.get_logger().warn(f'Could not set named state: {e}')
                # Use zero positions as fallback
                home_joints = [0.0] * len(self.joint_names)
                robot_state = RobotState(self.robot_model)
                robot_state.setVariablePositions(self.joint_names, home_joints)
                self.planning_component.set_goal_state(robot_state=robot_state)
            
            # Plan
            plan_result = self.planning_component.plan()
            
            if plan_result and plan_result.error_code.val == 1:
                robot_trajectory = plan_result.trajectory
                if robot_trajectory and len(robot_trajectory.joint_trajectory.points) > 0:
                    final_point = robot_trajectory.joint_trajectory.points[-1]
                    joint_values = list(final_point.positions)
                    
                    self.send_joints_to_esp32(joint_values, "home")
                    self.publish_joint_states(joint_values)
                    
                    self.get_logger().info('✓ Home position reached')
                    return True
            
            self.get_logger().error('Failed to plan to home position')
            return False
            
        except Exception as e:
            self.get_logger().error(f'Move to home failed: {e}')
            import traceback
            traceback.print_exc()
            return False

    def is_pose_safe(self, pose):
        """Check if pose is within safe workspace limits"""
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        
        if not (self.workspace_limits['x_min'] <= x <= self.workspace_limits['x_max']):
            return False
        if not (self.workspace_limits['y_min'] <= y <= self.workspace_limits['y_max']):
            return False
        if not (self.workspace_limits['z_min'] <= z <= self.workspace_limits['z_max']):
            return False
        
        return True

    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = f"[Motion] {message}"
        self.motion_status_pub.publish(msg)
        self.get_logger().info(message)

    def publish_joint_states(self, joint_values):
        """Publish current joint state"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = joint_values
        msg.velocity = [0.0] * len(joint_values)
        msg.effort = [0.0] * len(joint_values)
        
        self.joint_state_pub.publish(msg)

    def __del__(self):
        """Cleanup on node shutdown"""
        if self.esp32_serial and self.esp32_serial.is_open:
            self.esp32_serial.close()
            self.get_logger().info('ESP32 serial connection closed')


def main(args=None):
    rclpy.init(args=args)
    node = motionControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()