#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients, GenerateRecipe
from my_package.msg import Ingredients, Steps
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs
from dataclasses import dataclass
import math

@dataclass
class ArmStep:
    action: str          # 'pick', 'place', 'cut', 'stir' 
    target: str = None   # ingredient name
    tool: str = None     # 'gripper', 'knife', 'spoon'
    pose: PoseStamped = None    # target pose with frame


class controlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Clients
        self.detect_client = self.create_client(DetectIngredients, 'detect_ingredients')
        self.recipe_client = self.create_client(GenerateRecipe, 'generate_recipe')

        # Publisher for arm goal (now with frame info)
        self.publisher_ = self.create_publisher(PoseStamped, 'arm_goal', 10)

        # Frame definitions
        self.base_frame = 'base_link'
        self.camera_frame = 'camera_link'
        self.end_effector_frame = 'end_effector_link'
        
        # Grid scaling and positions
        self.grid_size = 0.1  # meters per grid unit
        self.home_position = (0.0, 0.0, 0.5)
        self.current_tool = None

        # Tool positions relative to base_link
        self.tool_positions = {
            'gripper': (-0.2, 0.0, 0.1),  # Behind robot
            'knife': (-0.15, 0.0, 0.1),
            'spoon': (-0.1, 0.0, 0.1)
        }

        # Workspace definitions
        self.ingredient_workspace = {'x_min': 0.1, 'x_max': 0.4, 'y_min': -0.2, 'y_max': 0.2}
        self.pan_position = (-0.05, 0.3, 0.05)  # Left of robot

        # Publish static transforms
        self.publish_static_transforms()

        # Wait for services
        while not self.detect_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for detect_ingredients service...')
        while not self.recipe_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for generate_recipe service...')

        # Execute workflow
        self.execute_workflow()

    def publish_static_transforms(self):
        """Publish static transforms for camera and workspace"""
        timer = self.create_timer(0.1, self.broadcast_transforms)

    def broadcast_transforms(self):
        """Broadcast coordinate frame transforms"""
        # Camera transform (front of robot, looking at ingredients)
        cam_transform = TransformStamped()
        cam_transform.header.stamp = self.get_clock().now().to_msg()
        cam_transform.header.frame_id = self.base_frame
        cam_transform.child_frame_id = self.camera_frame
        cam_transform.transform.translation.x = 0.3
        cam_transform.transform.translation.y = 0.0
        cam_transform.transform.translation.z = 0.5
        cam_transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(cam_transform)

    def execute_workflow(self):
        # Step 1: Detect ingredients
        detect_req = DetectIngredients.Request()
        future = self.detect_client.call_async(detect_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error('Failed to detect ingredients')
            return

        detected_ingredients = future.result().ingredients
        self.get_logger().info(f'Detected {len(detected_ingredients)} ingredients.')

        # Step 2: Generate recipe
        recipe_req = GenerateRecipe.Request()
        recipe_req.ingredients = detected_ingredients
        future = self.recipe_client.call_async(recipe_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error('Failed to generate recipe')
            return

        recipe = future.result()
        self.process_recipe(recipe, detected_ingredients)
        self.return_home()

    def process_recipe(self, recipe, detected_ingredients):
        for step_msg in recipe.steps:
            for step_num, step_desc in zip(step_msg.step_numbers, step_msg.step_descriptions):
                self.get_logger().info(f'Processing Step {step_num}: {step_desc}')
                arm_steps = self.parse_recipe_step(step_desc, detected_ingredients)
                self.execute_arm_steps(arm_steps)

    def parse_recipe_step(self, step_desc, detected_ingredients):
        step_desc_lower = step_desc.lower()
        arm_steps = []

        for ing_msg in detected_ingredients:
            if ing_msg.name.lower() in step_desc_lower:
                pose = self.get_ingredient_pose_stamped(ing_msg)

                if 'pick' in step_desc_lower:
                    arm_steps.append(ArmStep('pick', target=ing_msg.name, tool='gripper', pose=pose))
                elif 'place' in step_desc_lower and 'pan' in step_desc_lower:
                    pan_pose = self.get_pan_pose_stamped()
                    arm_steps.append(ArmStep('place', target='pan', tool='gripper', pose=pan_pose))
                elif 'cut' in step_desc_lower:
                    arm_steps.append(ArmStep('cut', target=ing_msg.name, tool='knife', pose=pose))
                elif 'stir' in step_desc_lower:
                    pan_pose = self.get_pan_pose_stamped()
                    arm_steps.append(ArmStep('stir', tool='spoon', pose=pan_pose))

        return arm_steps

    def execute_arm_steps(self, arm_steps):
        for step in arm_steps:
            if step.tool and step.tool != self.current_tool:
                self.switch_tool(step.tool)
            
            if step.pose:
                # Transform pose to base_link if needed
                try:
                    if step.pose.header.frame_id != self.base_frame:
                        transformed_pose = self.tf_buffer.transform(
                            step.pose, self.base_frame, timeout=rclpy.duration.Duration(seconds=1.0)
                        )
                    else:
                        transformed_pose = step.pose
                    
                    self.move_to_pose(transformed_pose)
                    self.get_logger().info(f"Executed {step.action} on {step.target} using {step.tool}")
                    
                except Exception as e:
                    self.get_logger().error(f"Transform failed: {e}")

    def get_ingredient_pose_stamped(self, ing_msg):
        """Convert ingredient detection to pose in camera frame, then transform to base"""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.camera_frame
        
        # Convert from camera pixel coordinates to camera frame coordinates
        # This assumes camera is looking down at ingredients
        pose_stamped.pose.position.x = ing_msg.x_grid * self.grid_size
        pose_stamped.pose.position.y = ing_msg.y_grid * self.grid_size
        pose_stamped.pose.position.z = -ing_msg.distance  # Camera looks down
        pose_stamped.pose.orientation.w = 1.0
        
        return pose_stamped

    def get_pan_pose_stamped(self):
        """Get pan position in base frame"""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.pose.position.x = self.pan_position[0]
        pose_stamped.pose.position.y = self.pan_position[1]
        pose_stamped.pose.position.z = self.pan_position[2]
        pose_stamped.pose.orientation.w = 1.0
        return pose_stamped

    def switch_tool(self, tool_name):
        """Switch to specified tool with proper coordinate frames"""
        if self.current_tool != tool_name and tool_name in self.tool_positions:
            # Return current tool first
            if self.current_tool:
                self.return_tool()
            
            # Pick up new tool
            x, y, z = self.tool_positions[tool_name]
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = self.base_frame
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = z
            pose_stamped.pose.orientation.w = 1.0
            
            self.move_to_pose(pose_stamped)
            self.current_tool = tool_name
            self.get_logger().info(f'Switched to tool: {tool_name}')

    def return_tool(self):
        """Return current tool to its storage position"""
        if self.current_tool and self.current_tool in self.tool_positions:
            x, y, z = self.tool_positions[self.current_tool]
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = self.base_frame
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = z
            pose_stamped.pose.orientation.w = 1.0
            
            self.move_to_pose(pose_stamped)
            self.get_logger().info(f'Returned tool: {self.current_tool}')
        self.current_tool = None

    def return_home(self):
        """Move arm to home position"""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.pose.position.x = self.home_position[0]
        pose_stamped.pose.position.y = self.home_position[1]
        pose_stamped.pose.position.z = self.home_position[2]
        pose_stamped.pose.orientation.w = 1.0
        
        self.move_to_pose(pose_stamped)
        self.get_logger().info('Moved to home position')

    def move_to_pose(self, pose_stamped):
        """Send pose command to arm controller"""
        self.publisher_.publish(pose_stamped)
        # Add small delay for realistic motion
        rclpy.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = controlNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
