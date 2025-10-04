#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients, GenerateRecipe, MoveToPosition, ExecuteGrip, SetTool
from my_package.msg import Ingredients, Steps
from geometry_msgs.msg import Pose, PoseStamped  #   FIXED: Added PoseStamped
from std_msgs.msg import String
from dataclasses import dataclass
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import math

@dataclass
class ArmStep:
    action: str          # 'pick', 'place', 'cut', 'stir' 
    target: str = None   # ingredient name
    tool: str = None     # 'gripper', 'knife', 'spoon'
    pose: Pose = None    # target pose
    

class controlNode(Node):
    def __init__(self):
        super().__init__('control_coordinator_node')

        # Service clients
        self.detect_client = self.create_client(DetectIngredients, 'detect_ingredients')
        self.recipe_client = self.create_client(GenerateRecipe, 'generate_recipe')
        self.motion_client = self.create_client(MoveToPosition, 'move_to_position')
        self.grip_client = self.create_client(ExecuteGrip, 'execute_grip')
        self.tool_client = self.create_client(SetTool, 'set_tool')

        # Publishers
        self.status_publisher = self.create_publisher(String, 'coordinator_status', 10)

        # Configuration
        self.current_tool = None
        
        # Camera parameters (should match webcamNode)
        self.focal_length = 600.0  # pixels
        self.image_width = 1280.0
        self.image_height = 720.0
        
        # Add TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        #   FIXED: Don't run workflow in __init__, use timer
        self.workflow_started = False

        # Wait for all services
        self.wait_for_services()

        # Start workflow after a delay to ensure TF is ready
        self.create_timer(2.0, self.start_workflow_once)

    def start_workflow_once(self):
        """Start workflow once, then disable timer"""
        if not self.workflow_started:
            self.workflow_started = True
            self.execute_workflow()

    def wait_for_services(self):
        """Wait for all required services to be available"""
        services = [
            (self.detect_client, 'detect_ingredients'),
            (self.recipe_client, 'generate_recipe'),
            (self.motion_client, 'move_to_position'),
            (self.grip_client, 'execute_grip'),
            (self.tool_client, 'set_tool')
        ]

        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {service_name} service...')
        
        self.get_logger().info('All services are available!')

    def execute_workflow(self):
        """Main cooking workflow coordinator"""
        self.publish_status("Starting autocook workflow")

        # Step 1: Detect ingredients
        self.publish_status("Step 1: Detecting ingredients")
        ingredients = self.detect_ingredients()
        if not ingredients:
            self.get_logger().error('Failed to detect ingredients')
            self.publish_status("Workflow failed: No ingredients detected")
            return

        self.get_logger().info(f'Detected {len(ingredients)} ingredients')

        # Step 2: Generate recipe
        self.publish_status("Step 2: Generating recipe")
        recipe = self.generate_recipe(ingredients)
        if not recipe:
            self.get_logger().error('Failed to generate recipe')
            self.publish_status("Workflow failed: Recipe generation failed")
            return

        self.get_logger().info(f'Generated recipe: {recipe.recipe_name}')

        # Step 3: Execute cooking steps
        self.publish_status("Step 3: Executing cooking steps")
        self.process_recipe(recipe, ingredients)

        # Step 4: Return to home and cleanup
        self.publish_status("Step 4: Cleaning up")
        self.cleanup()

        self.publish_status("Autocook workflow completed successfully!")

    def detect_ingredients(self):
        """Call ingredient detection service"""
        try:
            request = DetectIngredients.Request()
            future = self.detect_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result():
                return future.result().ingredients
            else:
                return None
                
        except Exception as e:
            self.get_logger().error(f'Ingredient detection failed: {e}')
            return None

    def generate_recipe(self, ingredients):
        """Call recipe generation service"""
        try:
            request = GenerateRecipe.Request()
            request.ingredients = ingredients
            future = self.recipe_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.result():
                return future.result()
            else:
                return None
                
        except Exception as e:
            self.get_logger().error(f'Recipe generation failed: {e}')
            return None

    def process_recipe(self, recipe, detected_ingredients):
        """Process recipe steps and coordinate arm movements"""
        self.get_logger().info(f'Processing recipe: {recipe.recipe_name}')
        
        #   IMPROVED: Track which ingredients have been used
        ingredient_usage = {ing.name: 0 for ing in detected_ingredients}
        
        for step_msg in recipe.steps:
            for step_num, step_desc in zip(step_msg.step_numbers, step_msg.step_descriptions):
                self.get_logger().info(f'Processing Step {step_num}: {step_desc}')
                self.publish_status(f'Executing step {step_num}: {step_desc}')
                
                # Parse recipe step into arm movements
                arm_steps = self.parse_recipe_step(step_desc, detected_ingredients, ingredient_usage)
                
                # Execute each arm step
                for arm_step in arm_steps:
                    success = self.execute_arm_step(arm_step)
                    if not success:
                        self.get_logger().warn(f'Failed to execute step: {arm_step.action} on {arm_step.target}')
                        # Continue anyway for demo purposes

    def parse_recipe_step(self, step_desc, detected_ingredients, ingredient_usage):
        """Parse recipe text into structured arm movements"""
        step_desc_lower = step_desc.lower()
        arm_steps = []

        #   IMPROVED: Better parsing with sequence awareness
        # Detect action types first
        has_pick = any(kw in step_desc_lower for kw in ['pick', 'grab', 'take', 'get'])
        has_place = any(kw in step_desc_lower for kw in ['place', 'put', 'add', 'drop'])
        has_cut = any(kw in step_desc_lower for kw in ['cut', 'chop', 'slice', 'dice'])
        has_stir = any(kw in step_desc_lower for kw in ['stir', 'mix', 'blend'])

        # Find ingredients mentioned in the step
        for ing_msg in detected_ingredients:
            if ing_msg.name.lower() in step_desc_lower:
                # Get the pose for this specific ingredient instance
                pose = self.get_ingredient_pose(ing_msg)
                
                if pose is None:
                    self.get_logger().warn(f'Could not get pose for {ing_msg.name}, skipping')
                    continue

                #   IMPROVED: Create proper action sequences
                if has_pick:
                    arm_steps.append(ArmStep('pick', target=ing_msg.name, tool='gripper', pose=pose))
                    ingredient_usage[ing_msg.name] += 1
                
                # For place, we'd need to know WHERE to place (cutting board, pan, etc)
                # For now, use a default placement zone
                if has_place:
                    placement_pose = self.get_placement_pose()
                    arm_steps.append(ArmStep('place', target=ing_msg.name, tool='gripper', pose=placement_pose))
                
                if has_cut:
                    arm_steps.append(ArmStep('cut', target=ing_msg.name, tool='knife', pose=pose))
                
                if has_stir:
                    arm_steps.append(ArmStep('stir', target=ing_msg.name, tool='spoon', pose=pose))

        return arm_steps

    def get_placement_pose(self):
        """Get a default placement pose (e.g., cutting board location)"""
        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.2
        pose.position.z = 0.15
        pose.orientation.w = 1.0
        return pose

    def execute_arm_step(self, arm_step):
        """Execute a single arm movement step"""
        try:
            self.get_logger().info(f'Executing: {arm_step.action} {arm_step.target} with {arm_step.tool}')

            # Switch tool if needed
            if arm_step.tool and arm_step.tool != self.current_tool:
                if not self.set_tool(arm_step.tool):
                    self.get_logger().error(f'Failed to switch to {arm_step.tool}')
                    return False

            # Move to position
            if arm_step.pose:
                if not self.move_to_pose(arm_step.pose, arm_step.action):
                    self.get_logger().error(f'Failed to move to pose for {arm_step.action}')
                    return False

            # Execute gripper action if needed
            if arm_step.action in ['pick', 'place']:
                gripper_action = 'close' if arm_step.action == 'pick' else 'open'
                if not self.execute_grip(gripper_action):
                    self.get_logger().error(f'Failed to execute gripper {gripper_action}')
                    return False

            self.get_logger().info(f'Successfully executed: {arm_step.action} on {arm_step.target}')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to execute arm step: {e}')
            return False

    def move_to_pose(self, pose, action_type="move"):
        """Move arm to specified pose using motion service"""
        try:
            request = MoveToPosition.Request()
            request.target_pose = pose
            request.action_type = action_type

            future = self.motion_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

            if future.result():
                return future.result().success
            else:
                return False

        except Exception as e:
            self.get_logger().error(f'Motion service call failed: {e}')
            return False

    def execute_grip(self, action):
        """Execute gripper action using grip service"""
        try:
            request = ExecuteGrip.Request()
            request.action = action

            future = self.grip_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.result():
                return future.result().success
            else:
                return False

        except Exception as e:
            self.get_logger().error(f'Grip service call failed: {e}')
            return False

    def set_tool(self, tool_name):
        """Set/change tool using tool service"""
        try:
            request = SetTool.Request()
            request.tool_name = tool_name

            future = self.tool_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

            if future.result() and future.result().success:
                self.current_tool = future.result().current_tool
                return True
            else:
                return False

        except Exception as e:
            self.get_logger().error(f'Tool service call failed: {e}')
            return False

    def get_ingredient_pose(self, ing_msg):
        """  FIXED: Convert ingredient coordinates from camera frame to base_link frame using TF2"""
        try:
            # Create pose in camera_link frame using PROPER pinhole camera model
            camera_pose = PoseStamped()
            camera_pose.header.frame_id = 'camera_link'
            #   FIXED: Use Time(0) for latest available transform
            camera_pose.header.stamp = rclpy.time.Time().to_msg()
            
            #   FIXED: Convert pixel coordinates to 3D using pinhole camera model
            # Extract pixel coordinates from message
            pixel_x = ing_msg.x_center
            pixel_y = ing_msg.y_center
            distance = ing_msg.distance
            
            # Camera principal point (image center)
            cx = self.image_width / 2.0
            cy = self.image_height / 2.0
            
            # Convert pixel coordinates to camera frame 3D coordinates
            # In camera frame: X=right, Y=down, Z=forward
            x_cam = (pixel_x - cx) * distance / self.focal_length
            y_cam = (pixel_y - cy) * distance / self.focal_length
            z_cam = distance
            
            camera_pose.pose.position.x = z_cam      # Forward (into scene)
            camera_pose.pose.position.y = -x_cam     # Right (in image) = left in camera frame
            camera_pose.pose.position.z = -y_cam     # Down (in image) = up in camera frame
            camera_pose.pose.orientation.w = 1.0
            
            self.get_logger().debug(
                f"Camera frame {ing_msg.name}: "
                f"pixel({pixel_x:.0f}, {pixel_y:.0f}) @ {distance:.2f}m -> "
                f"cam({camera_pose.pose.position.x:.3f}, "
                f"{camera_pose.pose.position.y:.3f}, "
                f"{camera_pose.pose.position.z:.3f})"
            )
            
            # Transform from camera_link to base_link using TF2
            try:
                base_pose = self.tf_buffer.transform(
                    camera_pose, 
                    'base_link',
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                
                self.get_logger().info(
                    f"Transformed {ing_msg.name}: "
                    f"base({base_pose.pose.position.x:.3f}, "
                    f"{base_pose.pose.position.y:.3f}, "
                    f"{base_pose.pose.position.z:.3f})"
                )
                
                return base_pose.pose
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f'TF2 transform failed: {e}')
                self.get_logger().error('Check that camera_link->base_link transform is published!')
                
                #   FIXED: Return a safe default pose instead of undefined variable
                default_pose = Pose()
                default_pose.position.x = 0.3
                default_pose.position.y = 0.0
                default_pose.position.z = 0.2
                default_pose.orientation.w = 1.0
                
                self.get_logger().warn(f'Using default pose for {ing_msg.name}')
                return default_pose
                
        except Exception as e:
            self.get_logger().error(f'Failed to get ingredient pose: {e}')
            return None

    def cleanup(self):
        """Return tools and move to home position"""
        try:
            # Return current tool
            if self.current_tool:
                self.set_tool("none")

            # Move to home position
            home_pose = Pose()
            home_pose.position.x = 0.2
            home_pose.position.y = 0.0
            home_pose.position.z = 0.3
            home_pose.orientation.w = 1.0
            
            self.move_to_pose(home_pose, "home")
            self.publish_status("Cleanup completed")

        except Exception as e:
            self.get_logger().error(f'Cleanup failed: {e}')

    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = f"[Coordinator] {message}"
        self.status_publisher.publish(msg)
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = controlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()