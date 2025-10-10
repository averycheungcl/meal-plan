#!/usr/bin/env python3
#transiution to services for motion control services as goal progress and results is more suitable for 
import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients, GenerateRecipe, MoveToPosition, ExecuteGrip, SetTool
from my_package.msg import Ingredients, Steps
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from dataclasses import dataclass
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import math
import time
import threading

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
        
        self.workflow_started = False
        self.executor = None  # Will be set by main()

        # Only wait for detection and recipe services initially
        self.get_logger().info('Control node initialized, waiting for services...')
        self.wait_for_initial_services()

        # Start workflow after a delay to ensure TF is ready
        self.workflow_timer = self.create_timer(3.0, self.start_workflow_once)

    def set_executor(self, executor):
        """Set the executor reference for proper async handling"""
        self.executor = executor

    def start_workflow_once(self):
        """Start workflow once, then disable timer"""
        if not self.workflow_started:
            self.workflow_started = True
            self.workflow_timer.cancel()  # Cancel the timer
            
            # Run workflow in a separate thread to avoid blocking the executor
            workflow_thread = threading.Thread(target=self.execute_workflow)
            workflow_thread.daemon = True
            workflow_thread.start()

    def wait_for_initial_services(self):
        """Wait only for detection and planning services at startup"""
        services = [
            (self.detect_client, 'detect_ingredients'),
            (self.recipe_client, 'generate_recipe'),
        ]

        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {service_name} service...')
        
        self.get_logger().info('✓ Detection and planning services are available!')

    def wait_for_motion_services(self):
        """Wait for motion control services when needed"""
        self.get_logger().info('Waiting for motion control services...')
        
        services = [
            (self.motion_client, 'move_to_position'),
            (self.grip_client, 'execute_grip'),
            (self.tool_client, 'set_tool')
        ]

        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {service_name} service...')
        
        self.get_logger().info('✓ Motion control services are available!')

    def execute_workflow(self):
        """Main cooking workflow coordinator - runs in separate thread"""
        try:
            self.publish_status("Starting autocook workflow")

            # Step 1: Detect ingredients
            self.publish_status("Step 1: Detecting ingredients")
            ingredients = self.detect_ingredients()
            
            if not ingredients:
                self.get_logger().error('Failed to detect ingredients')
                self.publish_status("Workflow failed: No ingredients detected")
                return

            self.get_logger().info(f'✓ Detected {len(ingredients)} ingredients: {[i.name for i in ingredients]}')

            # Step 2: Generate recipe
            self.publish_status("Step 2: Generating recipe")
            recipe = self.generate_recipe(ingredients)
            if not recipe:
                self.get_logger().error('Failed to generate recipe')
                self.publish_status("Workflow failed: Recipe generation failed")
                return

            self.get_logger().info(f'✓ Generated recipe: {recipe.recipe_name}')

            # Step 3: NOW wait for motion services before executing
            self.publish_status("Step 3: Waiting for motion control system...")
            self.wait_for_motion_services()

            # Step 4: Execute cooking steps
            self.publish_status("Step 4: Executing cooking steps")
            self.process_recipe(recipe, ingredients)

            # Step 5: Return to home and cleanup
            self.publish_status("Step 5: Cleaning up")
            self.cleanup()

            self.publish_status("✓ Autocook workflow completed successfully!")
            
        except Exception as e:
            self.get_logger().error(f'Workflow failed with exception: {e}')
            import traceback
            traceback.print_exc()
            self.publish_status(f"Workflow failed: {str(e)}")

    def call_service_async(self, client, request, timeout_sec=10.0):
        """Helper method to properly call services asynchronously with MultiThreadedExecutor"""
        future = client.call_async(request)
        
        # Wait for the future to complete
        start_time = time.time()
        while not future.done():
            if (time.time() - start_time) > timeout_sec:
                future.cancel()
                return None
            time.sleep(0.01)  # Small sleep to avoid busy waiting
        
        return future.result()

    def detect_ingredients(self):
        """Call ingredient detection service"""
        try:
            if not self.detect_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('detect_ingredients service not available')
                return None
            
            request = DetectIngredients.Request()
            self.get_logger().info('Calling detect_ingredients service...')
            
            # Use helper method for proper async handling
            result = self.call_service_async(self.detect_client, request, timeout_sec=10.0)
            
            if result and result.ingredients:
                self.get_logger().info(f'✓ Received {len(result.ingredients)} ingredients from service')
                
                # Log each ingredient
                for i, ing in enumerate(result.ingredients):
                    self.get_logger().debug(f'  [{i}] {ing.name}')
                
                return result.ingredients
            else:
                self.get_logger().error('Service returned None or empty result')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Ingredient detection failed: {e}')
            import traceback
            traceback.print_exc()
            return None

    def generate_recipe(self, ingredients):
        """Call recipe generation service"""
        try:
            if not self.recipe_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('generate_recipe service not available')
                return None
            
            request = GenerateRecipe.Request()
            request.ingredients = ingredients
            
            self.get_logger().info(f'Calling generate_recipe with {len(ingredients)} ingredients...')
            
            # Use helper method with longer timeout for LLM
            result = self.call_service_async(self.recipe_client, request, timeout_sec=60.0)
            
            if result:
                self.get_logger().info(f'✓ Recipe received: {result.recipe_name}')
                return result
            else:
                self.get_logger().error('Recipe service returned None')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Recipe generation failed: {e}')
            import traceback
            traceback.print_exc()
            return None

    def process_recipe(self, recipe, detected_ingredients):
        """Process recipe steps and coordinate arm movements"""
        self.get_logger().info(f'Processing recipe: {recipe.recipe_name}')
        
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
                        self.get_logger().error(f'Failed to execute step: {arm_step.action} on {arm_step.target}')

    def parse_recipe_step(self, step_desc, detected_ingredients, ingredient_usage):
        """Parse recipe text into structured arm movements"""
        step_desc_lower = step_desc.lower()
        arm_steps = []

        # Detect action types first
        has_pick = any(kw in step_desc_lower for kw in ['pick', 'grab', 'take', 'get'])
        has_place = any(kw in step_desc_lower for kw in ['place', 'put', 'add', 'drop'])
        has_cut = any(kw in step_desc_lower for kw in ['cut', 'chop', 'slice', 'dice'])
        has_stir = any(kw in step_desc_lower for kw in ['stir', 'mix', 'blend'])

        # Find ingredients mentioned in the step
        for ing_msg in detected_ingredients:
            if ing_msg.name.lower() in step_desc_lower:
                pose = self.get_ingredient_pose(ing_msg)
                
                if pose is None:
                    self.get_logger().error(f'Could not get pose for {ing_msg.name}, skipping')
                    continue

                if has_pick:
                    arm_steps.append(ArmStep('pick', target=ing_msg.name, tool='gripper', pose=pose))
                    ingredient_usage[ing_msg.name] += 1
                
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

            self.get_logger().info(f'✓ Successfully executed: {arm_step.action} on {arm_step.target}')
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to execute arm step: {e}')
            return False

    def move_to_pose(self, pose, action_type="move"):
        """Move arm to specified pose using motion service"""
        try:
            if not self.motion_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Motion service not ready, skipping')
                return True
            
            request = MoveToPosition.Request()
            request.target_pose = pose
            request.action_type = action_type

            result = self.call_service_async(self.motion_client, request, timeout_sec=30.0)
            return result.success if result else False

        except Exception as e:
            self.get_logger().error(f'Motion service call failed: {e}')
            return False

    def execute_grip(self, action):
        """Execute gripper action using grip service"""
        try:
            if not self.grip_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Grip service not ready, skipping')
                return True
            
            request = ExecuteGrip.Request()
            request.action = action

            result = self.call_service_async(self.grip_client, request, timeout_sec=10.0)
            return result.success if result else False

        except Exception as e:
            self.get_logger().error(f'Grip service call failed: {e}')
            return False

    def set_tool(self, tool_name):
        """Set/change tool using tool service"""
        try:
            if not self.tool_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('Tool service not ready, skipping')
                return True
            
            request = SetTool.Request()
            request.tool_name = tool_name

            result = self.call_service_async(self.tool_client, request, timeout_sec=15.0)
            
            if result and result.success:
                self.current_tool = result.current_tool
                return True
            return False

        except Exception as e:
            self.get_logger().error(f'Tool service call failed: {e}')
            return False

    def get_ingredient_pose(self, ing_msg):
        """Convert ingredient coordinates from camera frame to base_link frame using TF2"""
        try:
            # Handle hardcoded ingredients (all zeros)
            if ing_msg.x_center == 0 and ing_msg.y_center == 0 and ing_msg.distance == 0:
                default_pose = Pose()
                default_pose.position.x = 0.3
                default_pose.position.y = 0.0
                default_pose.position.z = 0.2
                default_pose.orientation.w = 1.0
                
                self.get_logger().debug(f'Using default pose for hardcoded ingredient: {ing_msg.name}')
                return default_pose
            
            # Create pose in camera_link frame
            camera_pose = PoseStamped()
            camera_pose.header.frame_id = 'camera_link'
            camera_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Extract pixel coordinates from message
            pixel_x = ing_msg.x_center
            pixel_y = ing_msg.y_center
            distance = ing_msg.distance
            
            # Camera principal point (image center)
            cx = self.image_width / 2.0
            cy = self.image_height / 2.0
            
            # Convert pixel coordinates to camera frame 3D coordinates
            x_cam = (pixel_x - cx) * distance / self.focal_length
            y_cam = (pixel_y - cy) * distance / self.focal_length
            z_cam = distance
            
            camera_pose.pose.position.x = z_cam
            camera_pose.pose.position.y = -x_cam
            camera_pose.pose.position.z = -y_cam
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
                
                self.get_logger().debug(
                    f"Transformed {ing_msg.name}: "
                    f"base({base_pose.pose.position.x:.3f}, "
                    f"{base_pose.pose.position.y:.3f}, "
                    f"{base_pose.pose.position.z:.3f})"
                )
                
                return base_pose.pose
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'TF2 transform failed: {e}')
                
                # Return a safe default pose
                default_pose = Pose()
                default_pose.position.x = 0.3
                default_pose.position.y = 0.0
                default_pose.position.z = 0.2
                default_pose.orientation.w = 1.0
                
                self.get_logger().debug(f'Using default pose for {ing_msg.name}')
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
    
    # Create node
    node = controlNode()
    
    # Use MultiThreadedExecutor for proper async handling
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Set executor reference in node
    node.set_executor(executor)
    
    try:
        executor.spin()
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
