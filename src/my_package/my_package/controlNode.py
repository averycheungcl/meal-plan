#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients, GenerateRecipe, MoveToPosition, ExecuteGrip, SetTool
from my_package.msg import Ingredients, Steps
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from dataclasses import dataclass

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
        self.grid_size = 0.1  # meters per grid unit
        self.current_tool = None

        # Wait for all services
        self.wait_for_services()

        # Execute main workflow
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
            return

        self.get_logger().info(f'Detected {len(ingredients)} ingredients')

        # Step 2: Generate recipe
        self.publish_status("Step 2: Generating recipe")
        recipe = self.generate_recipe(ingredients)
        if not recipe:
            self.get_logger().error('Failed to generate recipe')
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
            rclpy.spin_until_future_complete(self, future)
            
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
            rclpy.spin_until_future_complete(self, future)
            
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
        
        for step_msg in recipe.steps:
            for step_num, step_desc in zip(step_msg.step_numbers, step_msg.step_descriptions):
                self.get_logger().info(f'Processing Step {step_num}: {step_desc}')
                self.publish_status(f'Executing step {step_num}: {step_desc}')
                
                # Parse recipe step into arm movements
                arm_steps = self.parse_recipe_step(step_desc, detected_ingredients)
                
                # Execute each arm step
                for arm_step in arm_steps:
                    success = self.execute_arm_step(arm_step)
                    if not success:
                        self.get_logger().warn(f'Failed to execute step: {arm_step.action} on {arm_step.target}')

    def parse_recipe_step(self, step_desc, detected_ingredients):
        """Parse recipe text into structured arm movements"""
        step_desc_lower = step_desc.lower()
        arm_steps = []

        # Find ingredients mentioned in the step
        for ing_msg in detected_ingredients:
            if ing_msg.name.lower() in step_desc_lower:
                pose = self.get_ingredient_pose(ing_msg)

                # Determine actions based on keywords
                if 'pick' in step_desc_lower or 'grab' in step_desc_lower or 'take' in step_desc_lower:
                    arm_steps.append(ArmStep('pick', target=ing_msg.name, tool='gripper', pose=pose))
                
                if 'place' in step_desc_lower or 'put' in step_desc_lower or 'add' in step_desc_lower:
                    arm_steps.append(ArmStep('place', target=ing_msg.name, tool='gripper', pose=pose))
                
                if 'cut' in step_desc_lower or 'chop' in step_desc_lower or 'slice' in step_desc_lower:
                    arm_steps.append(ArmStep('cut', target=ing_msg.name, tool='knife', pose=pose))
                
                if 'stir' in step_desc_lower or 'mix' in step_desc_lower:
                    arm_steps.append(ArmStep('stir', target=ing_msg.name, tool='spoon', pose=pose))

        return arm_steps

    def execute_arm_step(self, arm_step):
        """Execute a single arm movement step"""
        try:
            self.get_logger().info(f'Executing: {arm_step.action} {arm_step.target} with {arm_step.tool}')

            # Switch tool if needed
            if arm_step.tool and arm_step.tool != self.current_tool:
                if not self.set_tool(arm_step.tool):
                    return False

            # Move to position
            if arm_step.pose:
                if not self.move_to_pose(arm_step.pose, arm_step.action):
                    return False

            # Execute gripper action if needed
            if arm_step.action in ['pick', 'place']:
                gripper_action = 'close' if arm_step.action == 'pick' else 'open'
                if not self.execute_grip(gripper_action):
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
            rclpy.spin_until_future_complete(self, future)

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
            rclpy.spin_until_future_complete(self, future)

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
            rclpy.spin_until_future_complete(self, future)

            if future.result() and future.result().success:
                self.current_tool = future.result().current_tool
                return True
            else:
                return False

        except Exception as e:
            self.get_logger().error(f'Tool service call failed: {e}')
            return False

    def get_ingredient_pose(self, ing_msg):
        """Convert ingredient grid coordinates to pose"""
        pose = Pose()
        pose.position.x = ing_msg.x_grid * self.grid_size
        pose.position.y = ing_msg.y_grid * self.grid_size
        pose.position.z = ing_msg.z_grid * self.grid_size
        pose.orientation.w = 1.0
        return pose

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
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
