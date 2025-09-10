#!/usr/bin/env python3

#NEED TO ADD PID 

import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients, GenerateRecipe
from my_package.msg import Ingridients, Steps
from geometry_msgs.msg import Pose
from dataclasses import dataclass

# Structured ArmStep
@dataclass #class is mostly for storing data
class ArmStep:
    action: str          # 'pick', 'place', 'cut', 'stir' 
    target: str = None   # ingredient name
    tool: str = None     # 'gripper', 'knife', 'spoon'
    pose: Pose = None    # target pose


class controlNode(Node):
    # need to update so that tools are behind robot, ingredients in front and pans are to the left of the robot 
    def __init__(self):
        super().__init__('arm_control_node')

        # Clients
        self.detect_client = self.create_client(DetectIngredients, 'detect_ingredients')
        self.recipe_client = self.create_client(GenerateRecipe, 'generate_recipe')

        # Publisher for arm goal
        self.publisher_ = self.create_publisher(Pose, 'arm_goal', 10)

        # Grid scaling
        self.grid_size = 0.1  # meters per grid unit
        self.home_position = (0.0, 0.0, 0.5)
        self.current_tool = None

        # Tool positions (static, for picking tools)
        self.tool_positions = {
            'gripper': (0.0, 0.0, 0.1),
            'knife': (0.1, 0.0, 0.1),
            'spoon': (0.2, 0.0, 0.1)
        }

        # Wait for services
        while not self.detect_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for detect_ingredients service...')
        while not self.recipe_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for generate_recipe service...')

        # Execute workflow
        self.execute_workflow()

    # ------------------------------
    # Main workflow
    # ------------------------------
    def execute_workflow(self):
        # Step 1: Detect ingredients dynamically
        detect_req = DetectIngredients.Request()
        future = self.detect_client.call_async(detect_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error('Failed to detect ingredients')
            return

        detected_ingredients = future.result().ingredients
        self.get_logger().info(f'Detected {len(detected_ingredients)} ingredients.')

        # Step 2: Generate recipe
        ingredient_names = [ing.name for ing in detected_ingredients]
        recipe_req = GenerateRecipe.Request()
        recipe_req.ingredients = ingredient_names
        future = self.recipe_client.call_async(recipe_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error('Failed to generate recipe')
            return

        recipe = future.result()
        self.process_recipe(recipe, detected_ingredients)

        # Return home at the end
        self.return_tool()

    # ------------------------------
    # Process Recipe
    # ------------------------------
    def process_recipe(self, recipe, detected_ingredients):
        for step_num, step_desc in zip(recipe.step_numbers, recipe.step_descriptions):
            self.get_logger().info(f'Processing Step {step_num}: {step_desc}')
            arm_steps = self.parse_recipe_step(step_desc, detected_ingredients)
            self.execute_arm_steps(arm_steps)

    # ------------------------------
    # Parse textual step into ArmSteps
    # ------------------------------
    def parse_recipe_step(self, step_desc, detected_ingredients):
        step_desc_lower = step_desc.lower()
        arm_steps = []

        for ing_msg in detected_ingredients:
            if ing_msg.name.lower() in step_desc_lower:
                pose = self.get_ingredient_pose(ing_msg)

                if 'pick' in step_desc_lower:
                    arm_steps.append(ArmStep('pick', target=ing_msg.name, tool='gripper', pose=pose))
                if 'place' in step_desc_lower:
                    arm_steps.append(ArmStep('place', target=ing_msg.name, tool='gripper', pose=pose))
                if 'cut' in step_desc_lower:
                    arm_steps.append(ArmStep('cut', target=ing_msg.name, tool='knife', pose=pose))
                if 'stir' in step_desc_lower:
                    arm_steps.append(ArmStep('stir', tool='spoon', pose=pose))

        return arm_steps

    # ------------------------------
    # Execute ArmSteps
    # ------------------------------
    def execute_arm_steps(self, arm_steps):
        for step in arm_steps:
            if step.tool:
                self.switch_tool(step.tool)
            if step.pose:
                self.publisher_.publish(step.pose)
            self.get_logger().info(f"Executing {step.action} on {step.target} using {step.tool}")

    # ------------------------------
    # Pose generation from ingredient detection
    # ------------------------------
    def get_ingredient_pose(self, ing_msg):
        pose = Pose()
        pose.position.x = ing_msg.x_grid * self.grid_size
        pose.position.y = ing_msg.y_grid * self.grid_size
        pose.position.z = ing_msg.z_grid * self.grid_size
        pose.orientation.w = 1.0
        return pose

    # ------------------------------
    # Tool management
    # ------------------------------
    def switch_tool(self, tool_name):
        if self.current_tool != tool_name:
            self.return_tool()
            if tool_name in self.tool_positions:
                x, y, z = self.tool_positions[tool_name]
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z
                pose.orientation.w = 1.0
                self.publisher_.publish(pose)
                self.get_logger().info(f'Switching to tool {tool_name} at ({x:.2f}, {y:.2f}, {z:.2f})')
                self.current_tool = tool_name

    def return_tool(self):
        if self.current_tool:
            x, y, z = self.tool_positions[self.current_tool]
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0
            self.publisher_.publish(pose)
            self.get_logger().info(f'Returning tool {self.current_tool} to ({x:.2f}, {y:.2f}, {z:.2f})')
        self.current_tool = None

        # Move arm to home position
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.home_position
        pose.orientation.w = 1.0
        self.publisher_.publish(pose)
        self.get_logger().info(f'Moving to home position {self.home_position}')


def main(args=None):
    rclpy.init(args=args)
    node = controlNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
