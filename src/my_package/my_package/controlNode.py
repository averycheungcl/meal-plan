#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients, GenerateRecipe
from my_package.msg import Ingridients, Steps
from geometry_msgs.msg import Pose
from tf2_ros import TransformListener, Buffer
import sqlite3
import re

class controlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        self.detect_client = self.create_client(DetectIngredients, 'detect_ingredients')
        self.recipe_client = self.create_client(GenerateRecipe, 'generate_recipe')
        self.publisher_ = self.create_publisher(Pose, 'arm_goal', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.grid_size = 0.1  # 10 cm per cell

        # Initialize SQLite database for tool locations
        self.conn = sqlite3.connect('tool_locations.db')
        self.cursor = self.conn.cursor()
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS tools (
                tool_name TEXT PRIMARY KEY,
                x REAL,
                y REAL,
                z REAL
            )
        ''')
        self.tools = {
            'gripper': (0.0, 0.0, 0.1),
            'knife': (0.1, 0.0, 0.1),
            'spoon': (0.2, 0.0, 0.1)
        }
        for tool, (x, y, z) in self.tools.items():
            self.cursor.execute('INSERT OR REPLACE INTO tools VALUES (?, ?, ?, ?)', (tool, x, y, z))
        self.conn.commit()

        self.home_position = (0.0, 0.0, 0.5)
        self.current_tool = None

        # Wait for services to be available
        while not self.detect_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for detect_ingredients service...')
        while not self.recipe_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for generate_recipe service...')

        # Execute the workflow
        self.execute_workflow()

    def execute_workflow(self):
        # Step 1: Call DetectIngredients service
        detect_req = DetectIngredients.Request()
        future = self.detect_client.call_async(detect_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            ingredients = future.result().ingredients
            self.get_logger().info(f'Detected {len(ingredients)} ingredients')
            for ingredient in ingredients:
                self.process_ingredient(ingredient)
        else:
            self.get_logger().error('Failed to detect ingredients')
            return

        # Step 2: Call GenerateRecipe service
        ingredient_names = [ing.name for ing in ingredients]
        recipe_req = GenerateRecipe.Request()
        recipe_req.ingredient_names = ingredient_names
        future = self.recipe_client.call_async(recipe_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            recipe = future.result().recipe
            self.process_recipe(recipe)
        else:
            self.get_logger().error('Failed to generate recipe')
            return

    def process_ingredient(self, ingredient):
        x = ingredient.x_grid * self.grid_size
        y = ingredient.y_grid * self.grid_size
        z = ingredient.z_grid * self.grid_size
        self.switch_tool('gripper')
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0
        self.publisher_.publish(pose)
        self.get_logger().info(f'Moving arm to pick {ingredient.name} at ({x:.2f}, {y:.2f}, {z:.2f})')
        self.return_tool()

    def process_recipe(self, recipe):
        for step_num, step_desc in zip(recipe.step_numbers, recipe.step_descriptions):
            self.get_logger().info(f'Processing Step {step_num}: {step_desc}')
            actions = self.extract_actions(step_desc)
            ingredients = self.extract_ingredients(step_desc)
            self.get_logger().info(f'Actions: {actions}, Ingredients: {ingredients}')
            for action in actions:
                if action in ['pick', 'place']:
                    for ingredient in ingredients:
                        self.get_logger().info(f'Action: {action} ingredient {ingredient}')
                elif action == 'cut':
                    self.switch_tool('knife')
                    self.execute_cut(step_desc)
                elif action == 'stir':
                    self.switch_tool('spoon')
                    self.execute_stir(step_desc)
                self.return_tool()

    def extract_actions(self, step_desc):
        action_keywords = ['pick', 'place', 'cut', 'stir']
        found_actions = []
        step_lower = step_desc.lower()
        for action in action_keywords:
            if action in step_lower:
                found_actions.append(action)
        return found_actions

    def extract_ingredients(self, step_desc):
        known_ingredients = ['milk', 'egg', 'cheese']
        found_ingredients = []
        step_lower = step_desc.lower()
        for ingredient in known_ingredients:
            if ingredient in step_lower:
                found_ingredients.append(ingredient)
        return found_ingredients

    def switch_tool(self, tool_name):
        if self.current_tool != tool_name:
            self.return_tool()
            self.cursor.execute('SELECT x, y, z FROM tools WHERE tool_name = ?', (tool_name,))
            result = self.cursor.fetchone()
            if result:
                x, y, z = result
                pose = Pose()
                pose.position.x = float(x)
                pose.position.y = float(y)
                pose.position.z = float(z)
                pose.orientation.w = 1.0
                self.publisher_.publish(pose)
                self.get_logger().info(f'Switching to tool {tool_name} at ({x:.2f}, {y:.2f}, {z:.2f})')
                self.current_tool = tool_name

    def return_tool(self):
        if self.current_tool:
            self.cursor.execute('SELECT x, y, z FROM tools WHERE tool_name = ?', (self.current_tool,))
            result = self.cursor.fetchone()
            if result:
                x, y, z = result
                pose = Pose()
                pose.position.x = float(x)
                pose.position.y = float(y)
                pose.position.z = float(z)
                pose.orientation.w = 1.0
                self.publisher_.publish(pose)
                self.get_logger().info(f'Returning tool {self.current_tool} to ({x:.2f}, {y:.2f}, {z:.2f})')
            self.current_tool = None
        pose = Pose()
        pose.position.x = float(self.home_position[0])
        pose.position.y = float(self.home_position[1])
        pose.position.z = float(self.home_position[2])
        pose.orientation.w = 1.0
        self.publisher_.publish(pose)
        self.get_logger().info(f'Moving to home position ({self.home_position[0]:.2f}, {self.home_position[1]:.2f}, {self.home_position[2]:.2f})')

    def execute_cut(self, step_desc):
        size = 0.01
        if 'finely' in step_desc.lower():
            size = 0.005
        elif '1cm' in step_desc.lower():
            size = 0.01
        self.get_logger().info(f'Cutting with size {size*100:.1f}cm')
        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.0
        pose.position.z = 0.1
        pose.orientation.w = 1.0
        self.publisher_.publish(pose)
        self.get_logger().info(f'Executing cut at ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})')

    def execute_stir(self, step_desc):
        self.get_logger().info('Executing stir action')
        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.0
        pose.position.z = 0.1
        pose.orientation.w = 1.0
        self.publisher_.publish(pose)
        self.get_logger().info(f'Executing stir at ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})')

    def destroy_node(self):
        self.conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = controlNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
