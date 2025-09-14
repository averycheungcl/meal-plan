#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_package.srv import GenerateRecipe
from my_package.msg import Steps
import ollama
import re

class planningNode(Node):
    def __init__(self):
        super().__init__('meal_planner_node')
        self.srv = self.create_service(GenerateRecipe, 'generate_recipe', self.generate_recipe_callback)

    def generate_recipe_callback(self, request, response):
        # Keep original line (even though we overwrite it later)
        ingredient_names = [ing.name for ing in request.ingredients]
        ingredients = ', '.join(ingredient_names) # Remove duplicates

        # Hardcoded grocery list (overrides request.ingredients)
        common_groceries = [
            "flour", "sugar", "salt", "butter", "milk", "eggs",
            "rice", "pasta", "tomatoes", "onions", "garlic",
            "chicken", "beef", "carrots", "potatoes", "cheese"
        ]
        ingredients = ', '.join(common_groceries)

        # Build prompt for multiple recipes
        prompt = (
            f"Generate 2 different recipes using only the following ingredients: {ingredients}. "
            "For each recipe, provide the recipe name followed by a numbered list of detailed preparation steps. "
            "Format the response exactly like this:\n\n"
            "Recipe: [name]\n"
            "Steps:\n"
            "1. [step 1]\n"
            "2. [step 2]\n"
            "...\n\n"
            "Recipe: [name]\n"
            "Steps:\n"
            "1. [step 1]\n"
            "2. [step 2]\n"
            "...\n\n"
            "Continue until all recipes are listed. Ensure the recipes are concise and use only the listed ingredients."
        )

        # Call Ollama locally
        try:
            self.get_logger().info('Calling Ollama locally...')
            result = ollama.chat(model='llama3.2:1b', messages=[{'role': 'user', 'content': prompt}])
            recipe_text = result['message']['content']
            self.get_logger().info('Ollama call complete')
            self.get_logger().info(f'result :{result}')
        except Exception as e:
            self.get_logger().error(f'Ollama local call failed: {e}')
            return response

        # Split the response into separate recipes
        recipes = re.split(r'\n?Recipe:\s*', recipe_text)
        for recipe_block in recipes[1:]:  # skip the first empty split
            # Extract recipe name
            recipe_name_match = re.match(r'(.*?)\nSteps:', recipe_block, re.DOTALL)
            steps_match = re.findall(r'(\d+)\.\s*(.*?)(?=\n\d+\.|\n|$)', recipe_block, re.DOTALL)

            if recipe_name_match and steps_match:
                recipe_name = recipe_name_match.group(1).strip()
                response.recipe_name = recipe_name  # assign the name of the last parsed recipe
                for step_num, step_desc in steps_match:
                    step_msg = Steps()
                    step_msg.step_numbers = [int(step_num)]
                    step_msg.step_descriptions = [step_desc.strip()]
                    response.steps.append(step_msg)
                    self.get_logger().info(f'Meal: {recipe_name}, Step {step_num}: {step_desc.strip()}')
            else:
                self.get_logger().warning('No valid recipe or steps found in Ollama response')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = planningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
