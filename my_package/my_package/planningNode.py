import rclpy
from rclpy.node import Node
from my_package.srv import GenerateRecipe
from my_package.msg import Steps
import requests
import re

class planningNode(Node):
    def __init__(self):
        super().__init__('meal_planner_node')
        self.srv = self.create_service(GenerateRecipe, 'generate_recipe', self.generate_recipe_callback)
        self.ollama_url = 'http://localhost:11434/api/generate'

    def generate_recipe_callback(self, request, response):
        ingredients = ', '.join(set(request.ingredient_names))  # Remove duplicates
        prompt = (
            f"Generate a simple recipe using only the following ingredients: {ingredients}. "
            "Provide the recipe name followed by a list of detailed preparation steps. "
            "Format the response as: 'Recipe: [name]\nSteps:\n1. [step 1]\n2. [step 2]\n...' "
            "Ensure the recipe is concise and uses only the listed ingredients."
        )

        # Call Ollama API
        try:
            api_response = requests.post(
                self.ollama_url,
                json={'model': 'llama3', 'prompt': prompt, 'stream': False}
            )
            api_response.raise_for_status()
            recipe_text = api_response.json().get('response', '')
        except requests.RequestException as e:
            self.get_logger().error(f'Ollama API call failed: {e}')
            return response

        # Parse recipe
        recipe_match = re.search(r'Recipe: (.*?)\nSteps:', recipe_text, re.DOTALL)
        steps_match = re.findall(r'(\d+)\.\s*(.*?)(?=\n\d+\.|\n|$)', recipe_text, re.DOTALL)

        if recipe_match and steps_match:
            recipe_name = recipe_match.group(1).strip()
            response.recipe = MealSteps()
            response.recipe.step_numbers = []
            response.recipe.step_descriptions = []
            for step_num, step_desc in steps_match:
                response.recipe.step_numbers.append(int(step_num))
                response.recipe.step_descriptions.append(step_desc.strip())
                self.get_logger().info(f'Meal: {recipe_name}, Step {step_num}: {step_desc.strip()}')
        else:
            self.get_logger().warn('No valid recipe or steps found in Ollama response')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = planningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()