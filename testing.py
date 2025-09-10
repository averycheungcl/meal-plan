# Keep webcamNode running in Terminal 1, then test this in Terminal 2:
python3 -c "
import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients

rclpy.init()
node = Node('test_client')
client = node.create_client(DetectIngredients, 'detect_ingredients')

while not client.wait_for_service(timeout_sec=1.0):
    print('Waiting...')

req = DetectIngredients.Request()
future = client.call_async(req)
rclpy.spin_until_future_complete(node, future)

result = future.result()
print('Got service response')

# This is the exact line from your controlNode that might fail:
detected_ingredients = future.result().ingredients
print(f'Extracted ingredients: {len(detected_ingredients)}')

# This is the next line that might fail:
ingredient_names = [ing.name for ing in detected_ingredients]
print(f'Ingredient names: {ingredient_names}')

# Test accessing other fields like your controlNode does:
for ing in detected_ingredients:
    print(f'Ingredient: {ing.name}, x_grid: {ing.x_grid}, y_grid: {ing.y_grid}')

node.destroy_node() 
rclpy.shutdown()
"
