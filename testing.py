# Test if the issue is in DetectIngredients response processing
python3 -c "
import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients

rclpy.init()
node = Node('test_client')
client = node.create_client(DetectIngredients, 'detect_ingredients')

# Wait for service (like your code does)
while not client.wait_for_service(timeout_sec=1.0):
    print('Waiting...')

# Make the call
req = DetectIngredients.Request()
future = client.call_async(req)
rclpy.spin_until_future_complete(node, future)

# This is where the assertion likely fails
result = future.result()
if result:
    ingredients = result.ingredients  # <-- Assertion error likely here
    print(f'Got {len(ingredients)} ingredients')
    
node.destroy_node()
rclpy.shutdown()
"
