import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients
from my_package.msg import Ingridients
import cv2
import sys

# Assuming you already have a YOLO model loaded in webcamNode
# e.g. model = YOLO("path/to/best.pt")
from ultralytics import YOLO  


class ImageNode(Node):
    def __init__(self, image_path):
        super().__init__('image_node')
        self.image_path = image_path
        self.cli = self.create_client(DetectIngredients, 'detect_ingredients')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for detect_ingredients service...')

        # Load YOLO model (same as in webcamNode)
        self.model = YOLO("path/to/best.pt")  # 🔧 change path to your weights

        self.send_request()

    def send_request(self):
        img = cv2.imread(self.image_path)

        if img is None:
            self.get_logger().error(f"Could not read image at {self.image_path}")
            rclpy.shutdown()
            return

        # Run YOLO detection
        results = self.model(img)

        req = DetectIngredients.Request()

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                label = self.model.names[cls]
                x_center = float(box.xywh[0][0].cpu().numpy())
                y_center = float(box.xywh[0][1].cpu().numpy())

                ingredient_msg = Ingridients()
                ingredient_msg.name = label
                ingredient_msg.x_center = x_center
                ingredient_msg.y_center = y_center
                ingredient_msg.x_grid = x_center / img.shape[1]
                ingredient_msg.y_grid = y_center / img.shape[0]
                ingredient_msg.z_grid = 0.0
                ingredient_msg.distance = 0.0
                req.ingredients.append(ingredient_msg)

        future = self.cli.call_async(req)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Received ingredients: {[ing.name for ing in response.ingredients]}"
            )
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        finally:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run my_package image_node <image_path>")
        return

    image_path = sys.argv[1]
    node = ImageNode(image_path)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
