#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients
from my_package.msg import Ingredients
import cv2
from ultralytics import YOLO

class IngredientDetectionServer(Node):
    def __init__(self):
        super().__init__('ingredient_detection_server')

        # Create service
        self.srv = self.create_service(DetectIngredients, 'detect_ingredients', self.detect_callback)
        self.get_logger().info('DetectIngredients service is ready.')

        # Load the YOLO model (update path to your trained weights)
        self.model = YOLO("yolov8n.pt")  # 🔧 Change this path

        # Use a predefined image or webcam frame for detection
        self.image_source = '/home/avery/Downloads/food.jpg'  # 🔧 Change this if needed

    def detect_callback(self, request, response):
        # Read the image
        img = cv2.imread(self.image_source)
        if img is None:
            self.get_logger().error(f"Failed to load image from {self.image_source}")
            return response

        # Run YOLO detection
        results = self.model(img)

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                label = self.model.names[cls]
                x_center = float(box.xywh[0][0].cpu().numpy())
                y_center = float(box.xywh[0][1].cpu().numpy())

                ingredient_msg = Ingredients()
                ingredient_msg.name = label
                ingredient_msg.x_center = x_center
                ingredient_msg.y_center = y_center
                ingredient_msg.x_grid = x_center / img.shape[1]
                ingredient_msg.y_grid = y_center / img.shape[0]
                ingredient_msg.z_grid = 0.0
                ingredient_msg.distance = 0.0

                response.ingredients.append(ingredient_msg)

        self.get_logger().info(f"Detected {len(response.ingredients)} ingredients.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IngredientDetectionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
