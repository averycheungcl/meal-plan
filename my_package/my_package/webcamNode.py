mport rclpy
from rclpy.node import Node
from my_package.srv import detectIngredients
from my_package.msg import ingredients
from ultralytics import YOLO
import cv2
import numpy as np
import json

class webcamNode(Node):
    def __init__(self):
        super().__init__('webcam_yolo_node')
        self.srv = self.create_service(DetectIngredients, 'detect_ingredients', self.detect_ingredients_callback)
        self.model = YOLO('yolov8n.pt')  # Replace with fine-tuned model if available
        self.focal_length = 600  # In pixels, from camera calibration
        self.real_widths = {'milk': 0.1, 'egg': 0.05, 'cheese': 0.15}  # Real-world widths in meters
        self.grid_size = 0.1  # 10 cm per cell
        self.fridge_width = 0.5  # 50 cm
        self.fridge_height = 0.5  # 50 cm
        self.output_file = 'detected_ingredients.json'

        # Initialize webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam")
            rclpy.shutdown()
            return

        # Set webcam resolution to 1280x720
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Webcam resolution set to {width}x{height}")

    def detect_ingredients_callback(self, request, response):
        detected_data = []
        # Capture a single frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return response

        # Run YOLO inference
        results = self.model(frame, imgsz=1280, verbose=False)

        for result in results:
            for box in result.boxes:
                label = result.names[int(box.cls)]
                # Uncomment to filter only known ingredients
                # if label not in self.real_widths:
                #     continue

                # Get bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                pixel_width = x2 - x1

                # Compute center coordinates (in pixels)
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                # Estimate distance
                real_width = self.real_widths.get(label, 0.1)  # Default width if unknown
                distance = (real_width * self.focal_length) / pixel_width if pixel_width > 0 else 0

                # Map to grid
                x_grid = (center_x / frame.shape[1]) * self.fridge_width / self.grid_size
                y_grid = (center_y / frame.shape[0]) * self.fridge_height / self.grid_size
                z_grid = distance / self.grid_size

                # Store detection for JSON
                detection = {
                    'name': label,
                    'center_x': float(center_x),
                    'center_y': float(center_y),
                    'distance': float(distance)
                }
                detected_data.append(detection)

                # Create Ingredient message
                ingredient_msg = Ingredient()
                ingredient_msg.name = label
                ingredient_msg.x_center = float(center_x)
                ingredient_msg.y_center = float(center_y)
                ingredient_msg.x_grid = float(x_grid)
                ingredient_msg.y_grid = float(y_grid)
                ingredient_msg.z_grid = float(z_grid)
                ingredient_msg.distance = float(distance)
                response.ingredients.append(ingredient_msg)

                # Draw bounding box
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                label_text = f"{label} ({center_x:.1f}, {center_y:.1f})"
                (text_width, text_height), baseline = cv2.getTextSize(
                    label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
                )
                text_bg_top_left = (int(x1), int(y1) - text_height - baseline - 5)
                text_bg_bottom_right = (int(x1) + text_width, int(y1) - baseline + 5)
                cv2.rectangle(frame, text_bg_top_left, text_bg_bottom_right, (0, 0, 0), -1)
                cv2.putText(
                    frame, label_text, (int(x1), int(y1) - baseline - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                )

        # Log detections
        if detected_data:
            self.get_logger().info("Detected Objects:")
            for det in detected_data:
                self.get_logger().info(
                    f"Name: {det['name']}, Center: ({det['center_x']:.1f}, {det['center_y']:.1f}), Distance: {det['distance']:.2f}m"
                )
        else:
            self.get_logger().info("No objects detected")

        # Save to JSON
        with open(self.output_file, 'w') as f:
            json.dump(detected_data, f, indent=4)
        self.get_logger().info(f"Detections stored in {self.output_file}")

        # Display image
        cv2.imshow('Webcam Detection', frame)
        self.get_logger().info("Press any key to close the image window")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return response

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = webcamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()