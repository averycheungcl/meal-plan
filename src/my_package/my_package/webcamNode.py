#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients
from my_package.msg import Ingredients
from ultralytics import YOLO
import cv2
import numpy as np
import json

#need to use sensor.msgs for image maybe 

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
    
                # Get bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                pixel_width = x2 - x1
    
                # Compute center coordinates (in pixels)
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
    
                # Estimate distance (z in meters)
                real_width = self.real_widths.get(label, 0.1)  # Default width if unknown
                z_cam = (real_width * self.fx) / pixel_width if pixel_width > 0 else 0
    
                # Convert to camera frame coordinates (meters)
                x_cam = (center_x - self.cx) * z_cam / self.fx
                y_cam = (center_y - self.cy) * z_cam / self.fy
    
                # Store detection for JSON
                detection = {
                    'name': label,
                    'x_cam': float(x_cam),
                    'y_cam': float(y_cam),
                    'z_cam': float(z_cam)
                }
                detected_data.append(detection)
    
                # Create Ingredient message
                ingredient_msg = Ingredients()
                ingredient_msg.name = label
                ingredient_msg.x_center = float(center_x)
                ingredient_msg.y_center = float(center_y)
                ingredient_msg.x_cam = float(x_cam)
                ingredient_msg.y_cam = float(y_cam)
                ingredient_msg.z_cam = float(z_cam)
                response.ingredients.append(ingredient_msg)
    
                # Draw bounding box
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                label_text = f"{label} ({x_cam:.2f}, {y_cam:.2f}, {z_cam:.2f} m)"
                cv2.putText(
                    frame, label_text, (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                )
    
        # Log detections
        if detected_data:
            self.get_logger().info("Detected Objects (camera frame in meters):")
            for det in detected_data:
                self.get_logger().info(
                    f"Name: {det['name']}, Pose: (x={det['x_cam']:.2f}, y={det['y_cam']:.2f}, z={det['z_cam']:.2f})"
                )
        else:
            self.get_logger().info("No objects detected")
    
        # Save to JSON
        with open(self.output_file, 'w') as f:
            json.dump(detected_data, f, indent=4)
        self.get_logger().info(f"Detections stored in {self.output_file}")
    
        # Display image
        cv2.imshow('Webcam Detection', frame)
        cv2.waitKey(1)  # don’t block execution
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
