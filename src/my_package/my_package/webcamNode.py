#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients
from my_package.msg import Ingredients
from ultralytics import YOLO
import cv2
import numpy as np
import json

class webcamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')
        self.image_mode = True  # Set to True to use a static image for detection
        self.image_path = '/root/meal-plan/food2.jpg'  # Path to the static image
        # --- Parameters ---
        self.confidence_threshold = 0.5
        self.min_pixel_size = 30
        self.image_width = 1280
        self.image_height = 720
        self.focal_length = 600  # camera calibration

        # Known object widths in meters
        self.real_widths = {
            'milk': 0.1, 'egg': 0.05, 'cheese': 0.15, 'bottle': 0.08,
            'cup': 0.08, 'bowl': 0.15, 'apple': 0.08, 'orange': 0.08,
            'banana': 0.12, 'carrot': 0.03,
        }

        self.output_file = 'detected_ingredients.json'
        
        # --- Initialize camera ---
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam - running in fallback mode")
            self.cap = None
        else:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.get_logger().info(f"Webcam resolution: {w}x{h}")
        
        # --- Load YOLO model ---
        try:
            self.model = YOLO('ingredients.pt')
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.model = None
        
        # --- Create the detection service ---
        self.srv = self.create_service(
            DetectIngredients,
            'detect_ingredients',
            self.detect_ingredients_callback
        )

        self.get_logger().info('Webcam node initialized successfully')

    def detect_ingredients_callback(self, request, response):
        """Service callback for ingredient detection"""
        detected_data = []

        if self.image_mode:
            frame = cv2.imread(self.image_path)
            if frame is None:
                raise ValueError(f"Failed to read image from {self.image_path}")
            self.get_logger().info(f"Loaded image from {self.image_path} for detection.")
        else:
            # If camera or model unavailable → fallback
            if self.cap is None or not self.cap.isOpened() or self.model is None:
                self.get_logger().error("Camera or YOLO unavailable — using hardcoded ingredients")
                self._populate_hardcoded_ingredients(response)
                self.get_logger().info(f"Service returning ingredients: {[ing.name for ing in response.ingredients]}")

                return response
        
        # Capture image
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to capture frame — using hardcoded ingredients")
                self._populate_hardcoded_ingredients(response)
                return response


    
        

        # Run YOLO inference
        results = self.model(frame, imgsz=self.image_width, verbose=False)

        for result in results:
            for box in result.boxes:
                label = result.names[int(box.cls)]
                confidence = float(box.conf[0])
                if confidence < self.confidence_threshold:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                pixel_width = x2 - x1
                pixel_height = y2 - y1

                if pixel_width < self.min_pixel_size or pixel_height < self.min_pixel_size:
                    continue

                center_x = float((x1 + x2) / 2.0)
                center_y = float((y1 + y2) / 2.0)

                real_width = self.real_widths.get(label, 0.08)
                distance = (real_width * self.focal_length / pixel_width) if pixel_width > 0 else 0.5
                distance = max(0.15, min(distance, 2.5))

                # Build Ingredients message
                ingredient_msg = Ingredients()
                ingredient_msg.name = str(label)
                ingredient_msg.x_center = float(center_x)
                ingredient_msg.y_center = float(center_y)
                ingredient_msg.x_grid = 0.0
                ingredient_msg.y_grid = 0.0
                ingredient_msg.z_grid = 0.0
                ingredient_msg.distance = float(distance)


                response.ingredients.append(ingredient_msg)

                detected_data.append({
                    'name': str(label),
                    'confidence': float(confidence),
                    'center_x': float(center_x),
                    'center_y': float(center_y),
                    'distance': float(distance)
                })


                self._draw_detection(
                    frame, label, confidence, distance, 
                    int(x1), int(y1), int(x2), int(y2), 
                    int(center_x), int(center_y)
                )

        # Save JSON for debugging/logging
        with open(self.output_file, 'w') as f:
            json.dump(detected_data, f, indent=4)

        if detected_data:
            self.get_logger().info(f"Detected {len(detected_data)} ingredients")
        else:
            self.get_logger().info("No ingredients detected — returning fallback list")
            self._populate_hardcoded_ingredients(response)

        # Optional: show result briefly
        cv2.imshow('Ingredient Detection', frame)
        cv2.waitKey(3000)
        cv2.destroyAllWindows()

        return response

    def _populate_hardcoded_ingredients(self, response):
        """Adds default ingredient list to the service response"""
        hardcoded_items = [
            'flour', 'sugar', 'salt', 'butter', 'milk',
            'eggs', 'rice', 'pasta', 'tomatoes', 'onions',
            'garlic', 'chicken', 'beef', 'carrots', 'potatoes', 'cheese'
        ]

        for item in hardcoded_items:
            ingr = Ingredients()
            ingr.name = item
            ingr.x_center = 0.0
            ingr.y_center = 0.0
            ingr.x_grid = 0.0
            ingr.y_grid = 0.0
            ingr.z_grid = 0.0
            ingr.distance = 0.0
            response.ingredients.append(ingr)

        self.get_logger().info(f"Returned {len(hardcoded_items)} hardcoded ingredients")

    def _draw_detection(self, frame, label, confidence, distance, 
                        x1, y1, x2, y2, center_x, center_y):
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        text = f"{label} {confidence:.2f} ({distance:.2f}m)"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        cv2.rectangle(frame, (x1, y1 - th - 5), (x1 + tw, y1), (0, 255, 0), -1)
        cv2.putText(frame, text, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = webcamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
