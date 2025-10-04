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
        
        # Initialize webcam FIRST
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam - node will continue without camera")
            self.cap = None
            # DON'T call rclpy.shutdown() here!
            return

        # Set webcam resolution
        self.image_width = 1280
        self.image_height = 720
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Webcam resolution: {width}x{height}")
        
        # Now initialize YOLO and service
        self.model = YOLO('yolov8n.pt')
        self.srv = self.create_service(DetectIngredients, 'detect_ingredients', self.detect_ingredients_callback)
        
        # Camera calibration parameters
        self.focal_length = 600
        
        # Known object widths in meters
        self.real_widths = {
            'milk': 0.1, 'egg': 0.05, 'cheese': 0.15, 'bottle': 0.08,
            'cup': 0.08, 'bowl': 0.15, 'apple': 0.08, 'orange': 0.08,
            'banana': 0.12, 'carrot': 0.03,
        }
        
        self.output_file = 'detected_ingredients.json'
        self.get_logger().info('Webcam node initialized successfully')

    def detect_ingredients_callback(self, request, response):
        """Service callback for ingredient detection"""
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error("Webcam not available")
            return response
            
        detected_data = []
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return response

        results = self.model(frame, imgsz=self.image_width, verbose=False)

        for result in results:
            for box in result.boxes:
                label = result.names[int(box.cls)]
                confidence = float(box.conf[0])
                
                # Filter by confidence
                if confidence < self.confidence_threshold:
                    continue
                
                # Get bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                pixel_width = x2 - x1
                pixel_height = y2 - y1
                
                # Filter tiny detections
                if pixel_width < self.min_pixel_size or pixel_height < self.min_pixel_size:
                    continue

                # Calculate center in pixel coordinates
                center_x = float((x1 + x2) / 2.0)
                center_y = float((y1 + y2) / 2.0)

                # Estimate distance using pinhole camera model
                real_width = self.real_widths.get(label, 0.08)  # Default 8cm
                
                if pixel_width > 0:
                    # Distance = (Real_Width * Focal_Length) / Pixel_Width
                    distance = (real_width * self.focal_length) / pixel_width
                    
                    #   Add uncertainty bounds (±30% typical error)
                    distance_min = distance * 0.7
                    distance_max = distance * 1.3
                    
                    # Clamp to reasonable range
                    distance = max(0.15, min(distance, 2.5))
                else:
                    distance = 0.5  # Default fallback
                    distance_min = 0.35
                    distance_max = 0.65

                #   FIXED: Store actual pixel coordinates, not normalized
                # The controlNode will handle the conversion to 3D
                ingredient_msg = Ingredients()
                ingredient_msg.name = label
                ingredient_msg.x_center = center_x  # Actual pixel coordinate
                ingredient_msg.y_center = center_y  # Actual pixel coordinate
                ingredient_msg.x_grid = 0.0  # Unused (kept for compatibility)
                ingredient_msg.y_grid = 0.0  # Unused (kept for compatibility)
                ingredient_msg.z_grid = 0.0  # Unused
                ingredient_msg.distance = distance
                response.ingredients.append(ingredient_msg)

                # Store detection for JSON logging
                detection = {
                    'name': label,
                    'confidence': float(confidence),
                    'center_x': center_x,
                    'center_y': center_y,
                    'distance': distance,
                    'distance_range': [float(distance_min), float(distance_max)],
                    'bbox': [float(x1), float(y1), float(x2), float(y2)],
                    'pixel_width': float(pixel_width),
                    'real_width_assumed': real_width
                }
                detected_data.append(detection)

                # Draw on frame
                self._draw_detection(
                    frame, label, confidence, distance, 
                    int(x1), int(y1), int(x2), int(y2), 
                    int(center_x), int(center_y)
                )

        if detected_data:
            self.get_logger().info(f"Detected {len(detected_data)} objects")
        else:
            self.get_logger().info("No objects detected")

        with open(self.output_file, 'w') as f:
            json.dump(detected_data, f, indent=4)

        cv2.imshow('Ingredient Detection', frame)
        cv2.waitKey(1000)  # Show for 1 second instead of waiting indefinitely
        cv2.destroyAllWindows()

        return response

    def _draw_detection(self, frame, label, confidence, distance, 
                       x1, y1, x2, y2, center_x, center_y):
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        
        label_text = f"{label} {confidence:.2f} ({distance:.2f}m)"
        (text_width, text_height), baseline = cv2.getTextSize(
            label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
        )
        
        cv2.rectangle(frame, (x1, y1 - text_height - 10), 
                     (x1 + text_width, y1), (0, 255, 0), -1)
        cv2.putText(frame, label_text, (x1, y1 - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = webcamNode()
    
    # Even if camera fails, keep the node alive to provide the service
    # (it will just return empty results)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()