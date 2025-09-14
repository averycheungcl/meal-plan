#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients
from my_package.msg import Ingredients
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs
from ultralytics import YOLO
import cv2
import numpy as np
import json

class webcamNode(Node):
    def __init__(self):
        super().__init__('webcam_yolo_node')
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Frame definitions
        self.camera_frame = 'camera_link'
        self.base_frame = 'base_link'
        self.world_frame = 'world'
        
        # Service
        self.srv = self.create_service(DetectIngredients, 'detect_ingredients', self.detect_ingredients_callback)
        
        # YOLO model
        self.model = YOLO('yolov8n.pt')
        
        # Camera parameters (CRITICAL for TF2 coordinate conversion)
        self.camera_matrix = np.array([
            [600.0, 0.0, 640.0],      # fx, 0, cx
            [0.0, 600.0, 360.0],      # 0, fy, cy  
            [0.0, 0.0, 1.0]           # 0, 0, 1
        ])
        self.dist_coeffs = np.zeros((4,1))  # Assuming no distortion
        
        # Real-world object sizes (for depth estimation)
        self.real_widths = {
            'milk': 0.08,      # 8cm width
            'egg': 0.05,       # 5cm width
            'cheese': 0.12,    # 12cm width
            'apple': 0.07,     # 7cm width
            'bottle': 0.06     # 6cm width
        }
        
        # Camera physical setup
        self.camera_height = 0.5  # 50cm above table
        self.table_height = 0.0   # Table at z=0 in world frame
        
        self.output_file = 'detected_ingredients.json'

        # Initialize webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam")
            rclpy.shutdown()
            return

        # Set webcam resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # Publish camera transform
        self.publish_camera_transform()

    def publish_camera_transform(self):
        """Publish camera to base_link transform"""
        timer = self.create_timer(0.1, self.broadcast_camera_transform)

    def broadcast_camera_transform(self):
        """Broadcast camera coordinate frame"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.camera_frame
        
        # Camera positioned above and in front of workspace
        transform.transform.translation.x = 0.2   # 20cm forward
        transform.transform.translation.y = 0.0   # centered
        transform.transform.translation.z = self.camera_height  # 50cm up
        
        # Camera looking down at 45-degree angle
        transform.transform.rotation.x = 0.3827   # ~45 degrees
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 0.9239
        
        self.tf_broadcaster.sendTransform(transform)

    def pixel_to_world_coordinates(self, pixel_x, pixel_y, object_width_pixels, object_name):
        """Convert pixel coordinates to world coordinates using camera calibration"""
        
        # Estimate distance using known object size
        real_width = self.real_widths.get(object_name, 0.08)  # Default 8cm
        if object_width_pixels > 0:
            # Distance = (real_width * focal_length) / pixel_width
            distance = (real_width * self.camera_matrix[0,0]) / object_width_pixels
        else:
            distance = 0.5  # Default 50cm
        
        # Convert pixel coordinates to camera frame coordinates
        # Assuming camera looks down at table
        cx, cy = self.camera_matrix[0,2], self.camera_matrix[1,2]
        fx, fy = self.camera_matrix[0,0], self.camera_matrix[1,1]
        
        # 3D point in camera frame
        x_cam = (pixel_x - cx) * distance / fx
        y_cam = (pixel_y - cy) * distance / fy
        z_cam = -distance  # Negative because camera looks down
        
        # Create point in camera frame
        point_camera = PointStamped()
        point_camera.header.stamp = self.get_clock().now().to_msg()
        point_camera.header.frame_id = self.camera_frame
        point_camera.point.x = x_cam
        point_camera.point.y = y_cam
        point_camera.point.z = z_cam
        
        try:
            # Transform to base_link frame
            point_base = self.tf_buffer.transform(
                point_camera, self.base_frame, 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return point_base.point.x, point_base.point.y, point_base.point.z, distance
            
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}, using approximate coordinates")
            # Fallback to simple approximation
            return x_cam + 0.2, y_cam, 0.05, distance  # Rough workspace position

    def detect_ingredients_callback(self, request, response):
        detected_data = []
        
        # Capture frame
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return response

        # Run YOLO inference
        results = self.model(frame, imgsz=1280, verbose=False)

        for result in results:
            for box in result.boxes:
                label = result.names[int(box.cls)]
                confidence = float(box.conf[0])
                
                # Skip low confidence detections
                if confidence < 0.5:
                    continue

                # Get bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                pixel_width = x2 - x1
                pixel_height = y2 - y1

                # Compute center coordinates
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                # Convert to world coordinates using TF2
                world_x, world_y, world_z, distance = self.pixel_to_world_coordinates(
                    center_x, center_y, pixel_width, label
                )

                # Store detection for JSON
                detection = {
                    'name': label,
                    'confidence': confidence,
                    'pixel_center_x': float(center_x),
                    'pixel_center_y': float(center_y),
                    'world_x': float(world_x),
                    'world_y': float(world_y),
                    'world_z': float(world_z),
                    'distance': float(distance)
                }
                detected_data.append(detection)

                # Create Ingredient message with WORLD coordinates
                ingredient_msg = Ingredients()
                ingredient_msg.name = label
                ingredient_msg.x_center = float(center_x)  # Keep pixel for display
                ingredient_msg.y_center = float(center_y)  # Keep pixel for display
                ingredient_msg.x_grid = float(world_x)     # WORLD coordinates
                ingredient_msg.y_grid = float(world_y)     # WORLD coordinates  
                ingredient_msg.z_grid = float(world_z)     # WORLD coordinates
                ingredient_msg.distance = float(distance)
                response.ingredients.append(ingredient_msg)

                # Draw visualization
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                
                # Show both pixel and world coordinates
                label_text = f"{label} P:({center_x:.0f},{center_y:.0f}) W:({world_x:.2f},{world_y:.2f})"
                cv2.putText(frame, label_text, (int(x1), int(y1) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Log detections
        if detected_data:
            self.get_logger().info("Detected Objects (with world coordinates):")
            for det in detected_data:
                self.get_logger().info(
                    f"Name: {det['name']}, World: ({det['world_x']:.2f}, {det['world_y']:.2f}, {det['world_z']:.2f})"
                )
        else:
            self.get_logger().info("No objects detected")

        # Save to JSON
        with open(self.output_file, 'w') as f:
            json.dump(detected_data, f, indent=4)
        self.get_logger().info(f"Detections with world coordinates saved to {self.output_file}")

        # Display image
        cv2.imshow('Webcam Detection (TF2 Coordinates)', frame)
        self.get_logger().info("Press any key to close the image window")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return response

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = webcamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
