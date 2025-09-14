#!/usr/bin/env python3

#need add to package.xml:<depend>tf2_geometry_msgs</depend>
import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients
from my_package.msg import Ingredients  # Fixed: was "Ingridients"
import cv2
import sys

# Assuming you already have a YOLO model loaded in webcamNode
from ultralytics import YOLO  


class ImageNode(Node):
    def __init__(self, image_path):
        super().__init__('image_node')
        self.image_path = image_path
        self.cli = self.create_client(DetectIngredients, 'detect_ingredients')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for detect_ingredients service...')

        # Load YOLO model (update this path to your actual weights)
        self.model = YOLO("yolov8n.pt")  # Change to your fine-tuned model path

        self.send_request()

    def pixel_to_world_coordinates(self, pixel_x, pixel_y, object_width_pixels, object_name):
        """Convert pixel coordinates to world coordinates"""
        # Estimate distance using known object size
        real_width = self.real_widths.get(object_name, 0.08)
        if object_width_pixels > 0:
            distance = (real_width * self.camera_matrix[0,0]) / object_width_pixels
        else:
            distance = 0.5

        # Convert to camera frame coordinates
        cx, cy = self.camera_matrix[0,2], self.camera_matrix[1,2]
        fx, fy = self.camera_matrix[0,0], self.camera_matrix[1,1]
        
        x_cam = (pixel_x - cx) * distance / fx
        y_cam = (pixel_y - cy) * distance / fy
        z_cam = -distance

        # Create point in camera frame
        point_camera = PointStamped()
        point_camera.header.stamp = self.get_clock().now().to_msg()
        point_camera.header.frame_id = self.camera_frame
        point_camera.point.x = x_cam
        point_camera.point.y = y_cam
        point_camera.point.z = z_cam

        try:
            # Transform to base_link
            point_base = self.tf_buffer.transform(
                point_camera, self.base_frame,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return point_base.point.x, point_base.point.y, point_base.point.z, distance
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")
            # Fallback approximation
            return x_cam + 0.2, y_cam, 0.05, distance

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
                confidence = float(box.conf[0])
                
                if confidence < 0.5:  # Skip low confidence
                    continue
                
                # Get bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                pixel_width = x2 - x1
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                # Convert to world coordinates using TF2
                world_x, world_y, world_z, distance = self.pixel_to_world_coordinates(
                    center_x, center_y, pixel_width, label
                )

                ingredient_msg = Ingredients()
                ingredient_msg.name = label
                ingredient_msg.x_center = float(center_x)  # Pixel coords for display
                ingredient_msg.y_center = float(center_y)
                ingredient_msg.x_grid = float(world_x)     # World coordinates
                ingredient_msg.y_grid = float(world_y)
                ingredient_msg.z_grid = float(world_z)
                ingredient_msg.distance = float(distance)
                req.ingredients.append(ingredient_msg)
                
                self.get_logger().info(f"Detected {label}: World({world_x:.2f}, {world_y:.2f}, {world_z:.2f})")

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
        print("Usage: ros2 run my_package webcamTest <image_path>")
        return

    image_path = sys.argv[1]
    node = ImageNode(image_path)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
