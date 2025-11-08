#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from my_package.srv import DetectIngredients
from my_package.msg import Ingredients
from ultralytics import YOLO
import cv2
import numpy as np
import json

# GPIO library for ultrasonic sensor (choose based on your platform)
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("RPi.GPIO not available - ultrasonic sensor disabled")
    GPIO_AVAILABLE = False

import time


class UltrasonicSensor:
    """HC-SR04 Ultrasonic Sensor Interface"""
    
    def __init__(self, trigger_pin=23, echo_pin=24, max_distance=4.0):
        """
        Initialize ultrasonic sensor
        
        Args:
            trigger_pin: GPIO pin for trigger
            echo_pin: GPIO pin for echo
            max_distance: Maximum reliable distance in meters
        """
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.max_distance = max_distance
        self.available = False
        
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.trigger_pin, GPIO.OUT)
                GPIO.setup(self.echo_pin, GPIO.IN)
                GPIO.output(self.trigger_pin, False)
                time.sleep(0.1)  # Settle time
                self.available = True
                print(f"✓ Ultrasonic sensor initialized on GPIO {trigger_pin}/{echo_pin}")
            except Exception as e:
                print(f"✗ Ultrasonic sensor initialization failed: {e}")
                self.available = False
    
    def get_distance(self, samples=3):
        """
        Measure distance using ultrasonic sensor
        
        Args:
            samples: Number of measurements to average
            
        Returns:
            Distance in meters, or None if measurement failed
        """
        if not self.available:
            return None
        
        measurements = []
        
        for _ in range(samples):
            try:
                # Send trigger pulse
                GPIO.output(self.trigger_pin, True)
                time.sleep(0.00001)  # 10 microseconds
                GPIO.output(self.trigger_pin, False)
                
                # Wait for echo start
                timeout_start = time.time()
                while GPIO.input(self.echo_pin) == 0:
                    pulse_start = time.time()
                    if (pulse_start - timeout_start) > 0.1:  # 100ms timeout
                        break
                
                # Wait for echo end
                timeout_start = time.time()
                while GPIO.input(self.echo_pin) == 1:
                    pulse_end = time.time()
                    if (pulse_end - timeout_start) > 0.1:  # 100ms timeout
                        break
                
                # Calculate distance
                pulse_duration = pulse_end - pulse_start
                distance = pulse_duration * 17150  # Speed of sound = 34300 cm/s
                distance = distance / 100.0  # Convert to meters
                
                # Validate measurement
                if 0.02 < distance < self.max_distance:
                    measurements.append(distance)
                
                time.sleep(0.06)  # 60ms between measurements (HC-SR04 minimum)
                
            except Exception as e:
                print(f"Ultrasonic measurement error: {e}")
                continue
        
        if measurements:
            # Return median to filter outliers
            measurements.sort()
            return measurements[len(measurements) // 2]
        
        return None
    
    def cleanup(self):
        """Cleanup GPIO pins"""
        if self.available and GPIO_AVAILABLE:
            GPIO.cleanup([self.trigger_pin, self.echo_pin])


class webcamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')
        
        # Declare ROS parameters
        self.declare_parameter('image_mode', True)
        self.declare_parameter('image_path', '/root/meal-plan/src/my_package/images/food2.jpg')
        self.declare_parameter('use_ultrasonic', True)
        self.declare_parameter('ultrasonic_trigger_pin', 23)
        self.declare_parameter('ultrasonic_echo_pin', 24)
        
        # Get parameters
        self.image_mode = self.get_parameter('image_mode').value
        self.image_path = self.get_parameter('image_path').value
        use_ultrasonic = self.get_parameter('use_ultrasonic').value
        trigger_pin = self.get_parameter('ultrasonic_trigger_pin').value
        echo_pin = self.get_parameter('ultrasonic_echo_pin').value
        
        # Vision parameters
        self.confidence_threshold = 0.5
        self.min_pixel_size = 30
        self.image_width = 1280
        self.image_height = 720
        self.focal_length = 600  # Camera calibration
        
        # Distance measurement fallback
        self.real_widths = {
            'milk': 0.1, 'egg': 0.05, 'cheese': 0.15, 'bottle': 0.08,
            'cup': 0.08, 'bowl': 0.15, 'apple': 0.08, 'orange': 0.08,
            'banana': 0.12, 'carrot': 0.03,
        }
        
        self.output_file = 'detected_ingredients.json'
        
        # Initialize ultrasonic sensor
        self.ultrasonic = None
        if use_ultrasonic:
            self.ultrasonic = UltrasonicSensor(
                trigger_pin=trigger_pin,
                echo_pin=echo_pin,
                max_distance=4.0
            )
            if self.ultrasonic.available:
                self.get_logger().info('✓ Ultrasonic sensor enabled')
            else:
                self.get_logger().warn('⚠ Ultrasonic sensor unavailable - using vision-based estimation')
        else:
            self.get_logger().info('Ultrasonic sensor disabled by parameter')
        
        # Initialize camera
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
        
        # Load YOLO model
        try:
            self.model = YOLO('ingredients.pt')
            self.get_logger().info('✓ YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.model = None
        
        # Create detection service
        self.srv = self.create_service(
            DetectIngredients,
            'detect_ingredients',
            self.detect_ingredients_callback
        )
        
        self.get_logger().info('Webcam node initialized successfully')
    
    def estimate_distance_from_pixels(self, pixel_width, label):
        """
        Fallback method: estimate distance from pixel width
        
        Args:
            pixel_width: Width in pixels
            label: Object label for real-world size lookup
            
        Returns:
            Estimated distance in meters
        """
        real_width = self.real_widths.get(label, 0.08)
        distance = (real_width * self.focal_length / pixel_width) if pixel_width > 0 else 0.5
        distance = max(0.15, min(distance, 2.5))
        return distance
    
    def get_distance_measurement(self, pixel_width, label):
        """
        Get distance using ultrasonic sensor with vision-based fallback
        
        Args:
            pixel_width: Width of detected object in pixels
            label: Object label
            
        Returns:
            Distance in meters
        """
        # Try ultrasonic sensor first
        if self.ultrasonic and self.ultrasonic.available:
            ultrasonic_distance = self.ultrasonic.get_distance(samples=3)
            
            if ultrasonic_distance is not None:
                self.get_logger().debug(
                    f'Ultrasonic distance for {label}: {ultrasonic_distance:.3f}m'
                )
                return ultrasonic_distance
            else:
                self.get_logger().warn(
                    f'Ultrasonic measurement failed for {label}, using vision fallback'
                )
        
        # Fallback to vision-based estimation
        vision_distance = self.estimate_distance_from_pixels(pixel_width, label)
        self.get_logger().debug(
            f'Vision-based distance for {label}: {vision_distance:.3f}m'
        )
        return vision_distance
    
    def detect_ingredients_callback(self, request, response):
        """Service callback for ingredient detection"""
        detected_data = []
        
        # Handle image mode
        if self.image_mode:
            frame = cv2.imread(self.image_path)
            if frame is None:
                self.get_logger().error(f"Failed to read image from {self.image_path}")
                self._populate_hardcoded_ingredients(response)
                return response
            self.get_logger().info(f"Loaded image from {self.image_path}")
        else:
            # Check camera availability
            if self.cap is None or not self.cap.isOpened() or self.model is None:
                self.get_logger().error("Camera or YOLO unavailable — using hardcoded ingredients")
                self._populate_hardcoded_ingredients(response)
                return response
            
            # Capture frame
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
                
                # Get distance using ultrasonic sensor with fallback
                distance = self.get_distance_measurement(pixel_width, label)
                
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
                    'distance': float(distance),
                    'measurement_method': 'ultrasonic' if (self.ultrasonic and self.ultrasonic.available) else 'vision'
                })
                
                self._draw_detection(
                    frame, label, confidence, distance,
                    int(x1), int(y1), int(x2), int(y2),
                    int(center_x), int(center_y)
                )
        
        # Save detection results
        with open(self.output_file, 'w') as f:
            json.dump(detected_data, f, indent=4)
        
        if detected_data:
            self.get_logger().info(f"✓ Detected {len(detected_data)} ingredients")
            for item in detected_data:
                self.get_logger().info(
                    f"  - {item['name']}: {item['distance']:.2f}m ({item['measurement_method']})"
                )
        else:
            self.get_logger().warn("No ingredients detected — returning fallback list")
            self._populate_hardcoded_ingredients(response)
        
        # Display result
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
        """Draw detection visualization on frame"""
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        
        # Display method indicator
        method = "US" if (self.ultrasonic and self.ultrasonic.available) else "VIS"
        text = f"{label} {confidence:.2f} ({distance:.2f}m-{method})"
        
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
        cv2.rectangle(frame, (x1, y1 - th - 5), (x1 + tw, y1), (0, 255, 0), -1)
        cv2.putText(frame, text, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    
    def destroy_node(self):
        """Cleanup resources"""
        if self.cap:
            self.cap.release()
        if self.ultrasonic:
            self.ultrasonic.cleanup()
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