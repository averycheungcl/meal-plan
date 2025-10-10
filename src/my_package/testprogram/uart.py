#!/usr/bin/env python3
"""
Raspberry Pi to ESP32 Joint Command Test
This mimics the motionControlNode.py behavior for testing

Save this file as: test_esp32_communication.py

Usage:
    python3 test_esp32_communication.py
    python3 test_esp32_communication.py /dev/ttyUSB0
"""

import serial
import json
import time
import sys

class SimpleMotionController:
    def __init__(self, port='/dev/serial0', baud=115200):
        """Initialize serial connection to ESP32"""
        print(f"Attempting to connect to ESP32 on {port}...")
        try:
            self.ser = serial.Serial(port, baud, timeout=2)
            time.sleep(2)  # Wait for connection
            print(f"✓ Connected to ESP32 on {port} at {baud} baud")
        except serial.SerialException as e:
            print(f"✗ Failed to connect: {e}")
            print("\nTroubleshooting:")
            print("  1. Check if ESP32 is connected via USB")
            print("  2. Try: ls /dev/tty* to find the port")
            print("  3. Common ports: /dev/ttyUSB0, /dev/ttyACM0, /dev/serial0")
            sys.exit(1)
    
    def send_joint_command(self, joint_angles, action_type="move", speed=30):
        """
        Send joint angles to ESP32 in format similar to motionControlNode
        
        Args:
            joint_angles: Dict like {'J1': 90, 'J2': 45, 'J3': 0, 'J4': 60, 'J5': 0, 'J6': 0}
            action_type: String describing the action ('move', 'pick', 'place', etc.)
            speed: Movement speed percentage (0-100)
        """
        command = {
            'action': action_type,
            'joints': joint_angles,
            'speed': speed,
            'timestamp': time.time()
        }
        
        # Convert to JSON and send
        json_str = json.dumps(command) + '\n'
        self.ser.write(json_str.encode('utf-8'))
        self.ser.flush()
        
        print(f"\n→ Sent: {json_str.strip()}")
        
        # Wait for acknowledgment and other responses
        timeout = time.time() + 5  # 5 second timeout
        received_ok = False
        
        while time.time() < timeout:
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode('utf-8').strip()
                print(f"← ESP32: {response}")
                
                if response == "OK":
                    received_ok = True
                    break
        
        if not received_ok:
            print("⚠ Warning: No OK acknowledgment received")
            return False
        
        return True
    
    def test_single_servo(self):
        """Test a single servo movement"""
        print("\n" + "="*50)
        print("TEST 1: Single Servo Test (J1 only)")
        print("="*50)
        
        angles = [0, 45, 90, 135, 180, 90]
        for angle in angles:
            joints = {'J1': angle, 'J2': 90, 'J3': 90, 'J4': 90, 'J5': 90, 'J6': 90}
            print(f"\nMoving J1 to {angle}°")
            self.send_joint_command(joints, "test_single", speed=40)
            time.sleep(2)
    
    def test_all_servos_center(self):
        """Center all servos"""
        print("\n" + "="*50)
        print("TEST 2: Center All Servos")
        print("="*50)
        
        center_joints = {
            'J1': 90,
            'J2': 90,
            'J3': 90,
            'J4': 90,
            'J5': 90,
            'J6': 90
        }
        self.send_joint_command(center_joints, "center", speed=25)
        time.sleep(3)
    
    def test_coordinated_movement(self):
        """Test coordinated multi-joint movement"""
        print("\n" + "="*50)
        print("TEST 3: Coordinated Movement")
        print("="*50)
        
        # Position 1
        print("\n→ Position 1: All joints to different angles")
        pos1 = {
            'J1': 45,
            'J2': 60,
            'J3': 30,
            'J4': 120,
            'J5': 45,
            'J6': 15
        }
        self.send_joint_command(pos1, "coord_move", speed=30)
        time.sleep(4)
        
        # Position 2
        print("\n→ Position 2: Mirror position")
        pos2 = {
            'J1': 135,
            'J2': 120,
            'J3': 150,
            'J4': 60,
            'J5': 135,
            'J6': 165
        }
        self.send_joint_command(pos2, "coord_move", speed=30)
        time.sleep(4)
        
        # Return to center
        print("\n→ Returning to center")
        center = {'J1': 90, 'J2': 90, 'J3': 90, 'J4': 90, 'J5': 90, 'J6': 90}
        self.send_joint_command(center, "center", speed=25)
        time.sleep(3)
    
    def test_robot_sequence(self):
        """Simulate a robot arm pick-and-place sequence"""
        print("\n" + "="*50)
        print("TEST 4: Robot Pick-and-Place Simulation")
        print("="*50)
        
        # Home position
        print("\n→ Moving to HOME position")
        home = {'J1': 0, 'J2': 0, 'J3': 0, 'J4': 0, 'J5': 0, 'J6': 0}
        self.send_joint_command(home, "home", speed=20)
        time.sleep(3)
        
        # Approach object
        print("\n→ APPROACH object")
        approach = {'J1': 45, 'J2': 30, 'J3': 60, 'J4': 90, 'J5': 45, 'J6': 0}
        self.send_joint_command(approach, "approach", speed=30)
        time.sleep(3)
        
        # Pick (simulated with angles)
        print("\n→ PICK object")
        pick = {'J1': 45, 'J2': 25, 'J3': 70, 'J4': 100, 'J5': 45, 'J6': 0}
        self.send_joint_command(pick, "pick", speed=15)
        time.sleep(3)
        
        # Lift
        print("\n→ LIFT object")
        lift = {'J1': 45, 'J2': 40, 'J3': 50, 'J4': 80, 'J5': 45, 'J6': 0}
        self.send_joint_command(lift, "lift", speed=20)
        time.sleep(3)
        
        # Move to place position
        print("\n→ MOVE to place position")
        move = {'J1': 135, 'J2': 40, 'J3': 50, 'J4': 80, 'J5': 135, 'J6': 0}
        self.send_joint_command(move, "move_to_place", speed=30)
        time.sleep(3)
        
        # Place
        print("\n→ PLACE object")
        place = {'J1': 135, 'J2': 25, 'J3': 70, 'J4': 100, 'J5': 135, 'J6': 0}
        self.send_joint_command(place, "place", speed=15)
        time.sleep(3)
        
        # Return home
        print("\n→ RETURN to home")
        self.send_joint_command(home, "home", speed=20)
        time.sleep(3)
    
    def run_all_tests(self):
        """Run complete test suite"""
        print("\n" + "#"*50)
        print("# ESP32 Joint Controller Test Suite")
        print("#"*50)
        
        try:
            self.test_all_servos_center()
            input("\nPress Enter to continue to Test 1...")
            
            self.test_single_servo()
            input("\nPress Enter to continue to Test 3...")
            
            self.test_coordinated_movement()
            input("\nPress Enter to continue to Test 4...")
            
            self.test_robot_sequence()
            
            print("\n" + "="*50)
            print("✓ ALL TESTS COMPLETED SUCCESSFULLY")
            print("="*50)
            
        except KeyboardInterrupt:
            print("\n\nTests interrupted by user")
        except Exception as e:
            print(f"\n✗ Test failed with error: {e}")
    
    def close(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("\n✓ Serial connection closed")


def main():
    # Check if custom port specified
    port = '/dev/serial0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    print("ESP32 Joint Controller Test")
    print(f"Using port: {port}")
    print("-" * 50)
    
    try:
        # Initialize controller
        controller = SimpleMotionController(port=port, baud=115200)
        
        # Run test suite
        controller.run_all_tests()
        
    except KeyboardInterrupt:
        print("\n\nStopping...")
    except Exception as e:
        print(f"\n✗ Error: {e}")
    finally:
        if 'controller' in locals():
            controller.close()


if __name__ == "__main__":
    main()