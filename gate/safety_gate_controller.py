#!/usr/bin/env python3

"""
This is the main script for the safety gate controller.
It subscribes to the gate topic and sends commands to the ESP32 microcontroller to lock and unlock the gate.

Note: Should check the serial port for the ESP32 microcontroller connection. 

Author: Max Chen
Date: 2025-02-12
v0.1.2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial
import sys

# Initialize the serial connection
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# This is the main class for the safety gate controller.
class ESP32ControlNode(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('esp32_control_node')

        # Create subscription to the gate topic (changed to Bool type)
        self.subscription = self.create_subscription(
            Bool,
            'gate',  # Topic name matches the GUI publisher
            self.gate_callback,
            10
        )

        # Initialize the serial connection
        self.serial_connection()

    # This function initializes the serial connection to the ESP32 microcontroller
    def serial_connection(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info("Connected to ESP32 on %s" % SERIAL_PORT)
        except serial.SerialException as e:
            self.get_logger().error("Could not open serial port: %s" % e)
            sys.exit(1)

    def gate_callback(self, msg):
        """Handle gate status messages (Bool)
        True = locked, False = unlocked"""
        try:
            command = 'lock' if msg.data else 'unlock'
            self.get_logger().info(f"Received command: {command}")
            self.ser.write(command.encode('utf-8'))
            self.get_logger().info(f"Sent '{command}' to ESP32")
        except Exception as e:
            self.get_logger().error(f"Error handling gate command: {e}")

    def cleanup(self):
        """Cleanup function to close serial connection"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial connection closed.")

# This is the main function for the safety gate controller.
def main(args=None):
    rclpy.init(args=args)
    node = ESP32ControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup in proper order
        node.cleanup()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Ignore shutdown errors

# This is the main entry point for the safety gate controller.
if __name__ == '__main__':
    main()