#!/usr/bin/env python3

"""
This node records the /pleaseDispense topic using rosbag2_py.

Author: Max Chen
Date: 2025-02-20
"""

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String
import rosbag2_py
from datetime import datetime
import os

class DispenseBagRecorder(Node):
    def __init__(self):
        super().__init__('dispense_bag_recorder')

        # Create a unique bag file name using the current timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_file_name = f"dispense_bag_{timestamp}.db3"  # Added .db3 extension
        recorded_bags_folder = os.path.join(os.getcwd(), "src", "ppe_gui_package", "gui_package", "main_gui_modules", "recordedBags")

        # Ensure the recorded_bags folder exists
        os.makedirs(recorded_bags_folder, exist_ok=True)

        # Set up storage options for the bag file
        storage_options = rosbag2_py.StorageOptions(
            uri=os.path.join(recorded_bags_folder, bag_file_name),  # Full path to the bag file
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer = rosbag2_py.SequentialWriter()
        
        try:
            self.writer.open(storage_options, converter_options)
            self.get_logger().info(f"Opened new bag file: {bag_file_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to open bag file: {e}")
            rclpy.shutdown()

        # Create topic metadata for the /pleaseDispense topic
        topic_info = rosbag2_py.TopicMetadata(
            name='/pleaseDispense',
            type='std_msgs/msg/String',
            serialization_format='cdr'
        )
        self.writer.create_topic(topic_info)

        # Create a subscription to the /pleaseDispense topic
        self.subscription = self.create_subscription(
            String,
            '/pleaseDispense',
            self.topic_callback,
            10
        )
        self.get_logger().info("Subscribed to /pleaseDispense topic.")

    def topic_callback(self, msg):
        """Callback function to write messages to the bag."""
        try:
            self.writer.write(
                '/pleaseDispense',
                serialize_message(msg),
                self.get_clock().now().nanoseconds
            )
            self.get_logger().info(f"Recorded message: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error recording message: {e}")

    def cleanup(self):
        """Cleanup function to finalize the bag file."""
        try:
            self.writer = None  # Let Python handle cleanup
            self.get_logger().info("Bag file closed successfully.")
        except Exception as e:
            self.get_logger().error(f"Error closing bag file: {e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = DispenseBagRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
    finally:
        node.cleanup()  # Ensure cleanup is called
        node.destroy_node()
        if rclpy.ok():  # Check if rclpy is still running
            rclpy.shutdown()

if __name__ == '__main__':
    main()