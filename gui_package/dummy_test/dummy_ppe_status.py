#!/usr/bin/env python3
"""
Dummy PPE Status Publisher Node
-------------------------------
A test node that simulates PPE detection by publishing random PPE status messages.
Used for testing the PPE Vending Machine GUI without actual hardware.

Author: Max Chen
Version: 0.1.1
"""

import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class for creating ROS nodes
from std_msgs.msg import String  # Import the String message type
import random  # Import the random library for generating random boolean values
import signal  # Import the signal library for handling termination signals
from contextlib import contextmanager  # Import contextmanager for context management

@contextmanager
def ros_context():
    """Context manager for initializing and shutting down the ROS 2 environment."""
    try:
        rclpy.init()  # Initialize the ROS 2 client library
        yield  # Yield control back to the calling context
    finally:
        try:
            rclpy.shutdown()  # Shutdown the ROS 2 client library
        except Exception:
            pass  # Ignore any exceptions during shutdown

class DummyPPEPublisher(Node):
    def __init__(self):
        # Initialize the ROS node with the name 'dummy_ppe_publisher'
        super().__init__('dummy_ppe_publisher')
        self.is_shutting_down = False  # Flag to indicate if the node is shutting down
        
        # Create a publisher for PPE status messages on the 'ppe_status' topic
        self.publisher = self.create_publisher(
            String,
            'ppe_status',
            10)  # Queue size of 10 for outgoing messages
            
        # Create a timer that triggers the publish_ppe_status method every 5 seconds
        self.timer = self.create_timer(5.0, self.publish_ppe_status)
        
        # Log a message indicating that the Dummy PPE Publisher has started
        self.get_logger().info('Dummy PPE Publisher started')

    def publish_ppe_status(self):
        """Generate and publish random PPE status messages."""
        if self.is_shutting_down:  # Check if the node is in the process of shutting down
            return  # Exit the method if shutting down
            
        # Define a list of PPE items to simulate detection
        ppe_items = ['hardhat', 'beardnet', 'gloves', 'glasses', 'earplugs']
        status = []  # Initialize a list to hold the status of each PPE item
        
        # Generate random status for each PPE item
        for item in ppe_items:
            detected = random.choice([True, False])  # Randomly determine if the item is detected
            status.append(f"{item}:{detected}")  # Append the status to the list
        
        # Create a message to publish
        msg = String()  # Create a new String message
        msg.data = ', '.join(status)  # Join the status list into a single string
        
        # Publish the message to the 'ppe_status' topic
        self.publisher.publish(msg)
        # Log the published PPE status
        self.get_logger().info(f'Published PPE status: {msg.data}')

def main(args=None):
    """Main function to initialize and run the ROS node."""
    with ros_context():  # Use the context manager for ROS initialization and shutdown
        node = DummyPPEPublisher()  # Create an instance of the DummyPPEPublisher class
        
        def signal_handler(signum, frame):
            """Handle termination signals to gracefully shut down the node."""
            node.is_shutting_down = True  # Set the shutdown flag
            node.destroy_node()  # Destroy the node
            
        # Set up signal handlers for graceful shutdown on SIGINT and SIGTERM
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        try:
            # Keep the node running and processing callbacks until shutting down
            while rclpy.ok() and not node.is_shutting_down:
                rclpy.spin_once(node, timeout_sec=0.1)  # Spin the node with a timeout
        except KeyboardInterrupt:
            pass  # Allow for graceful exit on keyboard interrupt
        finally:
            # Ensure that shutdown is called only once
            if not node.is_shutting_down:
                node.destroy_node()  # Clean up the node

if __name__ == '__main__':
    main()  # Execute the main function when the script is run