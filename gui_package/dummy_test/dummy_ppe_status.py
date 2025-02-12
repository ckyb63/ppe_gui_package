#!/usr/bin/env python3
"""
Dummy PPE Status Publisher
------------------------
A test node that simulates PPE detection by publishing random PPE status messages.
Used for testing the PPE Vending Machine GUI without actual hardware.

Author: Max Chen
Version: 0.1.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time
import signal
from contextlib import contextmanager

@contextmanager
def ros_context():
    """Context manager for ROS initialization and shutdown"""
    try:
        rclpy.init()
        yield
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass

class DummyPPEPublisher(Node):
    def __init__(self):
        super().__init__('dummy_ppe_publisher')
        self.is_shutting_down = False
        
        # Create publisher
        self.publisher = self.create_publisher(
            String,
            'ppe_status',
            10)
            
        # Create timer for periodic publishing
        self.timer = self.create_timer(5.0, self.publish_ppe_status)
        
        self.get_logger().info('Dummy PPE Publisher started')

    def publish_ppe_status(self):
        """Generate and publish random PPE status"""
        if self.is_shutting_down:
            return
            
        # Generate random status for each PPE item
        ppe_items = ['hardhat', 'beardnet', 'gloves', 'glasses', 'earplugs']
        status = []
        
        for item in ppe_items:
            # 50% chance of each item being detected
            detected = random.choice([True, False])
            status.append(f"{item}:{detected}")
        
        # Create message
        msg = String()
        msg.data = ', '.join(status)
        
        # Publish message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published PPE status: {msg.data}')

def main(args=None):
    with ros_context():
        node = DummyPPEPublisher()
        
        def signal_handler(signum, frame):
            node.is_shutting_down = True
            node.destroy_node()
        
        # Set up signal handlers
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        try:
            while rclpy.ok() and not node.is_shutting_down:
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            if not node.is_shutting_down:
                node.destroy_node()

if __name__ == '__main__':
    main()