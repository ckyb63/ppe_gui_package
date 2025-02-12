#!/usr/bin/env python3
"""
Dummy Inventory Publisher Node
----------------------------
Simulates a PPE inventory system by responding to inventory requests
with mock inventory data.

Author: Max Chen
v0.1.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random

class DummyInventoryPublisher(Node):
    def __init__(self):
        super().__init__('dummy_inventory_publisher')
        
        # Create subscriber for inventory requests
        self.subscription = self.create_subscription(
            String,
            'ppeInventory',
            self.handle_request,
            10)
            
        # Create publisher for inventory status
        self.publisher = self.create_publisher(
            String,
            'ppeInventoryStatus',
            10)
            
        # Initialize mock inventory
        self.inventory = {
            'hardhat': 50,
            'beardnet': 75,
            'gloves': 100,
            'glasses': 60,
            'earplugs': 200
        }
        
        # Create timer for random inventory changes
        self.create_timer(5.0, self.random_inventory_change)
        
        self.get_logger().info('Dummy Inventory Publisher initialized')
        
    def handle_request(self, msg):
        """Handle inventory request messages"""
        if msg.data == "request":
            self.publish_inventory()
            self.get_logger().info('Received inventory request')
            
    def publish_inventory(self):
        """Publish current inventory status"""
        msg = String()
        msg.data = json.dumps(self.inventory)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published inventory: {msg.data}')
        
    def random_inventory_change(self):
        """Randomly modify inventory levels to simulate usage"""
        # Randomly select an item
        item = random.choice(list(self.inventory.keys()))
        
        # Randomly increase or decrease by 1-5 units
        change = random.randint(-5, 5)
        
        # Apply change but keep inventory between 0 and 200
        self.inventory[item] = max(0, min(200, self.inventory[item] + change))
        
        # Log the change
        self.get_logger().info(f'Random inventory change: {item} {"+"+str(change) if change > 0 else change}')
        
def main(args=None):
    rclpy.init(args=args)
    
    publisher = DummyInventoryPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        # Ensure that shutdown is called only once
        if rclpy.ok():
            publisher.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main() 