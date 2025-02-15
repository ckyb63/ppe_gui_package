#!/usr/bin/env python3
"""
Dummy Inventory Publisher Node
----------------------------
Simulates a PPE inventory system by responding to inventory requests
with mock inventory data.

Author: Max Chen
v0.1.1
"""

import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class for creating ROS nodes
from std_msgs.msg import String  # Import the String message type
import json  # Import the JSON library for data serialization
import random  # Import the random library for generating random numbers

class DummyInventoryPublisher(Node):
    def __init__(self):
        # Initialize the ROS node with the name 'dummy_inventory_publisher'
        super().__init__('dummy_inventory_publisher')
        
        # Create a subscriber for inventory requests on the 'ppeInventory' topic
        self.subscription = self.create_subscription(
            String,
            'ppeInventory',
            self.handle_request,
            10)  # Queue size of 10 for incoming messages
            
        # Create a publisher for inventory status on the 'ppeInventoryStatus' topic
        self.publisher = self.create_publisher(
            String,
            'ppeInventoryStatus',
            10)  # Queue size of 10 for outgoing messages
            
        # Initialize a mock inventory with item quantities
        self.inventory = {
            'hardhat': 50,
            'beardnet': 75,
            'gloves': 100,
            'safetyglasses': 60,
            'earplugs': 200
        }
        
        # Create a timer that triggers the random_inventory_change method every 5 seconds
        self.create_timer(5.0, self.random_inventory_change)
        
        # Log a message indicating that the node has been initialized
        self.get_logger().info('Dummy Inventory Publisher initialized')
        
    def handle_request(self, msg):
        """Handle incoming inventory request messages."""
        if msg.data == "request":  # Check if the message data is a request
            self.publish_inventory()  # Call the method to publish the current inventory
            self.get_logger().info('Received inventory request')  # Log the request
            
    def publish_inventory(self):
        """Publish the current inventory status as a JSON string."""
        msg = String()  # Create a new String message
        msg.data = json.dumps(self.inventory)  # Serialize the inventory dictionary to JSON
        self.publisher.publish(msg)  # Publish the message to the 'ppeInventoryStatus' topic
        self.get_logger().info(f'Published inventory: {msg.data}')  # Log the published inventory
        
    def random_inventory_change(self):
        """Randomly modify inventory levels to simulate usage."""
        # Randomly select an item from the inventory
        item = random.choice(list(self.inventory.keys()))
        
        # Randomly determine a change in inventory (between -5 and +5 units)
        change = random.randint(-5, 5)
        
        # Apply the change while ensuring inventory levels remain between 0 and 200
        self.inventory[item] = max(0, min(200, self.inventory[item] + change))
        
        # Log the change made to the inventory
        self.get_logger().info(f'Random inventory change: {item} {"+"+str(change) if change > 0 else change}')
        
def main(args=None):
    """Main function to initialize and run the ROS node."""
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    
    publisher = DummyInventoryPublisher()  # Create an instance of the DummyInventoryPublisher class
    
    try:
        rclpy.spin(publisher)  # Keep the node running and processing callbacks
    except KeyboardInterrupt:
        publisher.get_logger().info('Keyboard interrupt received, shutting down...')  # Log shutdown message
    finally:
        # Ensure that shutdown is called only once
        if rclpy.ok():
            publisher.destroy_node()  # Clean up the node
            rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()  # Execute the main function when the script is run 