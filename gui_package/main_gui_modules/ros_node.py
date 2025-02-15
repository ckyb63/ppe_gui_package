#!/usr/bin/env python3

"""
ROS2 Node implementation for PPE GUI

Author: Max Chen
"""
import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class for creating ROS nodes
from std_msgs.msg import String, Bool  # Import message types for String and Bool

class PPEGuiNode(Node):
    def __init__(self):
        # Initialize the ROS node with the name 'ppe_gui_node'
        super().__init__('ppe_gui_node')
        
        # Create a subscriber for PPE status messages
        self.subscription = self.create_subscription(
            String,
            'ppe_status',  # Topic name for PPE status
            self.ppe_status_callback,  # Callback method to handle incoming messages
            10)  # Queue size for incoming messages
        
        # Create a publisher for dispense requests
        self.dispense_publisher = self.create_publisher(
            String,
            'pleaseDispense',  # Topic name for dispense requests
            10)  # Queue size for outgoing messages
            
        # Create a publisher for gate status
        self.gate_publisher = self.create_publisher(
            Bool,
            'gate',  # Topic name for gate status
            10)  # Queue size for outgoing messages
        
        # Create a publisher for inventory status
        self.inventory_publisher = self.create_publisher(
            String,
            'ppeInventory',  # Topic name for inventory updates
            10)  # Queue size for outgoing messages
            
        # Create a subscriber for inventory status messages
        self.inventory_subscription = self.create_subscription(
            String,
            'ppeInventoryStatus',  # Topic name for inventory status
            self.inventory_status_callback,  # Callback method to handle incoming messages
            10)  # Queue size for incoming messages
        
        # Reference to GUI (will be set later)
        self.gui = None
        
        # Log a message indicating that the ROS node has been initialized
        self.get_logger().info('ROS Node initialized')

    def ppe_status_callback(self, msg):
        """Handle incoming PPE status messages."""
        if self.gui and not self.gui.is_shutting_down:  # Check if GUI is available and not shutting down
            # Emit a signal to update the GUI with the received PPE status
            self.gui.status_update_signal.emit(msg.data)
            # Log the received PPE status
            self.get_logger().info(f'Received PPE status: {msg.data}')

    def publish_dispense_request(self, ppe_name):
        """Publish a request to dispense a PPE item."""
        msg = String()  # Create a new String message
        msg.data = ppe_name  # Set the message data to the PPE item name
        self.dispense_publisher.publish(msg)  # Publish the dispense request
        # Log the published dispense request
        self.get_logger().info(f'Published dispense request for: {ppe_name}')

    def publish_gate_status(self, is_locked):
        """Publish the safety gate status."""
        msg = Bool()  # Create a new Bool message
        msg.data = is_locked  # Set the message data to the lock status
        self.gate_publisher.publish(msg)  # Publish the gate status
        # Log the published gate status
        self.get_logger().info(f'Published gate status: {"locked" if is_locked else "unlocked"}')

    def inventory_status_callback(self, msg):
        """Handle incoming inventory status messages."""
        if self.gui and not self.gui.is_shutting_down:  # Check if GUI is available and not shutting down
            # Emit a signal to update the GUI with the received inventory status
            self.gui.inventory_update_signal.emit(msg.data)
            
    def request_inventory_update(self):
        """Request an update for the inventory status."""
        msg = String()  # Create a new String message
        msg.data = "request"  # Set the message data to indicate a request
        self.inventory_publisher.publish(msg)  # Publish the inventory update request
        # Log the published inventory update request
        self.get_logger().info('Published inventory update request') 