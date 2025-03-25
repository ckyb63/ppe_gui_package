#!/usr/bin/env python3

"""
ROS2 Node implementation for PPE GUI

This module provides the ROS2 node implementation for the PPE GUI application.
It handles communication with other ROS2 nodes in the system, including
subscribing to status messages and publishing dispense requests.

Author: Max Chen
"""
import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class for creating ROS nodes
from std_msgs.msg import String, Bool  # Import message types for String and Bool
import json
import os
import logging
from datetime import datetime

# Get logger
logger = logging.getLogger("ppe_gui.ros_node")

class PPEGuiNode(Node):
    """
    ROS2 Node for the PPE GUI application
    
    This class handles all ROS2 communication for the PPE GUI, including:
    - Subscribing to PPE status messages
    - Publishing dispense requests
    - Managing gate status
    - Handling inventory updates
    """
    
    def __init__(self):
        """Initialize the ROS node with publishers and subscribers"""
        # Initialize the ROS node with the name 'ppe_gui_node'
        super().__init__('ppe_gui_node')
        logger.info("Initializing PPE GUI ROS node")
        
        # Reference to the GUI (set later)
        self.gui = None
        
        # Create a subscriber for PPE status messages
        self.subscription = self.create_subscription(
            String,
            'ppe_status',  # Topic name for PPE status
            self.ppe_status_callback,  # Callback method to handle incoming messages
            10)  # Queue size for incoming messages
        logger.info("Created subscription to 'ppe_status' topic")
        
        # Create a publisher for dispense requests
        self.dispense_publisher = self.create_publisher(
            String,
            'pleaseDispense',  # Topic name for dispense requests
            10)  # Queue size for outgoing messages
        logger.info("Created publisher for 'pleaseDispense' topic")
            
        # Create a publisher for gate status
        self.gate_publisher = self.create_publisher(
            Bool,
            'gate',  # Topic name for gate status
            10)  # Queue size for outgoing messages
        logger.info("Created publisher for 'gate' topic")
        
        # Create a publisher for inventory status
        self.inventory_publisher = self.create_publisher(
            String,
            'ppeInventory',  # Topic name for inventory updates
            10)  # Queue size for outgoing messages
        logger.info("Created publisher for 'ppeInventory' topic")
            
        # Create a subscriber for inventory status messages
        self.inventory_subscription = self.create_subscription(
            String,
            'ppeInventoryStatus',  # Topic name for inventory status
            self.inventory_status_callback,  # Callback method to handle incoming messages
            10)  # Queue size for incoming messages
        logger.info("Created subscription to 'ppeInventoryStatus' topic")
        
        # Create a subscriber for camera feed
#        self.camera_subscription = self.create_subscription(
#            String,
#            'camera_feed',  # Topic name for camera feed
#            self.camera_feed_callback,  # Callback method to handle incoming messages
#            10)  # Queue size for incoming messages
#        logger.info("Created subscription to 'camera_feed' topic")
        
        logger.info("PPE GUI ROS node initialization complete")
    
    def ppe_status_callback(self, msg):
        """
        Callback for PPE status messages
        
        Args:
            msg (String): The status message from the PPE system
        """
        try:
            logger.debug(f"Received PPE status: {msg.data}")
            if self.gui:
                # Use the signal to update the GUI in a thread-safe way
                self.gui.status_update_signal.emit(msg.data)
            else:
                logger.warning("GUI reference not set, cannot update status")
        except Exception as e:
            logger.error(f"Error in ppe_status_callback: {e}", exc_info=True)
    
    def publish_dispense_request(self, ppe_name):
        """
        Publish a request to dispense a PPE item
        
        Args:
            ppe_name (str): The name of the PPE item to dispense
        """
        try:
            msg = String()
            msg.data = ppe_name
            logger.info(f"Publishing dispense request for: {ppe_name}")
            self.dispense_publisher.publish(msg)
            
            # Log the dispense event
            self.log_dispense_event(ppe_name)
        except Exception as e:
            logger.error(f"Error publishing dispense request: {e}", exc_info=True)
    
    def publish_gate_status(self, is_locked):
        """
        Publish the gate status (locked/unlocked)
        
        Args:
            is_locked (bool): True if the gate should be locked, False otherwise
        """
        try:
            msg = Bool()
            msg.data = is_locked
            logger.info(f"Publishing gate status: {'locked' if is_locked else 'unlocked'}")
            self.gate_publisher.publish(msg)
        except Exception as e:
            logger.error(f"Error publishing gate status: {e}", exc_info=True)
    
    def inventory_status_callback(self, msg):
        """
        Callback for inventory status messages
        
        Args:
            msg (String): The inventory status message
        """
        try:
            logger.debug(f"Received inventory status: {msg.data}")
            if self.gui:
                # Use the signal to update the GUI in a thread-safe way
                self.gui.inventory_update_signal.emit(msg.data)
            else:
                logger.warning("GUI reference not set, cannot update inventory")
        except Exception as e:
            logger.error(f"Error in inventory_status_callback: {e}", exc_info=True)
    
    def request_inventory_update(self):
        """Request an update of the inventory status"""
        try:
            msg = String()
            msg.data = "request_inventory_update"
            logger.info("Requesting inventory update")
            self.inventory_publisher.publish(msg)
        except Exception as e:
            logger.error(f"Error requesting inventory update: {e}", exc_info=True)
    
    def handle_dispense_complete(self, ppe_name):
        """
        Handle the completion of a dispense operation
        
        Args:
            ppe_name (str): The name of the PPE item that was dispensed
        """
        try:
            logger.info(f"Dispense complete for: {ppe_name}")
            
            # Update the inventory after dispensing
            self.request_inventory_update()
            
            # Log the dispense event
            self.log_dispense_event(ppe_name)
            
            if self.gui:
                # Show dispense complete message in the GUI
                self.gui.show_dispense_complete()
            else:
                logger.warning("GUI reference not set, cannot show dispense complete")
        except Exception as e:
            logger.error(f"Error handling dispense complete: {e}", exc_info=True)
    
    def log_dispense_event(self, ppe_name):
        """
        Log a dispense event to the dispense log
        
        Args:
            ppe_name (str): The name of the PPE item that was dispensed
        """
        try:
            # Get the current timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # Create the log entry
            log_entry = {
                "timestamp": timestamp,
                "item": ppe_name,
                "status": "dispensed"
            }
            
            logger.info(f"Logging dispense event: {log_entry}")
            
            # If the GUI is available, use its logging mechanism
            if self.gui:
                self.gui.log_dispense_event(ppe_name)
        except Exception as e:
            logger.error(f"Error logging dispense event: {e}", exc_info=True)
    
#    def camera_feed_callback(self, msg):
#        """
#        Callback for camera feed messages
#        
#        Args:
#            msg (String): The camera feed message
#        """
#        try:
#            logger.debug("Received camera feed update")
#            # Process camera feed data if needed
#            pass
#        except Exception as e:
#            logger.error(f"Error in camera_feed_callback: {e}", exc_info=True)