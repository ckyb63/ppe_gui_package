#!/usr/bin/env python3

"""
ROS2 Node for interfacing with Avend Local Vending API

This node handles:
1. Dispensing commands via local API calls
2. Session management
3. Multi-vend cart functionality
4. Health monitoring
5. Error handling and status updates

Author: Max Chen

Notes:
    export AVEND_HOST='your_host_ip'  # default: 127.0.0.1
    export AVEND_PORT='your_port'     # default: 8080
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import requests
import json
from datetime import datetime
import os
from urllib.parse import quote
from .logger import AvendLogger
import yaml
import time

class AvendConnector(Node):
    def __init__(self):
        super().__init__('avend_connector')
        
        # Load configuration
        self.load_configuration()
        
        # Initialize logger
        self.logger = AvendLogger(
            log_level=self.get_parameter('log_level').value,
            log_to_file=self.get_parameter('log_to_file').value,
            log_file_path=self.get_parameter('log_file_path').value
        )
        
        # Initialize API configuration
        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        self.api_base_url = f"http://{self.host}:{self.port}/avend"
        
        # Session management
        self.session_active = False
        self.session_timer = None
        
        # Cart management
        self.cart_items = []
        
        # Initialize subscribers
        self.dispense_sub = self.create_subscription(
            String,
            'pleaseDispense',
            self.handle_dispense_request,
            10
        )
        
        # Initialize publishers
        self.dispense_status_pub = self.create_publisher(
            Bool,
            'dispenseStatus',
            10
        )
        
        self.health_status_pub = self.create_publisher(
            String,
            'vendingHealth',
            10
        )
        
        # Initialize services
        self.clear_cart_srv = self.create_service(
            Trigger,
            'clear_cart',
            self.handle_clear_cart
        )

        # Initialize the inventory status publisher
        self.inventory_status_pub = self.create_publisher(
            String,
            'ppeInventoryStatus',
            10
        )
        
        # Create timers
        self.setup_timers()
        
        self.logger.info('Avend Connector Node initialized')

    def load_configuration(self):
        """Load configuration parameters"""
        self.declare_parameters(
            namespace='',
            parameters=[
                ('host', '127.0.0.1'),
                ('port', '8080'),
                ('request_timeout', 5.0),
                ('session_refresh_interval', 270.0),
                ('max_retries', 3),
                ('retry_delay', 1.0),
                ('slot_mapping', {}),
                ('health_check_interval', 60.0),
                ('log_level', 'info'),
                ('log_to_file', True),
                ('log_file_path', 'logs/avend_connector.log')
            ]
        )

    def setup_timers(self):
        """Initialize all timers"""
        # Session refresh timer
        self.session_refresh_timer = self.create_timer(
            self.get_parameter('session_refresh_interval').value,
            self.refresh_session
        )
        
        # Health check timer
        self.health_check_timer = self.create_timer(
            self.get_parameter('health_check_interval').value,
            self.check_health
        )

    def check_health(self):
        """Check vending machine health status"""
        try:
            response = requests.get(
                f"{self.api_base_url}?action=info",
                timeout=self.get_parameter('request_timeout').value
            )
            
            health_msg = String()
            if response.status_code == 200:
                health_msg.data = "healthy"
                self.logger.debug("Health check passed")
            else:
                health_msg.data = "unhealthy"
                self.logger.warning(f"Health check failed: {response.text}")
            
            self.health_status_pub.publish(health_msg)
            
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Health check failed: {str(e)}")
            health_msg = String()
            health_msg.data = "unreachable"
            self.health_status_pub.publish(health_msg)

    def make_api_request(self, endpoint, method='get', **kwargs):
        """Make API request with retry logic"""
        max_retries = self.get_parameter('max_retries').value
        retry_delay = self.get_parameter('retry_delay').value
        
        for attempt in range(max_retries):
            try:
                response = requests.request(
                    method,
                    endpoint,
                    timeout=self.get_parameter('request_timeout').value,
                    **kwargs
                )
                return response
            except requests.exceptions.RequestException as e:
                if attempt == max_retries - 1:
                    raise e
                self.logger.warning(f"Request failed, retrying... ({attempt + 1}/{max_retries})")
                time.sleep(retry_delay)

    def start_session(self):
        """Start a new vending session"""
        try:
            response = self.make_api_request(f"{self.api_base_url}?action=start")
            if response.status_code == 200:
                self.session_active = True
                self.logger.info('Session started successfully')
                return True
            else:
                self.logger.error(f'Failed to start session: {response.text}')
                return False
        except requests.exceptions.RequestException as e:
            self.logger.error(f'Session start request failed: {str(e)}')
            return False

    def handle_dispense_request(self, msg):
        """Handle incoming dispense requests"""
        try:
            ppe_item = msg.data.lower()
            slot_mapping = self.get_parameter('slot_mapping').value
            
            if ppe_item not in slot_mapping:
                self.logger.error(f'Invalid PPE item requested: {ppe_item}')
                self.publish_dispense_status(False)
                return
            
            # Add item to cart
            self.cart_items.append(slot_mapping[ppe_item])
            self.logger.info(f'Added {ppe_item} to cart')
            
            # Dispense items in cart
            success = self.dispense_cart()
            self.publish_dispense_status(success)
            
        except Exception as e:
            self.logger.error(f'Error handling dispense request: {str(e)}')
            self.publish_dispense_status(False)

    def dispense_cart(self):
        """Dispense all items in cart"""
        try:
            if not self.cart_items:
                self.logger.warning('Cart is empty')
                return False
            
            # Ensure active session
            if not self.session_active and not self.start_session():
                return False
            
            # Add all items to vending machine cart
            for slot_code in self.cart_items:
                encoded_code = quote(slot_code)
                response = self.make_api_request(
                    f"{self.api_base_url}?action=add&code={encoded_code}"
                )
                if not response or response.status_code != 200:
                    self.logger.error(f'Failed to add item {slot_code} to cart')
                    return False
            
            # Dispense all items
            response = self.make_api_request(f"{self.api_base_url}?action=dispense")
            success = response and response.status_code == 200
            
            if success:
                self.logger.info('Successfully dispensed all items')
                self.cart_items.clear()
            else:
                self.logger.error('Failed to dispense items')
            
            return success
            
        except Exception as e:
            self.logger.error(f'Error dispensing cart: {str(e)}')
            return False

    def handle_clear_cart(self, request, response):
        """Service handler for clearing the cart"""
        try:
            # Clear local cart
            self.cart_items.clear()
            
            # Clear vending machine cart
            api_response = self.make_api_request(f"{self.api_base_url}?action=clear")
            
            response.success = api_response and api_response.status_code == 200
            response.message = "Cart cleared successfully" if response.success else "Failed to clear cart"
            
            return response
            
        except Exception as e:
            self.logger.error(f'Error clearing cart: {str(e)}')
            response.success = False
            response.message = str(e)
            return response

    def refresh_session(self):
        """Refresh the session to prevent timeout"""
        if self.session_active:
            self.start_session()

    def publish_dispense_status(self, success):
        """Publish dispense status"""
        status_msg = Bool()
        status_msg.data = success
        self.dispense_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    avend_connector = AvendConnector()
    
    try:
        rclpy.spin(avend_connector)
    except KeyboardInterrupt:
        pass
    finally:
        avend_connector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
