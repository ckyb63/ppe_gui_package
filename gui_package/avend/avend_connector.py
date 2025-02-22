#!/usr/bin/env python3

"""
ROS2 Node for interfacing with Avend Smart Vending API

This node handles:
1. Dispensing commands via API calls
2. Inventory status reporting
3. Error handling and status updates

Author: Max Chen


Notes: 
   export AVEND_API_KEY='your_api_key_here'
   export AVEND_MACHINE_ID='your_machine_id_here'
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import requests
import json
from datetime import datetime
import os

class AvendConnector(Node):
    def __init__(self):
        super().__init__('avend_connector')
        
        # Initialize API configuration
        self.api_base_url = "https://api.avendvending.com/v1"  # Replace with actual API URL
        self.api_key = os.getenv('AVEND_API_KEY', 'default_key')  # Get API key from environment variable
        self.machine_id = os.getenv('AVEND_MACHINE_ID', 'default_id')
        
        # Headers for API requests
        self.headers = {
            'Authorization': f'Bearer {self.api_key}',
            'Content-Type': 'application/json'
        }
        
        # Initialize subscribers
        self.dispense_sub = self.create_subscription(
            String,
            'pleaseDispense',
            self.handle_dispense_request,
            10
        )
        
        # Initialize publishers
        self.inventory_pub = self.create_publisher(
            String,
            'inventoryStatus',
            10
        )
        
        self.dispense_status_pub = self.create_publisher(
            Bool,
            'dispenseStatus',
            10
        )
        
        # Create timer for periodic inventory updates
        self.inventory_timer = self.create_timer(
            300.0,  # 5 minutes
            self.publish_inventory_status
        )
        
        self.get_logger().info('Avend Connector Node initialized')

    def handle_dispense_request(self, msg):
        """Handle incoming dispense requests"""
        try:
            # Parse the dispense request
            ppe_item = msg.data.lower()
            
            # Map PPE items to Avend slot numbers
            slot_mapping = {
                'hardhat': '1',
                'beardnet': '2',
                'gloves': '3',
                'safetyglasses': '4',
                'earplugs': '5'
            }
            
            if ppe_item not in slot_mapping:
                self.get_logger().error(f'Invalid PPE item requested: {ppe_item}')
                self.publish_dispense_status(False)
                return
            
            # Make API call to dispense item
            response = self.dispense_item(slot_mapping[ppe_item])
            
            # Handle the response
            if response.status_code == 200:
                self.get_logger().info(f'Successfully dispensed {ppe_item}')
                self.publish_dispense_status(True)
                # Trigger an immediate inventory update
                self.publish_inventory_status()
            else:
                self.get_logger().error(f'Failed to dispense {ppe_item}: {response.text}')
                self.publish_dispense_status(False)
                
        except Exception as e:
            self.get_logger().error(f'Error handling dispense request: {str(e)}')
            self.publish_dispense_status(False)

    def dispense_item(self, slot_number):
        """Make API call to dispense item"""
        endpoint = f"{self.api_base_url}/machines/{self.machine_id}/dispense"
        payload = {
            'slot': slot_number,
            'timestamp': datetime.utcnow().isoformat()
        }
        
        try:
            response = requests.post(
                endpoint,
                headers=self.headers,
                json=payload,
                timeout=5.0
            )
            return response
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'API request failed: {str(e)}')
            return None

    def get_inventory_status(self):
        """Get inventory status from Avend API"""
        endpoint = f"{self.api_base_url}/machines/{self.machine_id}/inventory"
        
        try:
            response = requests.get(
                endpoint,
                headers=self.headers,
                timeout=5.0
            )
            
            if response.status_code == 200:
                inventory_data = response.json()
                # Map API response to our inventory format
                formatted_inventory = {
                    'hardhat': str(inventory_data.get('1', '0')),
                    'beardnet': str(inventory_data.get('2', '0')),
                    'gloves': str(inventory_data.get('3', '0')),
                    'safetyglasses': str(inventory_data.get('4', '0')),
                    'earplugs': str(inventory_data.get('5', '0'))
                }
                return formatted_inventory
            else:
                self.get_logger().error(f'Failed to get inventory: {response.text}')
                return None
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Inventory API request failed: {str(e)}')
            return None

    def publish_inventory_status(self):
        """Publish current inventory status"""
        inventory = self.get_inventory_status()
        if inventory:
            # Convert inventory dict to JSON string
            inventory_msg = String()
            inventory_msg.data = json.dumps(inventory)
            self.inventory_pub.publish(inventory_msg)
            self.get_logger().debug('Published inventory status')
        else:
            self.get_logger().warn('Failed to publish inventory status')

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
