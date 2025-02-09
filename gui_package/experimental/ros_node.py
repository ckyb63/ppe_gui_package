"""
ROS2 Node implementation for PPE GUI
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class PPEGuiNode(Node):
    def __init__(self):
        super().__init__('ppe_gui_node')
        
        # Create subscriber for PPE status
        self.subscription = self.create_subscription(
            String,
            'ppe_status',
            self.ppe_status_callback,
            10)
        
        # Create publisher for dispense requests
        self.dispense_publisher = self.create_publisher(
            String,
            'pleaseDispense',
            10)
            
        # Create publisher for gate status
        self.gate_publisher = self.create_publisher(
            Bool,
            'gate',
            10)
        
        # Add inventory publisher and subscriber
        self.inventory_publisher = self.create_publisher(
            String,
            'ppeInventory',
            10)
            
        self.inventory_subscription = self.create_subscription(
            String,
            'ppeInventoryStatus',
            self.inventory_status_callback,
            10)
        
        # Reference to GUI (will be set later)
        self.gui = None
        
        self.get_logger().info('ROS Node initialized')

    def ppe_status_callback(self, msg):
        """Handle incoming PPE status messages"""
        if self.gui and not self.gui.is_shutting_down:
            # Use signal to update GUI
            self.gui.status_update_signal.emit(msg.data)
            self.get_logger().info(f'Received PPE status: {msg.data}')

    def publish_dispense_request(self, ppe_name):
        """Publish a request to dispense a PPE item"""
        msg = String()
        msg.data = ppe_name
        self.dispense_publisher.publish(msg)
        self.get_logger().info(f'Published dispense request for: {ppe_name}')

    def publish_gate_status(self, is_locked):
        """Publish safety gate status"""
        msg = Bool()
        msg.data = is_locked
        self.gate_publisher.publish(msg)
        self.get_logger().info(f'Published gate status: {"locked" if is_locked else "unlocked"}')

    def inventory_status_callback(self, msg):
        """Handle incoming inventory status messages"""
        if self.gui and not self.gui.is_shutting_down:
            self.gui.inventory_update_signal.emit(msg.data)
            
    def request_inventory_update(self):
        """Request inventory update"""
        msg = String()
        msg.data = "request"
        self.inventory_publisher.publish(msg)
        self.get_logger().info('Published inventory update request') 