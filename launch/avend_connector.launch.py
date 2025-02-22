'''
Launch file for the Avend Connector Node

This launch file is used to start the Avend Connector Node with the configuration file.

Author: Max Chen

'''

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('ppe_gui_package')
    
    # Load the configuration file
    config_file = os.path.join(pkg_share, 'config', 'avend_config.yaml')
    
    # Create and return the launch description
    return LaunchDescription([
        Node(
            package='ppe_gui_package',
            executable='avend_connector',
            name='avend_connector',
            parameters=[config_file],
            output='screen'
        )
    ]) 