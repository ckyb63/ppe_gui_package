"""
Launch file for dummy nodes used in testing
Launches both the dummy PPE status publisher and dummy inventory publisher
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch dummy PPE status publisher
        Node(
            package='gui_package',
            executable='dummy_ppe',
            name='dummy_ppe_publisher',
            output='screen',
            emulate_tty=True,
        ),
        
        # Launch dummy inventory publisher
        Node(
            package='gui_package',
            executable='dummy_inventory',
            name='dummy_inventory_publisher',
            output='screen',
            emulate_tty=True,
        ),
    ]) 