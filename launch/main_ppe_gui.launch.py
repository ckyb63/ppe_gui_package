"""
Launch file for the main PPE GUI

Author: Max Chen
v0.5.1
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui_package',
            executable='main_ppe_gui',
            name='main_ppe_gui',
            output='screen',
            emulate_tty=True,
        ),
    ]) 