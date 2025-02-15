#!/usr/bin/env python3

""" 
Launch file for the main PPE GUI

Author: Max Chen
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the main PPE GUI
        Node(
            package='gui_package',
            executable='main_ppe_gui',
            name='main_ppe_gui',
            output='screen',
            emulate_tty=True,
        ),
    ]) 