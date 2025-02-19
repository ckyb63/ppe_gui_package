#!/usr/bin/env python3

""" 
Launch file launches:
- Main PPE GUI
- Record dispense bag node
- Safety gate controller node

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

        # Launch the record dispense bag node
        Node(
            package='gui_package',
            executable='record_dispense_bag',
            name='record_dispense_bag',
            output='screen',
            emulate_tty=True,
        ),

        # Launch the safety gate controller node
        Node(
            package='gate',
            executable='safety_gate_controller',
            name='safety_gate_controller',
            output='screen',
            emulate_tty=True,
        ),
    ]) 