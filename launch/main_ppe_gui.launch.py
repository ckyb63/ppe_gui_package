"""
Launch file launches:
- Main PPE GUI
- Record dispense bag node
- Safety gate controller node

Includes coordinated shutdown handling for all nodes.

Author: Max Chen
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    # Define the nodes
    main_gui_node = Node(
        package='gui_package',
        executable='main_ppe_gui',
        name='main_ppe_gui',
        output='screen',
        emulate_tty=True,
    )

    record_node = Node(
        package='gui_package',
        executable='record_dispense_bag',
        name='record_dispense_bag',
        output='screen',
        emulate_tty=True,
    )

    safety_node = Node(
        package='gui_package',
        executable='safety_gate_controller',
        name='safety_gate_controller',
        output='screen',
        emulate_tty=True,
    )

    # Create event handler for shutdown
    main_gui_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=main_gui_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return LaunchDescription([
        main_gui_node,
        record_node,
        safety_node,
        main_gui_exit_handler
    ])