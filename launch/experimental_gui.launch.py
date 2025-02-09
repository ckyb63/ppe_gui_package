from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui_package',
            executable='experimental_gui',
            name='ppe_gui',
            output='screen',
            emulate_tty=True,
        ),
    ]) 