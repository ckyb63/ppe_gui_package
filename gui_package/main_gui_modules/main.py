#!/usr/bin/env python3

"""
Main entry point for the PPE Vending Machine GUI

Author: Max Chen
"""
import os
import sys
import threading
import signal
import rclpy
from PyQt5.QtWidgets import QApplication

from .ros_node import PPEGuiNode
from .main_window import PPEVendingMachineGUI
from .utils.context import ros_context

def main():
    with ros_context():
        ros_node = PPEGuiNode()
        
        os.environ['XDG_RUNTIME_DIR'] = '/tmp/runtime-dir'
        app = QApplication(sys.argv)
        
        # Create and show the main window
        gui = PPEVendingMachineGUI(ros_node)
        gui.show()
        
        ros_node.gui = gui
        
        def spin_ros():
            try:
                while not gui.is_shutting_down:
                    if rclpy.ok():
                        rclpy.spin_once(ros_node, timeout_sec=0.1)
                    else:
                        break
            except Exception as e:
                if not gui.is_shutting_down:
                    print(f"ROS thread error: {e}")
        
        ros_thread = threading.Thread(target=spin_ros, daemon=True)
        ros_thread.start()
        
        def shutdown():
            if not gui.is_shutting_down:
                gui.is_shutting_down = True
                gui.cleanup()
                if rclpy.ok():
                    ros_node.destroy_node()
        
        def signal_handler(signum, frame):
            shutdown()
            app.quit()
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        try:
            exit_code = app.exec_()
        except KeyboardInterrupt:
            shutdown()
            exit_code = 0
        except Exception as e:
            print(f"Error in main loop: {e}")
            shutdown()
            exit_code = 1
        finally:
            shutdown()
            sys.exit(exit_code)

if __name__ == '__main__':
    main() 