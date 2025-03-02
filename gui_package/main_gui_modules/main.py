#!/usr/bin/env python3

"""
Main entry point for the PPE Vending Machine GUI

Author: Max Chen
"""
import os  # Import the os module for operating system dependent functionality
import sys  # Import the sys module for system-specific parameters and functions
import threading  # Import the threading module for concurrent execution
import signal  # Import the signal module for handling asynchronous events
import logging  # Import logging for better error tracking
import rclpy  # Import the ROS 2 Python client library
from PyQt5.QtWidgets import QApplication  # Import QApplication for creating a GUI application

from .ros_node import PPEGuiNode  # Import the PPEGuiNode class for ROS communication
from .main_window import PPEVendingMachineGUI  # Import the main GUI class
from .utils.context import ros_context  # Import the ros_context for managing ROS lifecycle

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("ppe_gui")

def main():
    """Main function to initialize and run the PPE Vending Machine GUI."""
    logger.info("Starting PPE Vending Machine GUI")
    
    try:
        with ros_context():  # Use the context manager to handle ROS initialization and shutdown
            logger.info("ROS context initialized")
            ros_node = PPEGuiNode()  # Create an instance of the PPEGuiNode for ROS communication
            logger.info("ROS node created")
            
            # Set the XDG runtime directory for the application
            os.environ['XDG_RUNTIME_DIR'] = '/tmp/runtime-dir'
            app = QApplication(sys.argv)  # Initialize the Qt application with command line arguments
            
            # Create and show the main window of the GUI
            gui = PPEVendingMachineGUI(ros_node)  # Create an instance of the main GUI
            gui.show()  # Display the GUI window
            logger.info("GUI window displayed")
            
            ros_node.gui = gui  # Set the GUI reference in the ROS node for communication
            
            # Create a stop event for signaling the ROS thread to stop
            stop_event = threading.Event()
            
            def spin_ros():
                """Function to spin the ROS node in a separate thread."""
                logger.info("ROS spin thread started")
                try:
                    while not stop_event.is_set() and not gui.is_shutting_down:  # Continue spinning until signaled to stop
                        if rclpy.ok():  # Check if ROS is still running
                            rclpy.spin_once(ros_node, timeout_sec=0.1)  # Spin the ROS node with a timeout
                        else:
                            logger.warning("ROS context is no longer valid")
                            break  # Exit the loop if ROS is not okay
                except Exception as e:
                    if not gui.is_shutting_down:  # Log error if not shutting down
                        logger.error(f"ROS thread error: {e}", exc_info=True)
                logger.info("ROS spin thread stopped")
            
            # Create and start a separate thread for spinning the ROS node
            ros_thread = threading.Thread(target=spin_ros, daemon=True)  # Daemon thread will exit when the main program exits
            ros_thread.start()  # Start the ROS spinning thread
            
            def shutdown():
                """Function to handle the shutdown process."""
                logger.info("Shutting down application")
                if not gui.is_shutting_down:  # Check if the GUI is not already shutting down
                    gui.is_shutting_down = True  # Set the shutdown flag
                    gui.cleanup()  # Perform cleanup operations in the GUI
                    logger.info("GUI cleanup completed")
                
                # Signal the ROS thread to stop
                stop_event.set()
                
                if rclpy.ok():  # Check if ROS is still running
                    logger.info("Destroying ROS node")
                    ros_node.destroy_node()  # Destroy the ROS node
                
                # Wait for the ROS thread to finish (with timeout)
                if ros_thread.is_alive():
                    logger.info("Waiting for ROS thread to finish")
                    ros_thread.join(timeout=2.0)  # Wait with timeout to avoid hanging
            
            def signal_handler(signum, frame):
                """Handle termination signals to gracefully shut down the application."""
                logger.info(f"Received signal {signum}")
                shutdown()  # Call the shutdown function
                app.quit()  # Quit the Qt application
            
            # Set up signal handlers for graceful shutdown on SIGINT and SIGTERM
            signal.signal(signal.SIGINT, signal_handler)  # Handle Ctrl+C
            signal.signal(signal.SIGTERM, signal_handler)  # Handle termination signal
            
            try:
                logger.info("Starting Qt event loop")
                exit_code = app.exec_()  # Start the Qt event loop
                logger.info(f"Qt event loop ended with code {exit_code}")
            except KeyboardInterrupt:
                logger.info("Keyboard interrupt received")
                shutdown()  # Handle keyboard interrupt
                exit_code = 0  # Set exit code to 0 for normal exit
            except Exception as e:
                logger.error(f"Error in main loop: {e}", exc_info=True)  # Log any errors in the main loop
                shutdown()  # Call shutdown on error
                exit_code = 1  # Set exit code to 1 for error exit
            finally:
                shutdown()  # Ensure shutdown is called
                logger.info(f"Exiting with code {exit_code}")
                sys.exit(exit_code)  # Exit the application with the appropriate exit code
    except Exception as e:
        logger.critical(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)

if __name__ == '__main__':
    main()  # Execute the main function when the script is run 