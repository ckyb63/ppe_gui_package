#!/usr/bin/env python3

"""
ROS2 Context Management Utilities

This module provides a context manager for ROS2 initialization and shutdown.
It allows for easy initialization and shutdown of ROS2 nodes.

Author: Max Chen
"""
import rclpy
import logging
from contextlib import contextmanager

# Get logger
logger = logging.getLogger("ppe_gui.ros_context")

@contextmanager
def ros_context(args=None):
    """
    Context manager for ROS initialization and shutdown
    
    Args:
        args: Command line arguments to pass to rclpy.init (optional)
    
    Yields:
        None: The context manager yields control back to the caller
    
    Raises:
        RuntimeError: If ROS initialization fails
    """
    initialized = False
    try:
        logger.info("Initializing ROS context")
        rclpy.init(args=args)
        initialized = True
        logger.info("ROS context initialized successfully")
        yield
    except Exception as e:
        logger.error(f"Error in ROS context: {e}", exc_info=True)
        raise
    finally:
        if initialized:
            try:
                logger.info("Shutting down ROS context")
                rclpy.shutdown()
                logger.info("ROS context shutdown complete")
            except Exception as e:
                logger.warning(f"Error during ROS shutdown: {e}", exc_info=True)