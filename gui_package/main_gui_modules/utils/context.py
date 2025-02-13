#!/usr/bin/env python3

"""
ROS2 Context Management Utilities

This module provides a context manager for ROS2 initialization and shutdown.
It allows for easy initialization and shutdown of ROS2 nodes.

Author: Max Chen
v0.5.0
"""
import rclpy
from contextlib import contextmanager

@contextmanager
def ros_context():
    """Context manager for ROS initialization and shutdown"""
    try:
        rclpy.init()
        yield
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass 