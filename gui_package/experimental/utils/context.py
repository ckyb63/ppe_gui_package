"""
ROS2 Context Management Utilities
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