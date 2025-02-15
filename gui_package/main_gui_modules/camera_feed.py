#!/usr/bin/env python3  

"""
Camera feed module for the PPE Vending Machine GUI

Author: Max Chen
"""

import cv2  # Import OpenCV for computer vision tasks
from PyQt5.QtCore import QThread, pyqtSignal  # Import QThread and pyqtSignal for threading and signal handling

class CameraFeed(QThread):
    frame_captured = pyqtSignal(object)  # Signal to emit captured frames

    def __init__(self, camera_index=0):
        """Initialize the CameraFeed thread."""
        super().__init__()  # Call the parent class constructor
        self.camera_index = camera_index  # Set the camera index (default is 0)<<<<<<<<
        self.capture = None  # Initialize the capture object
        self.running = False  # Flag to control the running state of the thread

    def run(self):
        """Start the camera feed."""
        self.capture = cv2.VideoCapture(self.camera_index)  # Open the camera for capturing video
        self.running = True  # Set the running flag to True

        while self.running:  # Loop while the thread is running
            ret, frame = self.capture.read()  # Read a frame from the camera
            if ret:  # Check if the frame was captured successfully
                # Emit the captured frame using the signal
                self.frame_captured.emit(frame)

    def stop(self):
        """Stop the camera feed."""
        self.running = False  # Set the running flag to False to exit the loop
        if self.capture is not None:  # Check if the capture object is initialized
            self.capture.release()  # Release the camera resource

    def __del__(self):
        """Ensure the camera is released when the object is deleted."""
        self.stop()  # Call stop to release the camera if the object is deleted 