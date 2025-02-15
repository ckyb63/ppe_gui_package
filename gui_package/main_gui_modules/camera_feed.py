#!/usr/bin/env python3  

"""
Camera feed module for the PPE Vending Machine GUI

Author: Max Chen
"""

import cv2
from PyQt5.QtCore import QThread, pyqtSignal

class CameraFeed(QThread):
    frame_captured = pyqtSignal(object)  # Signal to emit captured frames

    def __init__(self, camera_index=0):
        super().__init__()
        self.camera_index = camera_index
        self.capture = None
        self.running = False

    def run(self):
        """Start the camera feed."""
        self.capture = cv2.VideoCapture(self.camera_index)
        self.running = True

        while self.running:
            ret, frame = self.capture.read()
            if ret:
                # Emit the captured frame
                self.frame_captured.emit(frame)

    def stop(self):
        """Stop the camera feed."""
        self.running = False
        if self.capture is not None:
            self.capture.release()

    def __del__(self):
        """Ensure the camera is released when the object is deleted."""
        self.stop() 