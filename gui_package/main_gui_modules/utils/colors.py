#!/usr/bin/env python3
"""
Color scheme management for the GUI

This module provides a ColorScheme class that manages the color scheme of the GUI.
It allows for easy switching between light and dark modes.

The color scheme is defined in the update_colors method, which sets the colors for the
light and dark modes.

Author: Max Chen
"""

class ColorScheme:
    def __init__(self, is_dark=False):
        self.is_dark = is_dark
        self.update_colors()
        
    def update_colors(self):
        """Update the colors based on the current theme"""
        if self.is_dark:
            self.background = "#1e1e1e"  # Dark background
            self.surface = "#2d2d2d"     # Slightly lighter dark
            self.text = "#ffffff"        # White text
            self.text_secondary = "#aaaaaa"  # Light gray text
            self.primary = "#007bff"     # Blue
            self.primary_dark = "#0056b3" # Darker blue
            self.success = "#28a745"     # Green
            self.warning = "#ffc107"     # Yellow
            self.danger = "#dc3545"      # Red
            self.neutral = "#666666"     # Gray
        else:
            self.background = "#ffffff"  # White background
            self.surface = "#f8f9fa"     # Light gray surface
            self.text = "#000000"        # Black text
            self.text_secondary = "#666666"  # Dark gray text
            self.primary = "#007bff"     # Blue
            self.primary_dark = "#0056b3" # Darker blue
            self.success = "#4caf50"     # Green
            self.warning = "#ff9800"     # Orange
            self.danger = "#ff6b6b"      # Red
            self.neutral = "#666666"     # Gray 