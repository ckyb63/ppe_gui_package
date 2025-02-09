"""
Color scheme management for the GUI
"""

class ColorScheme:
    def __init__(self, is_dark=False):
        self.is_dark = is_dark
        self.update_colors()
        
    def update_colors(self):
        if self.is_dark:
            self.background = "#1e1e1e"
            self.surface = "#2d2d2d"
            self.text = "#ffffff"
            self.text_secondary = "#aaaaaa"
            self.primary = "#007bff"
            self.primary_dark = "#0056b3"
            self.success = "#28a745"
            self.warning = "#ffc107"
            self.danger = "#dc3545"
            self.neutral = "#666666"
        else:
            self.background = "#ffffff"
            self.surface = "#f8f9fa"
            self.text = "#000000"
            self.text_secondary = "#666666"
            self.primary = "#007bff"
            self.primary_dark = "#0056b3"
            self.success = "#4caf50"
            self.warning = "#ff9800"
            self.danger = "#ff6b6b"
            self.neutral = "#666666" 