"""
Custom button widgets for the GUI

Author: Max Chen
v0.5.0
"""
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtCore import QVariantAnimation, pyqtProperty
from PyQt5.QtGui import QColor

class ColoredButton(QPushButton):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self._color = "#ff6b6b"  # Default red
        self._animation = QVariantAnimation()
        self._animation.setDuration(300)  # 300ms transition
        self._animation.valueChanged.connect(self._updateStyleSheet)
        
    @pyqtProperty(str)
    def color(self):
        return self._color
        
    @color.setter
    def color(self, color):
        if self._color != color:
            self._animation.stop()
            self._animation.setStartValue(self._color)
            self._animation.setEndValue(color)
            self._animation.start()
            self._color = color
            
    # Update the style sheet of the button
    def _updateStyleSheet(self, color):
        self.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
                color: white;
                border: none;
                border-radius: 5px;
                font-weight: bold;
            }}
            QPushButton:hover {{
                background-color: {self._darken(color)};
            }}
            QPushButton:pressed {{
                background-color: {self._darken(color, 0.2)};
            }}
        """)
    # Helper function to darken a color
    def _darken(self, color, factor=0.1):
        c = QColor(color)
        h, s, v, a = c.getHsv()
        return QColor.fromHsv(h, s, int(v * (1 - factor)), a).name() 