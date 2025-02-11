"""
Main GUI sections for the PPE Vending Machine interface

Author: Max Chen
v0.5.0
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                            QPushButton, QGridLayout)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

from .buttons import ColoredButton

class TitleSection(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self._init_ui()
        
    def _init_ui(self):
        header_layout = QHBoxLayout(self)
        
        # Help button container
        help_container = QWidget()
        help_container.setFixedSize(40, 40)
        help_layout = QHBoxLayout(help_container)
        help_layout.setContentsMargins(0, 0, 0, 0)
        
        # Help button
        help_button = QPushButton("?")
        help_button.setFont(QFont('Arial', 16, QFont.Bold))
        help_button.setFixedSize(40, 40)
        help_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.parent.colors.primary};
                color: white;
                border-radius: 20px;
                border: none;
            }}
            QPushButton:hover {{
                background-color: {self.parent.colors.primary_dark};
            }}
        """)
        help_button.clicked.connect(self.parent.show_help)
        help_layout.addWidget(help_button)
        
        header_layout.addWidget(help_container)
        header_layout.addStretch(1)
        
        # Title
        title = QLabel('PPE Vending Machine')
        title.setFont(QFont('Arial', 24, QFont.Bold))
        header_layout.addWidget(title)
        
        header_layout.addStretch(1)
        
        # Settings button
        settings_button = QPushButton("⚙️")
        settings_button.setFont(QFont('Arial', 16, QFont.Bold))
        settings_button.setFixedSize(40, 40)
        settings_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.parent.colors.primary};
                color: white;
                border-radius: 20px;
                border: none;
            }}
            QPushButton:hover {{
                background-color: {self.parent.colors.primary_dark};
            }}
        """)
        settings_button.clicked.connect(self.parent.show_settings)
        header_layout.addWidget(settings_button)

class StatusSection(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self._init_ui()
        
    def _init_ui(self):
        status_layout = QVBoxLayout(self)
        
        # Status line widget
        status_line = QWidget()
        status_layout_line = QHBoxLayout(status_line)
        
        # Gate status
        self.gate_status = QLabel('Gate LOCKED')
        self.gate_status.setFont(QFont('Arial', 20, QFont.Bold))
        self.update_gate_status(True)  # Initially locked
        status_layout_line.addWidget(self.gate_status)
        
        status_layout_line.addStretch(1)
        
        # Status message
        self.status_label = QLabel('Ready to dispense...')
        self.status_label.setFont(QFont('Arial', 18, QFont.Bold))
        self.status_label.setStyleSheet(f"color: {self.parent.colors.text};")
        status_layout_line.addWidget(self.status_label)
        
        status_layout.addWidget(status_line)
        
    def update_gate_status(self, is_locked):
        self.gate_status.setText('Gate LOCKED' if is_locked else 'Gate UNLOCKED')
        self.gate_status.setStyleSheet(f"""
            QLabel {{
                color: {'red' if is_locked else 'green'};
                font-weight: bold;
                padding: 5px;
                font-size: 20pt;
            }}
        """)
        
    def show_status(self, message, color="black"):
        self.status_label.setText(message)
        self.status_label.setStyleSheet(f"color: {color}; font-weight: bold;")

class CameraSection(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self._init_ui()
        
    def _init_ui(self):
        camera_layout = QVBoxLayout(self)
        
        # Camera feed placeholder
        self.camera_placeholder = QLabel('Camera Feed Placeholder')
        self.camera_placeholder.setFont(QFont('Arial', 14))
        self.update_styling()
        self.camera_placeholder.setAlignment(Qt.AlignCenter)
        camera_layout.addWidget(self.camera_placeholder)
        
    def update_styling(self):
        self.camera_placeholder.setStyleSheet(f"""
            QLabel {{
                border: 2px dashed {self.parent.colors.neutral};
                background: {self.parent.colors.surface};
                color: {self.parent.colors.text_secondary};
                min-height: 400px;
                padding: 10px;
            }}
        """)

class PPEGridSection(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.buttons = {}
        self._init_ui()
        
    def _init_ui(self):
        grid = QGridLayout(self)
        
        ppe_items = [
            ('Hard Hat', 'hardhat', 0, 0),
            ('Beard Net', 'beardnet', 0, 1),
            ('Gloves', 'gloves', 0, 2),
            ('Safety Glasses', 'glasses', 1, 0),
            ('Ear Plugs', 'earplugs', 1, 1),
            ('OVERRIDE', 'override', 1, 2)
        ]
        
        for label, key, row, col in ppe_items:
            container = QWidget()
            layout = QHBoxLayout(container)
            
            button = ColoredButton(label)
            button.setFont(QFont('Arial', 16, QFont.Bold))
            button.setMinimumHeight(80)
            button.clicked.connect(lambda checked, k=key: self.parent.on_ppe_button_click(k))
            
            layout.addWidget(button)
            grid.addWidget(container, row, col)
            self.buttons[key] = button
            
        self.update_button_styles()
    
    def update_button_styles(self, ppe_status=None):
        if ppe_status is None:
            ppe_status = {}
            
        for key, button in self.buttons.items():
            if key == 'override':
                button.setStyleSheet("""
                    QPushButton {
                        background-color: #ff9800;
                        color: white;
                        border: none;
                        border-radius: 5px;
                        font-weight: bold;
                        padding: 10px;
                    }
                    QPushButton:hover {
                        background-color: #f57c00;
                    }
                    QPushButton:pressed {
                        background-color: #ef6c00;
                    }
                    QPushButton:disabled {
                        background-color: #cccccc;
                    }
                """)
            else:
                status = ppe_status.get(key, False)
                button.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {'#4caf50' if status else '#ff6b6b'};
                        color: white;
                        border: none;
                        border-radius: 5px;
                        font-weight: bold;
                        padding: 10px;
                    }}
                    QPushButton:hover {{
                        background-color: {'#45a049' if status else '#ff5252'};
                    }}
                    QPushButton:pressed {{
                        background-color: {'#3d8b40' if status else '#ff3838'};
                    }}
                    QPushButton:disabled {{
                        background-color: #cccccc;
                    }}
                """) 