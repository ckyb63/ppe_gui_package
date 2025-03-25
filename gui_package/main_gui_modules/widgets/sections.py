"""
Main GUI sections for the PPE Vending Machine interface

Author: Max Chen
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
        header_layout.setContentsMargins(15, 15, 15, 15)
        
        # Help button
        help_button = QPushButton("?")
        help_button.setFont(QFont('Arial', 28, QFont.Bold))
        help_button.setFixedSize(70, 70)
        help_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.parent.colors.primary};
                color: white;
                border: none;
                border-radius: 20px;
                margin: 5px;
            }}
            QPushButton:hover {{
                background-color: {self.parent.colors.primary_dark};
            }}
        """)
        help_button.clicked.connect(self.parent.show_help)
        header_layout.addWidget(help_button)
        
        header_layout.addStretch(1)
        
        # Title
        title = QLabel('PPE Vending Machine')
        title.setFont(QFont('Arial', 24, QFont.Bold))
        header_layout.addWidget(title)
        
        header_layout.addStretch(1)
        
        # Settings button
        settings_button = QPushButton("⚙️")
        settings_button.setFont(QFont('Arial', 28, QFont.Bold))
        settings_button.setFixedSize(70, 70)
        settings_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.parent.colors.primary};
                color: white;
                border: none;
                border-radius: 20px;
                margin: 5px;
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
        # Main container with background
        main_container = QWidget()
        main_container.setObjectName("status_container")  # Add object name for styling
        main_container.setStyleSheet(f"""
            QWidget#status_container {{
                background-color: {self.parent.colors.surface};
                border-radius: 15px;
                padding: 10px;
            }}
            QLabel {{
                color: {self.parent.colors.text};
                background-color: transparent;
            }}
        """)
        
        status_layout = QVBoxLayout(main_container)
        status_layout.setSpacing(15)
        status_layout.setContentsMargins(20, 20, 20, 20)
        status_layout.setAlignment(Qt.AlignCenter)
        
        # Gate status
        self.gate_status = QLabel('Gate LOCKED')
        self.gate_status.setFont(QFont('Arial', 42, QFont.Bold))
        self.gate_status.setAlignment(Qt.AlignCenter)
        self.update_gate_status(True)  # Initially locked
        status_layout.addWidget(self.gate_status)
        
        # Add divider
        divider = QWidget()
        divider.setFixedHeight(1)
        divider.setStyleSheet(f"background-color: {self.parent.colors.text_secondary};")
        status_layout.addWidget(divider)
        
        # Status message
        self.status_label = QLabel('Ready to dispense...')
        self.status_label.setFont(QFont('Arial', 26, QFont.Bold))
        self.status_label.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.status_label)
        
        # Add the container to the main layout
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 10, 20, 10)
        main_layout.addWidget(main_container)
        
    def update_gate_status(self, is_locked):
        self.gate_status.setText('Gate LOCKED' if is_locked else 'Gate UNLOCKED')
        self.gate_status.setStyleSheet(f"""
            QLabel {{
                color: {'red' if is_locked else 'green'};
                font-weight: bold;
                padding: 10px;
                font-size: 42px;
            }}
        """)
        
    def show_status(self, message, color="black"):
        self.status_label.setText(message)
        self.status_label.setStyleSheet(f"""
            QLabel {{
                color: {color};
                font-weight: bold;
                font-size: 26px;
            }}
        """)

# Comment out entire CameraSection class
"""
class CameraSection(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self._init_ui()
        
        # ROS2 Subscriber for Camera Feed
        #self.create_camera_subscriber()

    def _init_ui(self):
        camera_layout = QVBoxLayout(self)
        
        # Camera feed placeholder
        self.camera_placeholder = QLabel('Camera Feed Placeholder')
        self.camera_placeholder.setFont(QFont('Arial', 14))
        self.update_styling()
        self.camera_placeholder.setAlignment(Qt.AlignCenter)
        camera_layout.addWidget(self.camera_placeholder)
        
    def create_camera_subscriber(self):
        #self.subscription = self.parent.create_subscription(
        #    Image,
        #    'camera_feed_topic',
        #    self.camera_feed_callback,
        #    10
        #)

    def camera_feed_callback(self):
        pass

    def update_styling(self):
        self.camera_placeholder.setStyleSheet(f'''
            QLabel {{
                border: 2px dashed {self.parent.colors.neutral};
                background: {self.parent.colors.surface};
                color: {self.parent.colors.text_secondary};
                min-height: 400px;
                padding: 10px;
            }}
        ''')
"""

class PPEGridSection(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.buttons = {}
        self._init_ui()
        
    def _init_ui(self):
        grid = QGridLayout(self)
        grid.setSpacing(15)  # Increased spacing between buttons
        
        ppe_items = [
            ('Hard Hat', 'hardhat', 0, 0),
            ('Beard Net', 'beardnet', 1, 0),
            ('Gloves', 'gloves', 2, 0),
            ('Safety Glasses', 'safetyglasses', 0, 1),
            ('Ear Plugs', 'earplugs', 1, 1),
            ('OVERRIDE', 'override', 2, 1)
        ]
        
        for label, key, row, col in ppe_items:
            container = QWidget()
            layout = QHBoxLayout(container)
            
            button = ColoredButton(label)
            button.setFont(QFont('Arial', 22, QFont.Bold))  # Increased from 18 to 22
            button.setMinimumHeight(150)  # Keep existing height
            button.setMinimumWidth(250)  # Keep existing width
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
                button.setStyleSheet(f"""
                    QPushButton {{
                        background-color: #ff9800;
                        color: white;
                        border: none;
                        border-radius: 5px;
                        font-weight: bold;
                        padding: 10px;
                    }}
                    QPushButton:hover {{
                        background-color: #f57c00;
                    }}
                    QPushButton:pressed {{
                        background-color: #ef6c00;
                    }}
                    QPushButton:disabled {{
                        background-color: {self.parent.colors.neutral};
                    }}
                """)
            else:
                status = ppe_status.get(key, False)
                button.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {self.parent.colors.success if status else self.parent.colors.danger};
                        color: white;
                        border: none;
                        border-radius: 5px;
                        font-weight: bold;
                        padding: 10px;
                    }}
                    QPushButton:hover {{
                        background-color: {self.parent.colors.success if status else '#ff5252'};
                        opacity: 0.9;
                    }}
                    QPushButton:pressed {{
                        background-color: {self.parent.colors.success if status else '#ff3838'};
                        opacity: 0.8;
                    }}
                    QPushButton:disabled {{
                        background-color: {self.parent.colors.neutral};
                        opacity: 0.7;
                    }}
                """) 