"""
Help content widget

Author: Max Chen
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                            QPushButton)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

class HelpContent(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self._init_ui()
        
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(20)
        
        # Title section
        title_widget = QWidget()
        title_layout = QHBoxLayout(title_widget)
        title_layout.setAlignment(Qt.AlignCenter)
        
        help_icon = QLabel("?")
        help_icon.setFont(QFont('Arial', 48, QFont.Bold))
        help_icon.setStyleSheet(f"""
            QLabel {{
                color: {self.parent.colors.primary};
                background-color: {self.parent.colors.surface};
                border-radius: 40px;
                padding: 10px;
                min-width: 80px;
                min-height: 80px;
            }}
        """)
        help_icon.setAlignment(Qt.AlignCenter)
        
        title_text = QLabel("USER HELP GUIDE")
        title_text.setFont(QFont('Arial', 36, QFont.Bold))
        title_text.setStyleSheet(f"color: {self.parent.colors.primary};")
        
        title_layout.addWidget(help_icon)
        title_layout.addWidget(title_text)
        layout.addWidget(title_widget)
        
        # Help content
        help_text = """
        <h2>PPE Detection:</h2>
        <ul style='font-size: 14pt;'>
        <li>The AI detects required PPE using the camera.</li>
        <li>Rotate your head slightly to ensure all PPE is visible.</li>
        <li>Green buttons (O) indicate detected PPE; red buttons (X) indicate missing PPE.</li>
        </ul>

        <h2>Dispensing PPE:</h2>
        <ul style='font-size: 14pt;'>
        <li>Click on the PPE item to dispense if you do not have it.</li>
        <li>The safety gate will remain locked until all required PPE is detected.</li>
        </ul>

        <h2>Safety Override:</h2>
        <ul style='font-size: 14pt;'>
        <li>Use the orange OVERRIDE button for emergency or administrative overrides.</li>
        <li>All overrides are logged with the user, reason, and timestamp.</li>
        </ul>
        """
        text_label = QLabel(help_text)
        text_label.setTextFormat(Qt.RichText)
        text_label.setWordWrap(True)
        text_label.setStyleSheet(f"""
            QLabel {{
                background: {self.parent.colors.surface};
                color: {self.parent.colors.text};
                border-radius: 10px;
                padding: 20px;
                margin: 0 10px;
            }}
        """)
        layout.addWidget(text_label)
        
        # Button container
        button_container = QWidget()
        button_layout = QHBoxLayout(button_container)
        button_layout.setSpacing(40)
        
        # Accessibility toggle
        self.help_toggle_button = QPushButton(
            "Accessibility O/X OFF" if not self.parent.accessibility_mode else "Accessibility O/X ON"
        )
        self.help_toggle_button.setFont(QFont('Arial', 20, QFont.Bold))
        self.help_toggle_button.setFixedSize(300, 80)
        self._update_help_toggle_button_style()
        self.help_toggle_button.clicked.connect(self._toggle_accessibility)
        
        # OK button
        ok_button = QPushButton("OK")
        ok_button.setFont(QFont('Arial', 20, QFont.Bold))
        ok_button.setFixedSize(200, 80)
        ok_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.parent.colors.primary};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.parent.colors.primary_dark};
            }}
        """)
        ok_button.clicked.connect(lambda: self.parent.switchContent())
        
        button_layout.addWidget(self.help_toggle_button)
        button_layout.addWidget(ok_button)
        layout.addWidget(button_container)

    def _update_help_toggle_button_style(self):
        """Update help toggle button styling"""
        self.help_toggle_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.parent.colors.success if self.parent.accessibility_mode else self.parent.colors.neutral};
                color: white;
                border: none;
                border-radius: 10px;
                padding: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.parent.colors.success if self.parent.accessibility_mode else '#555'};
                opacity: 0.9;
            }}
            QPushButton:pressed {{
                opacity: 0.8;
            }}
        """)

    def update_toggle_button(self, text):
        """Update the accessibility toggle button text"""
        self.help_toggle_button.setText(text)
        self._update_help_toggle_button_style()

    def _toggle_accessibility(self):
        """Toggle accessibility mode and update button text"""
        # Toggle the mode through the parent
        self.parent.accessibility_mode = not self.parent.accessibility_mode
        # Update the accessibility handler
        self.parent.accessibility_handler.accessibility_mode = self.parent.accessibility_mode
        # Update button text
        self.help_toggle_button.setText(
            "Accessibility O/X ON" if self.parent.accessibility_mode else "Accessibility O/X OFF"
        )
        # Update button style
        self._update_help_toggle_button_style()
        # Sync all buttons
        self.parent._sync_accessibility_buttons()