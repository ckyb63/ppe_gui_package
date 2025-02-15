"""
Dialog windows for the PPE GUI

Author: Max Chen
"""
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                            QPushButton, QComboBox, QWidget, QTextEdit)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

class HelpDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.setWindowTitle("PPE Vending Machine Help")
        self.setModal(True)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowCloseButtonHint)
        self._init_ui()
        
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(20)
        
        # Add title section
        self._create_title_section(layout)
        
        # Add help content
        self._create_help_content(layout)
        
        # Add buttons
        self._create_button_section(layout)
        
        # Set dialog styling
        self._apply_styling()
        
        # Match parent window size
        if self.parent:
            self.setGeometry(self.parent.geometry())

    def _create_title_section(self, layout):
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

    def _create_help_content(self, layout):
        help_text = """
        <h2>PPE Detection:</h2>
        <ul style='font-size: 14pt;'>
        <li>The camera will detect the presence of required PPE</li>
        <li>Green buttons (O) indicate detected PPE</li>
        <li>Red buttons (X) indicate missing PPE</li>
        <li>The user may need to rotate head, look left and right, to fully detect all required PPE</li>
        </ul>

        <h2>Dispensing:</h2>
        <ul style='font-size: 14pt;'>
        <li>Click on the missing PPE button to dispense that item</li>
        <li>The safety gate remains locked until all required PPE is detected</li>
        </ul>

        <h2>Safety Override:</h2>
        <ul style='font-size: 14pt;'>
        <li>Orange OVERRIDE button for emergency or administrative override</li>
        <li>It will log the user and time of the override</li>
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

    def _create_button_section(self, layout):
        button_container = QWidget()
        button_layout = QHBoxLayout(button_container)
        button_layout.setSpacing(40)
        
        # Accessibility toggle
        self.toggle_button = QPushButton(
            "Accessibility O/X OFF" if not self.parent.accessibility_mode else "Accessibility O/X ON"
        )
        self.toggle_button.setFont(QFont('Arial', 20, QFont.Bold))
        self.toggle_button.setFixedSize(400, 80)
        self._update_toggle_button_style()
        self.toggle_button.clicked.connect(self.toggle_accessibility)
        
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
        ok_button.clicked.connect(self.accept)
        
        button_layout.addWidget(self.toggle_button)
        button_layout.addWidget(ok_button)
        layout.addWidget(button_container)

    def _update_toggle_button_style(self):
        self.toggle_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.parent.colors.success if self.parent.accessibility_mode else self.parent.colors.neutral};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.parent.colors.success if self.parent.accessibility_mode else '#555'};
            }}
        """)

    def _apply_styling(self):
        self.setStyleSheet(f"""
            QDialog {{
                background: {self.parent.colors.background};
            }}
            QLabel {{
                color: {self.parent.colors.text};
            }}
        """)

    def toggle_accessibility(self):
        """Toggle accessibility mode"""
        self.parent.accessibility_mode = not self.parent.accessibility_mode
        self.toggle_button.setText(
            "Accessibility O/X ON" if self.parent.accessibility_mode else "Accessibility O/X OFF"
        )
        self._update_toggle_button_style()
        self.parent.update_status_displays() 