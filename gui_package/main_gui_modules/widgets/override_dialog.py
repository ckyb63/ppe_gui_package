"""
Override confirmation dialog

Author: Max Chen
v0.5.0
"""
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                            QPushButton, QComboBox, QWidget, QFormLayout)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

class OverrideDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.setWindowTitle("Confirm Override")
        self.setModal(True)
        self._init_ui()
        
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(20)
        
        # Warning section
        self._create_warning_section(layout)
        
        # Message section
        self._create_message_section(layout)
        
        # Form section
        self._create_form_section(layout)
        
        # Button section
        self._create_button_section(layout)
        
        # Set dialog styling
        self._apply_styling()
        
        # Set size and position
        self._set_geometry()
        
    def _create_warning_section(self, layout):
        warning_widget = QWidget()
        warning_layout = QHBoxLayout(warning_widget)
        warning_layout.setAlignment(Qt.AlignCenter)
        
        warning_icon = QLabel("⚠️")
        warning_icon.setFont(QFont('Arial', 48))
        warning_icon.setStyleSheet(f"color: {self.parent.colors.warning};")
        warning_layout.addWidget(warning_icon)
        
        warning_text = QLabel("WARNING")
        warning_text.setFont(QFont('Arial', 36, QFont.Bold))
        warning_text.setStyleSheet(f"color: {self.parent.colors.warning};")
        warning_layout.addWidget(warning_text)
        
        layout.addWidget(warning_widget)
        
    def _create_message_section(self, layout):
        message = QLabel("Are you sure you want to override\nthe safety system?")
        message.setFont(QFont('Arial', 24))
        message.setAlignment(Qt.AlignCenter)
        message.setWordWrap(True)
        message.setStyleSheet(f"color: {self.parent.colors.text};")
        layout.addWidget(message)
        
        info_text = QLabel("This will unlock the safety gate for 10 seconds.")
        info_text.setFont(QFont('Arial', 18))
        info_text.setAlignment(Qt.AlignCenter)
        info_text.setWordWrap(True)
        info_text.setStyleSheet(f"color: {self.parent.colors.text_secondary};")
        layout.addWidget(info_text)
        
    def _create_form_section(self, layout):
        form_widget = QWidget()
        form_layout = QFormLayout(form_widget)
        form_layout.setSpacing(15)
        
        # User dropdown
        self.user_combo = QComboBox()
        self.user_combo.setFont(QFont('Arial', 14))
        self.user_combo.addItems(["Select User", "Operator", "Supervisor", "Admin", "Maintenance"])
        self.user_combo.setCurrentText("Select User")
        self._style_combo_box(self.user_combo)
        
        # Reason dropdown
        self.reason_combo = QComboBox()
        self.reason_combo.setFont(QFont('Arial', 14))
        self.reason_combo.addItems([
            "Select Reason",
            "PPE Not Detected",
            "System Maintenance",
            "Emergency Override",
            "Admin Access",
            "Calibration Required"
        ])
        self.reason_combo.setCurrentText("Select Reason")
        self._style_combo_box(self.reason_combo)
        
        # Add to form
        form_layout.addRow("User:", self.user_combo)
        form_layout.addRow("Reason:", self.reason_combo)
        
        # Style form labels
        for label in form_widget.findChildren(QLabel):
            label.setFont(QFont('Arial', 14))
            label.setStyleSheet(f"color: {self.parent.colors.text};")
            
        layout.addWidget(form_widget)
        
    def _create_button_section(self, layout):
        button_widget = QWidget()
        button_layout = QHBoxLayout(button_widget)
        button_layout.setSpacing(20)
        
        # No button
        no_button = QPushButton("NO")
        no_button.setFont(QFont('Arial', 20, QFont.Bold))
        no_button.setMinimumSize(200, 80)
        no_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.parent.colors.neutral};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: #555;
            }}
        """)
        no_button.clicked.connect(self.reject)
        
        # Yes button
        yes_button = QPushButton("YES")
        yes_button.setFont(QFont('Arial', 20, QFont.Bold))
        yes_button.setMinimumSize(200, 80)
        yes_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.parent.colors.warning};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: #f57c00;
            }}
        """)
        yes_button.clicked.connect(self._handle_accept)
        
        button_layout.addWidget(no_button)
        button_layout.addWidget(yes_button)
        layout.addWidget(button_widget)
        
    def _style_combo_box(self, combo):
        combo.setStyleSheet(f"""
            QComboBox {{
                background-color: {self.parent.colors.surface};
                color: {self.parent.colors.text};
                border: 1px solid {self.parent.colors.neutral};
                border-radius: 5px;
                padding: 8px;
                min-width: 200px;
                min-height: 40px;
            }}
        """)
        
    def _apply_styling(self):
        self.setStyleSheet(f"""
            QDialog {{
                background: {self.parent.colors.background};
                min-width: 600px;
                min-height: 500px;
            }}
        """)
        
    def _set_geometry(self):
        parent_geometry = self.parent.geometry()
        dialog_width = 600
        dialog_height = 500
        
        x = parent_geometry.x() + (parent_geometry.width() - dialog_width) // 2
        y = parent_geometry.y() + (parent_geometry.height() - dialog_height) // 2
        
        self.setGeometry(x, y, dialog_width, dialog_height)
        
    def _handle_accept(self):
        """Validate and handle acceptance"""
        if self.user_combo.currentText() == "Select User":
            self.parent.show_status("Please select a user", "red")
            return
        if self.reason_combo.currentText() == "Select Reason":
            self.parent.show_status("Please select a reason", "red")
            return
            
        # Log the override
        self.parent.override_logger.log_override(
            f"User: {self.user_combo.currentText()} - Reason: {self.reason_combo.currentText()}"
        )
        self.accept() 