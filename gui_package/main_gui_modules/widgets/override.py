"""
Override content widget

Author: Max Chen
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                            QPushButton, QComboBox, QFormLayout)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

class OverrideContent(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self._init_ui()
        
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(30)
        layout.setContentsMargins(20, 30, 20, 30)
        
        # Warning section
        warning_widget = QWidget()
        warning_layout = QHBoxLayout(warning_widget)
        warning_layout.setAlignment(Qt.AlignCenter)
        warning_layout.setSpacing(20)
        
        warning_icon = QLabel("⚠️")
        warning_icon.setFont(QFont('Arial', 56))
        warning_icon.setStyleSheet(f"color: {self.parent.colors.warning};")
        warning_layout.addWidget(warning_icon)
        
        warning_text = QLabel("WARNING")
        warning_text.setFont(QFont('Arial', 42, QFont.Bold))
        warning_text.setStyleSheet(f"color: {self.parent.colors.warning};")
        warning_layout.addWidget(warning_text)

        warning_icon2 = QLabel("⚠️")
        warning_icon2.setFont(QFont('Arial', 56))
        warning_icon2.setStyleSheet(f"color: {self.parent.colors.warning};")
        warning_layout.addWidget(warning_icon2)
        
        layout.addWidget(warning_widget)
        
        # Message section
        message = QLabel("Are you sure you want to override\nthe safety system?")
        message.setFont(QFont('Arial', 28))
        message.setAlignment(Qt.AlignCenter)
        message.setWordWrap(True)
        message.setStyleSheet(f"color: {self.parent.colors.text};")
        layout.addWidget(message)
        
        info_text = QLabel(f"This will unlock the safety gate for {int(self.parent.override_duration)} seconds.")
        info_text.setFont(QFont('Arial', 22))
        info_text.setAlignment(Qt.AlignCenter)
        info_text.setWordWrap(True)
        info_text.setStyleSheet(f"color: {self.parent.colors.text_secondary};")
        layout.addWidget(info_text)
        
        # Form section
        form_widget = QWidget()
        form_layout = QVBoxLayout(form_widget)
        form_layout.setSpacing(25)
        form_layout.setContentsMargins(20, 20, 20, 20)
        form_layout.setAlignment(Qt.AlignCenter)
        
        # User dropdown
        self.user_combo = QComboBox()
        self.user_combo.setFont(QFont('Arial', 20))
        self.user_combo.addItems(["Select User", "Operator", "Supervisor", "Admin", "Maintenance"])
        self.user_combo.setCurrentText("Select User")
        self._style_combo_box(self.user_combo)
        form_layout.addWidget(self.user_combo)
        
        # Reason dropdown
        self.reason_combo = QComboBox()
        self.reason_combo.setFont(QFont('Arial', 20))
        self.reason_combo.addItems([
            "Select Reason",
            "PPE Not Detected",
            "System Maintenance",
            "Emergency Override",
            "Admin Access",
            "Calibration Required",
            "Other"
        ])
        self.reason_combo.setCurrentText("Select Reason")
        self._style_combo_box(self.reason_combo)
        form_layout.addWidget(self.reason_combo)
            
        layout.addWidget(form_widget)
        
        # Button section
        button_widget = QWidget()
        button_layout = QHBoxLayout(button_widget)
        button_layout.setSpacing(30)
        
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
        no_button.clicked.connect(lambda: self.parent.switchContent())
        
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
        yes_button.clicked.connect(self._handle_override_accept)
        
        button_layout.addWidget(no_button)
        button_layout.addWidget(yes_button)
        layout.addWidget(button_widget)

    def _style_combo_box(self, combo):
        """Style a combo box widget"""
        combo.setStyleSheet(f"""
            QComboBox {{
                background-color: {self.parent.colors.surface};
                color: {self.parent.colors.text};
                border: 2px solid {self.parent.colors.neutral};
                border-radius: 8px;
                padding: 12px;
                min-width: 300px;
                min-height: 60px;
            }}
            QComboBox::drop-down {{
                border: none;
                width: 50px;
            }}
            QComboBox::down-arrow {{
                width: 30px;
                height: 30px;
            }}
            QComboBox QAbstractItemView {{
                background-color: {self.parent.colors.surface};
                color: {self.parent.colors.text};
                selection-background-color: {self.parent.colors.primary};
                selection-color: white;
                padding: 8px;
            }}
        """)

    def _handle_override_accept(self):
        """Handle override acceptance"""
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
        
        # Apply override
        for ppe in self.parent.ppe_status:
            self.parent.ppe_status[ppe] = True
        self.parent.safety_gate_locked = False
        
        self.parent.ros_node.publish_dispense_request('OVERRIDE')
        self.parent.ros_node.publish_gate_status(self.parent.safety_gate_locked)
        
        self.parent.ppe_grid.buttons['override'].setEnabled(False)
        self.parent.override_timer.start(int(self.parent.override_duration * 1000))
        
        self.parent.time_remaining = self.parent.override_duration
        self.parent.update_countdown()
        self.parent.countdown_timer.start()
        
        # Switch back to main content
        self.parent.switchContent()
        
        self.parent.show_status(f"Override activated for {int(self.parent.override_duration)}s!", "orange") 