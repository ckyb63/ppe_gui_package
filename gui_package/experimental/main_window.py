"""
Main window implementation for the PPE Vending Machine GUI

Author: Max Chen
v0.5.0
"""
import os
import sys
import threading
import signal
import time
import json
from datetime import datetime
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                            QApplication, QLabel, QPushButton, QComboBox, 
                            QFormLayout, QTabWidget, QFrame, QTextEdit, QSpinBox, QDoubleSpinBox, QTableWidget, QTableWidgetItem, QHeaderView)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont

from .widgets.sections import TitleSection, StatusSection, CameraSection, PPEGridSection
from .utils.colors import ColorScheme
from .utils.logger import OverrideLogger

class PPEVendingMachineGUI(QMainWindow):
    # Add signals for thread-safe updates
    shutdown_signal = pyqtSignal()
    status_update_signal = pyqtSignal(str)
    gate_status_signal = pyqtSignal(bool)
    inventory_update_signal = pyqtSignal(str)
    
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.is_shutting_down = False
        
        # Initialize components
        self.override_logger = OverrideLogger()
        self.colors = ColorScheme()
        self.accessibility_mode = False
        
        # Initialize PPE status
        self.ppe_status = {
            'hardhat': False,
            'beardnet': False,
            'gloves': False,
            'glasses': False,
            'earplugs': False
        }
        self.safety_gate_locked = True
        
        # Cooldown tracking
        self.last_dispense_time = 0
        self.dispense_cooldown = 1.0
        self.current_ppe = None
        
        # Create main widget and layout
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QVBoxLayout(self.main_widget)
        
        # Initialize resize timer
        self.resize_timer = QTimer()
        self.resize_timer.setSingleShot(True)
        self.resize_timer.timeout.connect(self.handleResize)
        
        # Initialize UI elements
        self.initUI()
        
        # Initialize timers
        self.setupTimers()
        
        # Connect signals to slots
        self.status_update_signal.connect(self.handle_status_update)
        self.gate_status_signal.connect(self.handle_gate_status)
        self.inventory_update_signal.connect(self.handle_inventory_update)
        
        # Enable mouse tracking
        self.setMouseTracking(True)
        self.centralWidget().setMouseTracking(True)
        
        # Set window properties
        self.setWindowTitle('PPE Vending Machine')
        self.setMinimumSize(300, 240)
        self.setFixedSize(600, 900)
        self.center()
        
        # Initial layout update
        self.updateLayout()
        
        # Show the window
        self.show()

    def initUI(self):
        """Initialize UI elements"""
        # Create sections
        self.title_section = TitleSection(self)
        self.status_section = StatusSection(self)
        self.camera_section = CameraSection(self)
        self.ppe_grid = PPEGridSection(self)
        
        # Add to layout based on orientation
        self.updateLayout()

    def setupTimers(self):
        """Initialize all timers"""
        # Override reset timer
        self.override_timer = QTimer()
        self.override_timer.timeout.connect(self.reset_override)
        self.override_timer.setSingleShot(True)
        self.override_duration = 10.0
        
        # Override countdown timer
        self.countdown_timer = QTimer()
        self.countdown_timer.timeout.connect(self.update_countdown)
        self.countdown_timer.setInterval(1000)
        self.time_remaining = self.override_duration
        
        # Status update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status_displays)
        self.timer.start(100)
        
        # Status message timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.reset_status)
        self.status_timer.setSingleShot(True)
        
        # Dispense completion timer
        self.dispense_complete_timer = QTimer()
        self.dispense_complete_timer.setSingleShot(True)
        self.dispense_complete_timer.timeout.connect(self.show_dispense_complete)

    def center(self):
        """Center window on screen"""
        frame = self.frameGeometry()
        screen = QApplication.desktop().screenNumber(QApplication.desktop().cursor().pos())
        center_point = QApplication.desktop().screenGeometry(screen).center()
        frame.moveCenter(center_point)
        self.move(frame.topLeft())

    def resizeEvent(self, event):
        """Handle window resize events"""
        super().resizeEvent(event)
        self.resize_timer.start(100)

    def handleResize(self):
        """Update layout based on window size"""
        self.updateLayout()

    def updateLayout(self):
        """Update layout based on window size"""
        # Clear main layout while keeping widgets
        while self.main_layout.count():
            item = self.main_layout.takeAt(0)
            if item.widget():
                item.widget().hide()
        
        if self.width() > self.height():  # Landscape mode
            landscape = QWidget()
            landscape_layout = QHBoxLayout(landscape)
            
            # Left panel
            left_widget = QWidget()
            left_layout = QVBoxLayout(left_widget)
            
            self.title_section.show()
            self.status_section.show()
            left_layout.addWidget(self.title_section)
            left_layout.addWidget(self.status_section)
            
            # Add PPE grid
            self.ppe_grid.show()
            left_layout.addWidget(self.ppe_grid)
            
            # Right panel (camera)
            self.camera_section.show()
            landscape_layout.addWidget(left_widget, 1)
            landscape_layout.addWidget(self.camera_section, 2)
            
            self.main_layout.addWidget(landscape)
        else:  # Portrait mode
            portrait = QWidget()
            layout = QVBoxLayout(portrait)
            
            self.title_section.show()
            self.status_section.show()
            self.camera_section.show()
            self.ppe_grid.show()
            
            layout.addWidget(self.title_section)
            layout.addWidget(self.status_section)
            layout.addWidget(self.camera_section, 1)
            layout.addWidget(self.ppe_grid)
            
            self.main_layout.addWidget(portrait)

    def switchContent(self, content_type=None):
        """Switch main window content between main GUI, help, and settings"""
        # Clear the main layout first
        while self.main_layout.count():
            item = self.main_layout.takeAt(0)
            if item.widget():
                item.widget().hide()
        
        if content_type == 'help':
            help_content = self._create_help_content()
            help_content.setFixedSize(self.size())
            self.main_layout.addWidget(help_content)
            
        elif content_type == 'settings':
            settings_content = self._create_settings_content()
            settings_content.setFixedSize(self.size())
            self.main_layout.addWidget(settings_content)
            
        elif content_type == 'override':
            override_content = self._create_override_content()
            override_content.setFixedSize(self.size())
            self.main_layout.addWidget(override_content)
            
        else:  # Main GUI
            # Show main content widgets
            self.title_section.show()
            self.status_section.show()
            self.camera_section.show()
            self.ppe_grid.show()
            
            # Add widgets to layout based on orientation
            if self.width() > self.height():
                self._create_landscape_layout()
            else:
                self._create_portrait_layout()

    def _create_help_content(self):
        """Create help content widget"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(20)
        
        # Title section
        title_widget = QWidget()
        title_layout = QHBoxLayout(title_widget)
        title_layout.setAlignment(Qt.AlignCenter)
        
        help_icon = QLabel("?")
        help_icon.setFont(QFont('Arial', 48, QFont.Bold))
        help_icon.setStyleSheet(f"""
            QLabel {{
                color: {self.colors.primary};
                background-color: {self.colors.surface};
                border-radius: 40px;
                padding: 10px;
                min-width: 80px;
                min-height: 80px;
            }}
        """)
        help_icon.setAlignment(Qt.AlignCenter)
        
        title_text = QLabel("USER HELP GUIDE")
        title_text.setFont(QFont('Arial', 36, QFont.Bold))
        title_text.setStyleSheet(f"color: {self.colors.primary};")
        
        title_layout.addWidget(help_icon)
        title_layout.addWidget(title_text)
        layout.addWidget(title_widget)
        
        # Help content
        help_text = """
        <h2>PPE Detection:</h2>
        <ul style='font-size: 14pt;'>
        <li>The camera will detect the presence of required PPE</li>
        <li>Green buttons (O) indicate detected PPE</li>
        <li>Red buttons (X) indicate missing PPE</li>
        <li>The user may need to rotate head to fully detect all required PPE</li>
        </ul>

        <h2>Dispensing:</h2>
        <ul style='font-size: 14pt;'>
        <li>Click any PPE button to dispense that item</li>
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
                background: {self.colors.surface};
                color: {self.colors.text};
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
            "Accessibility O/X OFF" if not self.accessibility_mode else "Accessibility O/X ON"
        )
        self.help_toggle_button.setFont(QFont('Arial', 20, QFont.Bold))
        self.help_toggle_button.setFixedSize(400, 80)
        self._update_help_toggle_button_style()
        self.help_toggle_button.clicked.connect(self.toggle_accessibility_help)
        
        # OK button
        ok_button = QPushButton("OK")
        ok_button.setFont(QFont('Arial', 20, QFont.Bold))
        ok_button.setFixedSize(200, 80)
        ok_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.colors.primary};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.colors.primary_dark};
            }}
        """)
        ok_button.clicked.connect(lambda: self.switchContent())
        
        button_layout.addWidget(self.help_toggle_button)
        button_layout.addWidget(ok_button)
        layout.addWidget(button_container)
        
        return widget

    def _create_settings_content(self):
        """Create settings content widget"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(20)
        
        # Title section
        title_widget = QWidget()
        title_layout = QHBoxLayout(title_widget)
        title_layout.setAlignment(Qt.AlignCenter)
        
        settings_icon = QLabel("⚙️")
        settings_icon.setFont(QFont('Arial', 48, QFont.Bold))
        settings_icon.setStyleSheet(f"""
            QLabel {{
                color: {self.colors.primary};
                background-color: {self.colors.surface};
                border-radius: 40px;
                padding: 10px;
                min-width: 80px;
                min-height: 80px;
            }}
        """)
        settings_icon.setAlignment(Qt.AlignCenter)
        
        title_text = QLabel("SETTINGS")
        title_text.setFont(QFont('Arial', 36, QFont.Bold))
        title_text.setStyleSheet(f"color: {self.colors.primary};")
        
        title_layout.addWidget(settings_icon)
        title_layout.addWidget(title_text)
        layout.addWidget(title_widget)
        
        # Create tabs
        tabs = QTabWidget()
        tabs.setFont(QFont('Arial', 12))
        tabs.addTab(self._create_colors_tab(), "Appearance")
        tabs.addTab(self._create_inventory_tab(), "Inventory")
        tabs.addTab(self._create_timing_tab(), "Timing")
        tabs.addTab(self._create_override_log_tab(), "Override Log")
        
        layout.addWidget(tabs)
        
        # Button container
        button_container = QWidget()
        button_layout = QHBoxLayout(button_container)
        button_layout.setSpacing(40)
        
        # Cancel button
        cancel_button = QPushButton("Cancel")
        cancel_button.setFont(QFont('Arial', 20, QFont.Bold))
        cancel_button.setFixedSize(200, 80)
        cancel_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.colors.neutral};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: #555;
            }}
        """)
        cancel_button.clicked.connect(self._handle_settings_cancel)
        
        # Save button
        save_button = QPushButton("Save")
        save_button.setFont(QFont('Arial', 20, QFont.Bold))
        save_button.setFixedSize(200, 80)
        save_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.colors.primary};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.colors.primary_dark};
            }}
        """)
        save_button.clicked.connect(self._handle_settings_save)
        
        button_layout.addWidget(cancel_button)
        button_layout.addWidget(save_button)
        layout.addWidget(button_container)
        
        return widget

    def _create_colors_tab(self):
        """Create colors tab for settings"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setAlignment(Qt.AlignTop)
        layout.setSpacing(20)
        
        # Color scheme section
        scheme_label = QLabel("Color Scheme:")
        scheme_label.setFont(QFont('Arial', 16))
        layout.addWidget(scheme_label)
        
        self.color_scheme_combo = QComboBox()
        self.color_scheme_combo.setObjectName("color_scheme")
        self.color_scheme_combo.setFont(QFont('Arial', 14))
        self.color_scheme_combo.addItems(["Light", "Dark"])
        self.color_scheme_combo.setCurrentText("Dark" if self.colors.is_dark else "Light")
        self.color_scheme_combo.currentTextChanged.connect(self._apply_theme_preview)
        layout.addWidget(self.color_scheme_combo)
        
        # Add separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet(f"background-color: {self.colors.neutral};")
        layout.addWidget(separator)
        
        # Accessibility section
        access_label = QLabel("Accessibility:")
        access_label.setFont(QFont('Arial', 16))
        layout.addWidget(access_label)
        
        self.settings_toggle_button = QPushButton(
            "Accessibility O/X OFF" if not self.accessibility_mode else "Accessibility O/X ON"
        )
        self.settings_toggle_button.setFont(QFont('Arial', 14, QFont.Bold))
        self.settings_toggle_button.setFixedHeight(50)
        self._update_settings_toggle_button_style()
        self.settings_toggle_button.clicked.connect(self.toggle_accessibility_settings)
        layout.addWidget(self.settings_toggle_button)
        
        layout.addStretch()
        return widget

    def _create_timing_tab(self):
        """Create timing tab for settings"""
        widget = QWidget()
        layout = QFormLayout(widget)
        
        # Create spinboxes with labels
        override_label = QLabel("Override Duration (s):")
        override_label.setStyleSheet(f"color: {self.colors.text};")
        self.override_duration_spin = QSpinBox()
        self.override_duration_spin.setRange(5, 30)
        self.override_duration_spin.setValue(int(self.override_duration))
        
        cooldown_label = QLabel("Cooldown Time (s):")
        cooldown_label.setStyleSheet(f"color: {self.colors.text};")
        self.cooldown_time_spin = QDoubleSpinBox()
        self.cooldown_time_spin.setRange(0.5, 5.0)
        self.cooldown_time_spin.setValue(self.dispense_cooldown)
        
        layout.addRow(override_label, self.override_duration_spin)
        layout.addRow(cooldown_label, self.cooldown_time_spin)
        
        return widget

    def _create_inventory_tab(self):
        """Create inventory tab for settings"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Create table
        self.inventory_table = QTableWidget()
        self.inventory_table.setColumnCount(2)
        self.inventory_table.setHorizontalHeaderLabels(['PPE Item', 'Quantity'])
        
        # Set up table properties
        self.inventory_table.setStyleSheet(f"""
            QTableWidget {{
                background-color: {self.colors.surface};
                color: {self.colors.text};
                border: 1px solid {self.colors.neutral};
                border-radius: 5px;
            }}
            QHeaderView::section {{
                background-color: {self.colors.primary};
                color: white;
                padding: 5px;
                border: none;
            }}
        """)
        
        # Set column widths
        header = self.inventory_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)
        header.setSectionResizeMode(1, QHeaderView.Fixed)
        self.inventory_table.setColumnWidth(1, 100)
        
        # Add PPE items
        ppe_items = ['Hard Hat', 'Beard Net', 'Gloves', 'Safety Glasses', 'Ear Plugs']
        self.inventory_table.setRowCount(len(ppe_items))
        
        for i, item in enumerate(ppe_items):
            # Item name
            name_item = QTableWidgetItem(item)
            name_item.setFlags(name_item.flags() & ~Qt.ItemIsEditable)  # Make read-only
            self.inventory_table.setItem(i, 0, name_item)
            
            # Quantity (initially empty)
            qty_item = QTableWidgetItem('--')
            qty_item.setFlags(qty_item.flags() & ~Qt.ItemIsEditable)  # Make read-only
            qty_item.setTextAlignment(Qt.AlignCenter)
            self.inventory_table.setItem(i, 1, qty_item)
        
        layout.addWidget(self.inventory_table)
        
        # Request button
        request_button = QPushButton("Request Inventory Update")
        request_button.setFont(QFont('Arial', 14, QFont.Bold))
        request_button.setMinimumHeight(40)
        request_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.colors.primary};
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.colors.primary_dark};
            }}
        """)
        request_button.clicked.connect(self._request_inventory_update)
        layout.addWidget(request_button)
        
        # Last update time label
        self.last_update_label = QLabel("Last Update: Never")
        self.last_update_label.setAlignment(Qt.AlignCenter)
        self.last_update_label.setStyleSheet(f"color: {self.colors.text_secondary};")
        layout.addWidget(self.last_update_label)
        
        return widget

    def _create_override_log_tab(self):
        """Create override log tab for settings"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        log_text = QTextEdit()
        log_text.setReadOnly(True)
        log_text.setStyleSheet(f"""
            QTextEdit {{
                background-color: {self.colors.surface};
                color: {self.colors.text};
                border: 1px solid {self.colors.neutral};
                border-radius: 5px;
                padding: 10px;
                font-size: 12pt;
            }}
        """)
        
        recent_logs = self.override_logger.get_recent_logs()
        log_text.setText("\n".join([
            f"{log['timestamp']}: {log['reason']}"
            for log in recent_logs
        ]))
        
        layout.addWidget(log_text)
        return widget

    def _handle_settings_save(self):
        """Save settings and return to main view"""
        # Apply color scheme
        is_dark = self.color_scheme_combo.currentText() == "Dark"
        if is_dark != self.colors.is_dark:
            self.colors.is_dark = is_dark
            self.colors.update_colors()
            self.apply_theme()
        
        # Apply timing settings
        self.override_duration = float(self.override_duration_spin.value())
        self.dispense_cooldown = float(self.cooldown_time_spin.value())
        
        # Return to main view
        self.switchContent()

    def _handle_settings_cancel(self):
        """Cancel settings changes and return to main view"""
        self.switchContent()

    def _sync_accessibility_buttons(self):
        """Sync accessibility toggle buttons in help and settings"""
        button_text = "Accessibility O/X ON" if self.accessibility_mode else "Accessibility O/X OFF"
        
        # Update help toggle if it exists
        if hasattr(self, 'help_toggle_button'):
            self.help_toggle_button.setText(button_text)
            self._update_help_toggle_button_style()
            
        # Update settings toggle if it exists
        if hasattr(self, 'settings_toggle_button'):
            self.settings_toggle_button.setText(button_text)
            self._update_settings_toggle_button_style()
            
        # Update PPE status displays
        self.update_status_displays()

    def toggle_accessibility_help(self):
        """Toggle accessibility mode from help screen"""
        self.accessibility_mode = not self.accessibility_mode
        self._sync_accessibility_buttons()

    def toggle_accessibility_settings(self):
        """Toggle accessibility mode from settings screen"""
        self.accessibility_mode = not self.accessibility_mode
        self._sync_accessibility_buttons()

    def _update_settings_toggle_button_style(self):
        """Update settings toggle button styling"""
        self.settings_toggle_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.colors.success if self.accessibility_mode else self.colors.neutral};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.colors.success if self.accessibility_mode else '#555'};
            }}
        """)

    def _apply_theme_preview(self):
        """Apply theme changes in real-time"""
        is_dark = self.color_scheme_combo.currentText() == "Dark"
        if is_dark != self.colors.is_dark:
            self.colors.is_dark = is_dark
            self.colors.update_colors()
            self.apply_theme()

    def _create_override_content(self):
        """Create override confirmation content"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(20)
        
        # Warning section
        warning_widget = QWidget()
        warning_layout = QHBoxLayout(warning_widget)
        warning_layout.setAlignment(Qt.AlignCenter)
        
        warning_icon = QLabel("⚠️")
        warning_icon.setFont(QFont('Arial', 48))
        warning_icon.setStyleSheet(f"color: {self.colors.warning};")
        warning_layout.addWidget(warning_icon)
        
        warning_text = QLabel("WARNING")
        warning_text.setFont(QFont('Arial', 36, QFont.Bold))
        warning_text.setStyleSheet(f"color: {self.colors.warning};")
        warning_layout.addWidget(warning_text)
        
        layout.addWidget(warning_widget)
        
        # Message section
        message = QLabel("Are you sure you want to override\nthe safety system?")
        message.setFont(QFont('Arial', 24))
        message.setAlignment(Qt.AlignCenter)
        message.setWordWrap(True)
        message.setStyleSheet(f"color: {self.colors.text};")
        layout.addWidget(message)
        
        info_text = QLabel(f"This will unlock the safety gate for {int(self.override_duration)} seconds.")
        info_text.setFont(QFont('Arial', 18))
        info_text.setAlignment(Qt.AlignCenter)
        info_text.setWordWrap(True)
        info_text.setStyleSheet(f"color: {self.colors.text_secondary};")
        layout.addWidget(info_text)
        
        # Form section
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
            label.setStyleSheet(f"color: {self.colors.text};")
            
        layout.addWidget(form_widget)
        
        # Button section
        button_widget = QWidget()
        button_layout = QHBoxLayout(button_widget)
        button_layout.setSpacing(20)
        
        # No button
        no_button = QPushButton("NO")
        no_button.setFont(QFont('Arial', 20, QFont.Bold))
        no_button.setMinimumSize(200, 80)
        no_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.colors.neutral};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: #555;
            }}
        """)
        no_button.clicked.connect(lambda: self.switchContent())
        
        # Yes button
        yes_button = QPushButton("YES")
        yes_button.setFont(QFont('Arial', 20, QFont.Bold))
        yes_button.setMinimumSize(200, 80)
        yes_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.colors.warning};
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
        
        return widget

    def _style_combo_box(self, combo):
        """Style a combo box widget"""
        combo.setStyleSheet(f"""
            QComboBox {{
                background-color: {self.colors.surface};
                color: {self.colors.text};
                border: 1px solid {self.colors.neutral};
                border-radius: 5px;
                padding: 8px;
                min-width: 200px;
                min-height: 40px;
            }}
        """)

    def _handle_override_accept(self):
        """Handle override acceptance"""
        if self.user_combo.currentText() == "Select User":
            self.show_status("Please select a user", "red")
            return
        if self.reason_combo.currentText() == "Select Reason":
            self.show_status("Please select a reason", "red")
            return
            
        # Log the override
        self.override_logger.log_override(
            f"User: {self.user_combo.currentText()} - Reason: {self.reason_combo.currentText()}"
        )
        
        # Apply override
        for ppe in self.ppe_status:
            self.ppe_status[ppe] = True
        self.safety_gate_locked = False
        
        self.ros_node.publish_dispense_request('OVERRIDE')
        self.ros_node.publish_gate_status(self.safety_gate_locked)
        
        self.ppe_grid.buttons['override'].setEnabled(False)
        self.override_timer.start(int(self.override_duration * 1000))
        
        self.time_remaining = self.override_duration
        self.update_countdown()
        self.countdown_timer.start()
        
        # Switch back to main content
        self.switchContent()
        
        self.show_status(f"Override activated for {int(self.override_duration)}s!", "orange")

    def show_help(self):
        """Switch to help content"""
        self.switchContent('help')

    def show_settings(self):
        """Switch to settings content"""
        self.switchContent('settings')

    def on_ppe_button_click(self, ppe_name):
        """Handle PPE button clicks"""
        current_time = time.time()
        
        if ppe_name == 'override':
            # Switch to override content instead of showing dialog
            self.switchContent('override')
        else:
            if current_time - self.last_dispense_time < self.dispense_cooldown:
                remaining = round(self.dispense_cooldown - (current_time - self.last_dispense_time), 1)
                self.show_status(f"Please wait {remaining}s before next request", "red")
                return
            
            self.current_ppe = ppe_name
            self.show_status(f"Requesting {ppe_name.title()}...", "blue")
            
            self.ros_node.publish_dispense_request(ppe_name)
            self.ros_node.publish_gate_status(self.safety_gate_locked)
            
            self.last_dispense_time = current_time
            
            # Disable button during cooldown
            self.ppe_grid.buttons[ppe_name].setEnabled(False)
            QTimer.singleShot(int(self.dispense_cooldown * 1000), 
                            lambda: self.handle_dispense_complete(ppe_name))
        
        self.update_safety_gate()
        self.update_status_displays()

    def update_countdown(self):
        """Update the override countdown display"""
        self.time_remaining -= 1
        if self.time_remaining > 0:
            self.show_status(f"Override active: {int(self.time_remaining)}s", "orange")
        else:
            self.countdown_timer.stop()
            self.reset_override()

    def reset_override(self):
        """Reset system after override period"""
        for ppe in self.ppe_status:
            self.ppe_status[ppe] = False
        
        self.safety_gate_locked = True
        self.ros_node.publish_gate_status(self.safety_gate_locked)
        self.ppe_grid.buttons['override'].setEnabled(True)
        
        self.countdown_timer.stop()
        
        self.show_status("Override period ended", "blue")
        # Start auto-reset timer to return to ready state
        self.status_timer.start(3000)
        
        self.update_status_displays()

    def show_status(self, message, color="black"):
        """Show status message with color and auto-reset"""
        self.status_section.show_status(message, color)
        
        # Cancel any pending auto-reset
        self.status_timer.stop()
        
        # Only start auto-reset timer for regular status messages
        if ("Override" not in message and 
            "Requesting" not in message and
            "Dispensed" not in message):
            self.status_timer.start(1500)

    def reset_status(self):
        """Reset status message to default"""
        self.show_status('Ready to dispense...', self.colors.text)

    def update_safety_gate(self):
        """Update safety gate based on PPE detection"""
        previous_state = self.safety_gate_locked
        self.safety_gate_locked = not all(self.ppe_status.values())
        
        if previous_state != self.safety_gate_locked:
            self.gate_status_signal.emit(self.safety_gate_locked)
            self.status_section.update_gate_status(self.safety_gate_locked)

    def handle_status_update(self, status_str):
        """Handle PPE status updates in the Qt thread"""
        try:
            status_parts = status_str.split(',')
            for part in status_parts:
                name, value = part.strip().split(':')
                bool_value = value.strip().lower() == 'true'
                if name.strip() in self.ppe_status:
                    self.ppe_status[name.strip()] = bool_value
            
            self.update_safety_gate()
            self.update_status_displays()
            self.show_status("PPE status updated", "green")
        except Exception as e:
            self.ros_node.get_logger().error(f'Error parsing PPE status: {e}')
            self.show_status("Error updating PPE status", "red")

    def handle_gate_status(self, is_locked):
        """Handle gate status updates in the Qt thread"""
        if not self.is_shutting_down:
            self.ros_node.publish_gate_status(is_locked)

    def handle_dispense_complete(self, ppe_name):
        """Handle completion of dispense action"""
        # Re-enable button
        self.ppe_grid.buttons[ppe_name].setEnabled(True)
        
        # Show completion message
        self.show_status(f"{ppe_name.title()} Dispensed!", "green")
        
        # Start auto-reset timer
        self.status_timer.start(3000)

    def show_dispense_complete(self):
        """Show dispense completion message"""
        if hasattr(self, 'current_ppe'):
            self.show_status(f"{self.current_ppe.title()} Dispensed!", "green")
            # Start auto-reset timer after showing dispensed message
            self.status_timer.start(3000)

    def update_status_displays(self):
        """Update all status displays"""
        if self.is_shutting_down:
            return
            
        try:
            # Update PPE grid buttons
            self.ppe_grid.update_button_styles(self.ppe_status)
            
            # Update button text based on accessibility mode
            for key, button in self.ppe_grid.buttons.items():
                if key != 'override':
                    base_text = button.text().split('\n')[0]  # Get the original label
                    if self.accessibility_mode:
                        # Add O/X indicator
                        status = self.ppe_status.get(key, False)
                        button.setText(f"{base_text}\n{'O' if status else 'X'}")
                    else:
                        # Remove O/X indicator
                        button.setText(base_text)
            
            # Update gate status display
            self.status_section.update_gate_status(self.safety_gate_locked)
            
        except Exception as e:
            if not self.is_shutting_down:
                print(f"Error updating status displays: {e}")

    def apply_theme(self):
        """Apply current color scheme to all widgets"""
        # Apply to main window
        self.setStyleSheet(f"""
            QMainWindow {{
                background-color: {self.colors.background};
            }}
            QLabel {{
                color: {self.colors.text};
            }}
            QWidget {{
                background-color: {self.colors.background};
                color: {self.colors.text};
            }}
        """)
        
        # Update camera placeholder
        self.camera_section.update_styling()
        
        # Force update of all buttons
        self.update_status_displays()

    def cleanup(self):
        """Clean up timers from the Qt thread"""
        if not self.is_shutting_down:
            self.is_shutting_down = True
            self.timer.stop()
            self.override_timer.stop()
            self.countdown_timer.stop()
            self.status_timer.stop()
            self.dispense_complete_timer.stop()
            QApplication.processEvents()

    def closeEvent(self, event):
        """Handle window close event"""
        self.cleanup()
        event.accept()

    def _create_landscape_layout(self):
        """Create landscape layout"""
        landscape = QWidget()
        landscape_layout = QHBoxLayout(landscape)
        
        # Left panel
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        self.title_section.show()
        self.status_section.show()
        left_layout.addWidget(self.title_section)
        left_layout.addWidget(self.status_section)
        
        # Add PPE grid
        self.ppe_grid.show()
        left_layout.addWidget(self.ppe_grid)
        
        # Right panel (camera)
        self.camera_section.show()
        landscape_layout.addWidget(left_widget, 1)
        landscape_layout.addWidget(self.camera_section, 2)
        
        self.main_layout.addWidget(landscape)

    def _create_portrait_layout(self):
        """Create portrait layout"""
        portrait = QWidget()
        layout = QVBoxLayout(portrait)
        
        self.title_section.show()
        self.status_section.show()
        self.camera_section.show()
        self.ppe_grid.show()
        
        layout.addWidget(self.title_section)
        layout.addWidget(self.status_section)
        layout.addWidget(self.camera_section, 1)
        layout.addWidget(self.ppe_grid)
        
        self.main_layout.addWidget(portrait)

    def _update_help_toggle_button_style(self):
        """Update help toggle button styling"""
        self.help_toggle_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.colors.success if self.accessibility_mode else self.colors.neutral};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.colors.success if self.accessibility_mode else '#555'};
            }}
        """) 

    def _request_inventory_update(self):
        """Request inventory update from ROS node"""
        self.ros_node.request_inventory_update()
        self.last_update_label.setText("Last Update: Requesting...")
        
    def handle_inventory_update(self, data):
        """Handle inventory status updates"""
        try:
            inventory_data = json.loads(data)
            
            # Update table
            for row in range(self.inventory_table.rowCount()):
                item_name = self.inventory_table.item(row, 0).text()
                key = item_name.lower().replace(' ', '')
                if key in inventory_data:
                    qty_item = QTableWidgetItem(str(inventory_data[key]))
                    qty_item.setFlags(qty_item.flags() & ~Qt.ItemIsEditable)
                    qty_item.setTextAlignment(Qt.AlignCenter)
                    self.inventory_table.setItem(row, 1, qty_item)
            
            # Update timestamp
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.last_update_label.setText(f"Last Update: {current_time}")
            
        except Exception as e:
            self.show_status("Error updating inventory", "red")
            print(f"Error parsing inventory data: {e}")

    def _update_help_toggle_button_style(self):
        """Update help toggle button styling"""
        self.help_toggle_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.colors.success if self.accessibility_mode else self.colors.neutral};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.colors.success if self.accessibility_mode else '#555'};
            }}
        """) 