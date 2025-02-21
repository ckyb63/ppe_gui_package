#!/usr/bin/env python3

"""
Main window implementation for the PPE Vending Machine GUI

Author: Max Chen
"""

import os
import time
import json
from datetime import datetime
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                            QApplication, QLabel, QPushButton, QComboBox, 
                            QFormLayout, QTabWidget, QFrame, QSpinBox, QDoubleSpinBox, 
                            QTableWidget, QTableWidgetItem, QHeaderView, QDialog, QTextEdit, QMessageBox)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QPalette, QColor
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from collections import Counter
import matplotlib.dates as mdates
import csv

from .widgets.sections import TitleSection, StatusSection, CameraSection, PPEGridSection
from .utils.colors import ColorScheme
from .utils.logger import OverrideLogger
from .jsonSupport.reporting import generate_report

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
        
        # Use the current working directory to construct the path
        self.inventory_log_dir = os.path.join(os.getcwd(), "src", "ppe_gui_package", "gui_package", "main_gui_modules", "jsonSupport")
        self.inventory_file = os.path.join(self.inventory_log_dir, "inventory_data.json")
        
        # Ensure the jsonSupport directory exists
        self._ensure_json_support_directory_exists()
        
        # Load saved inventory data and timestamp
        inventory_data = self._load_inventory_data()
        self.last_inventory = inventory_data['inventory']
        self.last_update_time = inventory_data['last_update']
        
        # Initialize PPE status
        self.ppe_status = {
            'hardhat': False,
            'beardnet': False,
            'gloves': False,
            'safetyglasses': False,
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
        self.setFixedSize(600, 950)
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
        self.help_toggle_button.setFixedSize(300, 80)
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
        
        # Create tabs with custom styling
        self.tabs = QTabWidget()  # Define tabs as an instance variable
        self.tabs.setFont(QFont('Arial', 18))
        
        # Custom tab bar
        tab_bar = self.tabs.tabBar()
        tab_bar.setStyleSheet("""
            QTabBar::tab {
                background: #007bff;  /* Default color */
                color: white;
                padding: 10px;
                border: 1px solid #ccc;
                border-bottom: none;
                min-width: 75px;
            }
            QTabBar::tab:selected {
                background: #0056b3;  /* Selected color */
            }
            QTabBar::tab:!selected {
                background: #007bff;  /* Unselected color */
            }
        """)
        
        # Add numbered tabs with titles
        self.tabs.addTab(self._create_info_tab(), "Info.") # Info
        self.tabs.addTab(self._create_colors_tab(), "1.") # Appearance
        self.tabs.addTab(self._create_inventory_tab(), "2.") # Inventory
        self.tabs.addTab(self._create_timing_tab(), "3.") # Timing
        self.tabs.addTab(self._create_override_log_tab(), "4.") # Override Log
        self.tabs.addTab(self._create_report_tab(), "5.") # Report
        
        layout.addWidget(self.tabs)
        
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

        # Create tab title
        colors_label = QLabel("Appearance")
        colors_label.setFont(QFont('Arial', 24, QFont.Bold))
        layout.addWidget(colors_label)  # Add title to the layout
        
        # Color scheme section
        scheme_label = QLabel("Color Scheme:")
        scheme_label.setFont(QFont('Arial', 18))
        layout.addWidget(scheme_label)
        
        self.color_scheme_combo = QComboBox()
        self.color_scheme_combo.setObjectName("color_scheme")
        self.color_scheme_combo.setFont(QFont('Arial', 20))  # Increase font size for better visibility
        self.color_scheme_combo.setMinimumHeight(70)  # Set minimum height for touch friendliness
        self.color_scheme_combo.setFixedHeight(50)  # Set fixed height for consistency
        self.color_scheme_combo.setStyleSheet("""
            QComboBox {
                padding: 10px;  /* Add padding for better touch area */
                min-width: 200px; /* Set minimum width */
                font-size: 24px; /* Increase font size */
            }
        """)
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
        access_label.setFont(QFont('Arial', 18))
        layout.addWidget(access_label)
        
        self.settings_toggle_button = QPushButton(
            "Accessibility O/X OFF" if not self.accessibility_mode else "Accessibility O/X ON"
        )
        self.settings_toggle_button.setFont(QFont('Arial', 24, QFont.Bold))
        self.settings_toggle_button.setFixedHeight(90)  # Set fixed height for button
        self._update_settings_toggle_button_style()
        self.settings_toggle_button.clicked.connect(self.toggle_accessibility_settings)
        layout.addWidget(self.settings_toggle_button)
        
        layout.addStretch()
        return widget

    def _create_timing_tab(self):
        """Create timing tab for settings"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(40)
        layout.setContentsMargins(30, 30, 30, 30)

        # Create tab title
        timing_label = QLabel("Timing")
        timing_label.setFont(QFont('Arial', 24, QFont.Bold))
        layout.addWidget(timing_label)  # Add title to the layout 
        
        # Override Duration Section
        override_section = QWidget()
        override_layout = QVBoxLayout(override_section)
        override_layout.setSpacing(10)
        
        override_title = QLabel("Override Duration (s)")
        override_title.setFont(QFont('Arial', 24, QFont.Bold))
        override_title.setPalette(self._create_palette(self.colors.text))
        override_title.setWordWrap(True)
        
        self.override_duration_spin = QSpinBox()
        self.override_duration_spin.setFont(QFont('Arial', 20, QFont.Bold))
        self.override_duration_spin.setMinimumHeight(100)
        self.override_duration_spin.setMinimumWidth(250)
        self.override_duration_spin.setRange(5, 30)
        self.override_duration_spin.setValue(int(self.override_duration))
        self._style_spinbox(self.override_duration_spin)
        
        # Set stylesheet for the increment buttons
        self.override_duration_spin.setStyleSheet("""
            QSpinBox {
                padding: 10px;  /* Add padding to the spin box */
            }
            QAbstractSpinBox::up-button {
                width: 50px;  /* Set the width of the up button */
                height: 50px; /* Set the height of the up button */
                background-color: #28a745; /* Green color for the up button */
                color: white; /* Button text color */
                border: none; /* Remove border */
                border-radius: 5px; /* Rounded corners */
            }
            QAbstractSpinBox::down-button {
                width: 50px;  /* Set the width of the down button */
                height: 50px; /* Set the height of the down button */
                background-color: #dc3545; /* Red color for the down button */
                color: white; /* Button text color */
                border: none; /* Remove border */
                border-radius: 5px; /* Rounded corners */
            }
            QAbstractSpinBox::up-button:hover {
                background-color: #218838; /* Darker green on hover */
            }
            QAbstractSpinBox::down-button:hover {
                background-color: #c82333; /* Darker red on hover */
            }
        """)
        
        override_layout.addWidget(override_title)
        override_layout.addWidget(self.override_duration_spin)
        
        # Cooldown Time Section
        cooldown_section = QWidget()
        cooldown_layout = QVBoxLayout(cooldown_section)
        cooldown_layout.setSpacing(10)
        
        cooldown_title = QLabel("PPE Request Cooldown Time (s)")
        cooldown_title.setFont(QFont('Arial', 24, QFont.Bold))
        cooldown_title.setPalette(self._create_palette(self.colors.text))
        cooldown_title.setWordWrap(True)
        
        self.cooldown_time_spin = QDoubleSpinBox()
        self.cooldown_time_spin.setFont(QFont('Arial', 20, QFont.Bold))
        self.cooldown_time_spin.setMinimumHeight(100)
        self.cooldown_time_spin.setMinimumWidth(250)
        self.cooldown_time_spin.setRange(0.5, 5.0)
        self.cooldown_time_spin.setSingleStep(0.5)
        self.cooldown_time_spin.setValue(self.dispense_cooldown)
        self._style_spinbox(self.cooldown_time_spin)
        
        # Set stylesheet for the increment buttons
        self.cooldown_time_spin.setStyleSheet("""
            QDoubleSpinBox {
                padding: 10px;  /* Add padding to the spin box */
            }
            QAbstractSpinBox::up-button {
                width: 50px;  /* Set the width of the up button */
                height: 50px; /* Set the height of the up button */
                background-color: #28a745; /* Green color for the up button */
                color: white; /* Button text color */
                border: none; /* Remove border */
                border-radius: 5px; /* Rounded corners */
            }
            QAbstractSpinBox::down-button {
                width: 50px;  /* Set the width of the down button */
                height: 50px; /* Set the height of the down button */
                background-color: #dc3545; /* Red color for the down button */
                color: white; /* Button text color */
                border: none; /* Remove border */
                border-radius: 5px; /* Rounded corners */
            }
            QAbstractSpinBox::up-button:hover {
                background-color: #218838; /* Darker green on hover */
            }
            QAbstractSpinBox::down-button:hover {
                background-color: #c82333; /* Darker red on hover */
            }
        """)
        
        cooldown_layout.addWidget(cooldown_title)
        cooldown_layout.addWidget(self.cooldown_time_spin)

        # Add sections to main layout
        layout.addWidget(override_section)
        layout.addSpacing(30)  # Space between sections
        layout.addWidget(cooldown_section)
        layout.addStretch()  # Push everything to the top
        
        return widget

    def _style_spinbox(self, spinbox):
        """Apply common styling to spinbox widgets"""
        # Set colors using palette
        palette = QPalette()
        palette.setColor(QPalette.Text, QColor(self.colors.text))
        palette.setColor(QPalette.Base, QColor(self.colors.surface))
        palette.setColor(QPalette.Button, QColor(self.colors.surface))
        spinbox.setPalette(palette)
        
        # Set frame and alignment
        spinbox.setFrame(True)
        spinbox.setAlignment(Qt.AlignCenter)
        
        # Style the buttons
        spinbox.setButtonSymbols(QSpinBox.UpDownArrows)
        
        # Make buttons more touch-friendly
        if hasattr(spinbox, 'findChild'):
            up_button = spinbox.findChild(QWidget, 'qt_spinbox_upbutton')
            down_button = spinbox.findChild(QWidget, 'qt_spinbox_downbutton')
            if up_button and down_button:
                up_button.setMinimumSize(50, 50)
                down_button.setMinimumSize(50, 50)

    def _create_palette(self, color):
        """Create a palette with the specified text color"""
        palette = QPalette()
        palette.setColor(QPalette.WindowText, QColor(color))
        return palette

    def _create_inventory_tab(self):
        """Create inventory tab for settings"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(20)

        # Create tab title
        inventory_label = QLabel("Inventory")
        inventory_label.setFont(QFont('Arial', 24, QFont.Bold))
        layout.addWidget(inventory_label)
        
        # Create table
        self.inventory_table = QTableWidget()
        self.inventory_table.setColumnCount(2)
        self.inventory_table.setHorizontalHeaderLabels(['PPE Item', 'Quantity'])
        
        # Set up table properties
        self.inventory_table.setFont(QFont('Arial', 16))
        self.inventory_table.verticalHeader().setDefaultSectionSize(50)
        self.inventory_table.horizontalHeader().setMinimumHeight(60)
        self.inventory_table.setMinimumHeight(400)
        self.inventory_table.setStyleSheet(f"""
            QTableWidget {{
                background-color: {self.colors.surface};
                color: {self.colors.text};
                border: 1px solid {self.colors.neutral};
                border-radius: 5px;
                padding: 5px;
            }}
            QHeaderView::section {{
                background-color: {self.colors.primary};
                color: white;
                padding: 10px;
                font-size: 16px;
                border: none;
            }}
        """)
        
        # Set column widths
        header = self.inventory_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Stretch)
        header.setSectionResizeMode(1, QHeaderView.Fixed)
        self.inventory_table.setColumnWidth(1, 150)
        
        # Add PPE items and restore last known values
        ppe_items = ['Hard Hat', 'Beard Net', 'Gloves', 'Safety Glasses', 'Ear Plugs']
        self.inventory_table.setRowCount(len(ppe_items))
        
        for i, item in enumerate(ppe_items):
            # Item name
            name_item = QTableWidgetItem(item)
            name_item.setFlags(name_item.flags() & ~Qt.ItemIsEditable)
            self.inventory_table.setItem(i, 0, name_item)
            
            # Get the key for this item
            key = item.lower().replace(' ', '')
            
            # Quantity (use stored value if available)
            qty_item = QTableWidgetItem(str(self.last_inventory.get(key, '--')))
            qty_item.setFlags(qty_item.flags() & ~Qt.ItemIsEditable)
            qty_item.setTextAlignment(Qt.AlignCenter)
            self.inventory_table.setItem(i, 1, qty_item)
        
        layout.addWidget(self.inventory_table)
        
        # Request button
        request_button = QPushButton("Request Inventory Update")
        request_button.setFont(QFont('Arial', 16))
        request_button.setMinimumHeight(60)
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
        
        # Last update label - use saved timestamp if available
        self.last_update_label = QLabel(f"Last Update: {self.last_update_time}")
        self.last_update_label.setAlignment(Qt.AlignCenter)
        self.last_update_label.setFont(QFont('Arial', 14))
        self.last_update_label.setStyleSheet(f"color: {self.colors.text_secondary};")
        layout.addWidget(self.last_update_label)
        
        return widget

    def _create_override_log_tab(self):
        """Create override log tab for settings"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(20)

        # Create tab title
        override_label = QLabel("Override Log")
        override_label.setFont(QFont('Arial', 24, QFont.Bold))
        layout.addWidget(override_label)  # Add title to the layout
        
        # Create table
        log_table = QTableWidget()
        log_table.setColumnCount(3)  # Three columns: Timestamp, User, Reason
        log_table.setHorizontalHeaderLabels(['Timestamp', 'User', 'Reason'])  # Update header labels
        
        # Set up table properties
        log_table.setFont(QFont('Arial', 12))
        log_table.verticalHeader().setVisible(False)
        log_table.verticalHeader().setDefaultSectionSize(30)
        log_table.horizontalHeader().setMinimumHeight(40)
        log_table.setMinimumHeight(200)
        log_table.setStyleSheet(f"""
            QTableWidget {{
                background-color: {self.colors.surface};
                color: {self.colors.text};
                border: 1px solid {self.colors.neutral};
                border-radius: 5px;
                padding: 5px;
            }}
            QHeaderView::section {{
                background-color: {self.colors.primary};
                color: white;
                padding: 10px;
                font-size: 16px;
                border: none;
            }}
        """)
        
        # Set column widths
        header = log_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Fixed)
        header.setSectionResizeMode(1, QHeaderView.Fixed)  # Fixed width for User
        header.setSectionResizeMode(2, QHeaderView.Stretch)  # Stretch for Reason
        log_table.setColumnWidth(0, 170)  # Fixed width for Timestamp
        log_table.setColumnWidth(1, 100)  # Fixed width for User

        # Add log entries
        recent_logs = self.override_logger.get_recent_logs()
        log_table.setRowCount(len(recent_logs))
        
        for i, log in enumerate(recent_logs):
            # Timestamp
            time_item = QTableWidgetItem(log['timestamp'])
            time_item.setFlags(time_item.flags() & ~Qt.ItemIsEditable)
            time_item.setTextAlignment(Qt.AlignLeft)
            log_table.setItem(i, 0, time_item)
            
            # Extract User from Reason
            reason_text = log['reason']
            user = reason_text.split(" - ")[0].replace("User: ", "").strip()  # Extract user
            reason = reason_text.split(" - ")[1].replace("Reason: ", "").strip()  # Extract reason
            
            # User
            user_item = QTableWidgetItem(user)
            user_item.setFlags(user_item.flags() & ~Qt.ItemIsEditable)
            user_item.setTextAlignment(Qt.AlignLeft)
            log_table.setItem(i, 1, user_item)
            
            # Reason
            details_item = QTableWidgetItem(reason)
            details_item.setFlags(details_item.flags() & ~Qt.ItemIsEditable)
            details_item.setTextAlignment(Qt.AlignLeft)
            log_table.setItem(i, 2, details_item)
        
        layout.addWidget(log_table)
        
        # Add separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet(f"background-color: {self.colors.neutral};")
        layout.addWidget(separator)
        
        # Add gate override toggle section at the bottom
        gate_override_widget = QWidget()
        gate_override_layout = QHBoxLayout(gate_override_widget)
        gate_override_layout.setAlignment(Qt.AlignCenter)
        
        # Gate override toggle button
        self.temp_gate_override_button = QPushButton("GATE LOCKED")
        self.temp_gate_override_button.setFont(QFont('Arial', 20, QFont.Bold))
        self.temp_gate_override_button.setMinimumSize(300, 60)
        self.temp_gate_override_button.setCheckable(True)
        self.temp_gate_override_button.setChecked(not self.safety_gate_locked)  # Set initial state
        self._update_temp_gate_override_button_style()
        self.temp_gate_override_button.clicked.connect(self._update_temp_gate_override_button_style)
        
        gate_override_layout.addWidget(self.temp_gate_override_button)
        layout.addWidget(gate_override_widget)
        
        return widget

    def _update_temp_gate_override_button_style(self):
        """Update temporary gate override button style based on state"""
        is_unlocked = self.temp_gate_override_button.isChecked()
        self.temp_gate_override_button.setText("GATE UNLOCKED" if is_unlocked else "GATE LOCKED")
        self.temp_gate_override_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.colors.warning if is_unlocked else self.colors.success};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {'#f57c00' if is_unlocked else '#1b5e20'};
            }}
        """)

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
        
        # Apply gate override if changed
        if hasattr(self, 'temp_gate_override_button'):
            is_unlocked = self.temp_gate_override_button.isChecked()
            if is_unlocked != (not self.safety_gate_locked):  # If state has changed
                self.safety_gate_locked = not is_unlocked
                self.ros_node.publish_gate_status(self.safety_gate_locked)
                
                # Log the change
                action = "unlocked" if is_unlocked else "locked"
                self.override_logger.log_override(
                    f"User: Admin - Reason: Gate manually {action} from settings"
                )
                
                # Show status message
                self.show_status(f"Gate manually {action}", "orange" if is_unlocked else "green")
        
        # Return to main view
        self.switchContent()

    def _handle_settings_cancel(self):
        """Cancel settings changes and return to main view"""
        # Reset gate override button state if it exists
        if hasattr(self, 'temp_gate_override_button'):
            self.temp_gate_override_button.setChecked(not self.safety_gate_locked)
            self._update_temp_gate_override_button_style()
        
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
            # Save inventory data before closing
            self._save_inventory_data()
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
            
            # Store the inventory data
            for key, value in inventory_data.items():
                self.last_inventory[key] = str(value)
            
            # Update timestamp with standardized format
            self.last_update_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # Save to file
            self._save_inventory_data()
            
            # Update table if it exists
            if hasattr(self, 'inventory_table'):
                self._update_inventory_table()
            
            # Update timestamp label
            if hasattr(self, 'last_update_label'):
                self.last_update_label.setText(f"Last Update: {self.last_update_time}")
            
        except Exception as e:
            self.show_status("Error updating inventory", "red")
            print(f"Error parsing inventory data: {e}")

    def _update_inventory_table(self):
        """Update inventory table with stored values"""
        for row in range(self.inventory_table.rowCount()):
            item_name = self.inventory_table.item(row, 0).text()
            key = item_name.lower().replace(' ', '')
            if key in self.last_inventory:
                qty_item = QTableWidgetItem(str(self.last_inventory[key]))
                qty_item.setFlags(qty_item.flags() & ~Qt.ItemIsEditable)
                qty_item.setTextAlignment(Qt.AlignCenter)
                self.inventory_table.setItem(row, 1, qty_item)

    def _ensure_json_support_directory_exists(self):
        """Ensure the jsonSupport directory exists."""
        if not os.path.exists(self.inventory_log_dir):
            os.makedirs(self.inventory_log_dir)

    def _load_inventory_data(self):
        """Load inventory data from file"""
        try:
            if os.path.exists(self.inventory_file):
                with open(self.inventory_file, 'r') as f:
                    data = json.load(f)
                    # Convert ISO format to standard format if needed
                    last_update = data.get('last_update', "Never")
                    if last_update != "Never" and 'T' in last_update:
                        try:
                            dt = datetime.fromisoformat(last_update)
                            last_update = dt.strftime("%Y-%m-%d %H:%M:%S")
                        except:
                            last_update = "Never"
                    
                    return {
                        'inventory': data.get('inventory', self._get_default_inventory()),
                        'last_update': last_update
                    }
        except Exception as e:
            print(f"Error loading inventory data: {e}")
        
        return {
            'inventory': self._get_default_inventory(),
            'last_update': "Never"
        }

    def _get_default_inventory(self):
        """Get default inventory values"""
        return {
            'hardhat': '--',
            'beardnet': '--',
            'gloves': '--',
            'safetyglasses': '--',
            'earplugs': '--'
        }

    def _save_inventory_data(self):
        """Save inventory data to file"""
        try:
            data = {
                'inventory': self.last_inventory,
                'last_update': self.last_update_time
            }
            with open(self.inventory_file, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print(f"Error saving inventory data: {e}")

    def _create_report_tab(self):
        """Create the report tab."""
        report_widget = QWidget()
        report_layout = QVBoxLayout(report_widget)
        
        # Add title for the reporting tab
        report_label = QLabel("PPE Dispensing Report")
        report_label.setFont(QFont('Arial', 24, QFont.Bold))
        report_layout.addWidget(report_label)  # Add title to the layout
        
        # Generate the report data
        report_data = generate_report()
        
        # Create charts using the item counts
        items = list(report_data["data"].keys())
        counts = list(report_data["data"].values())
        timestamps = [event['timestamp'] for event in report_data["events"]]  # Get timestamps
        
        # Create a vertical layout for the charts
        chart_layout = QVBoxLayout()  # Changed to vertical layout to stack charts
        
        # Create a figure for the pie chart
        fig, ax = plt.subplots()
        wedges, texts, autotexts = ax.pie(
            counts, 
            labels=items, 
            autopct=lambda p: f'{p:.1f}%\n({int(p * sum(counts) / 100)})', 
            startangle=90
        )
        ax.axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.
        
        # Customize the pie chart text
        for text in autotexts:
            text.set_color('white')  # Set the color of the percentage text to white
        
        # Create a canvas to display the pie chart
        pie_canvas = FigureCanvas(fig)
        chart_layout.addWidget(pie_canvas)  # Add to layout without fixed size
        
        # Create a bar chart for total counts
        bar_fig, bar_ax = plt.subplots()
        bar_ax.bar(items, counts)
        bar_ax.set_xlabel('PPE Items')
        bar_ax.set_ylabel('Count')
        bar_ax.set_title('Total PPE Dispensed Count')
        
        # Create a canvas to display the bar chart
        bar_canvas = FigureCanvas(bar_fig)
        chart_layout.addWidget(bar_canvas)  # Add to layout without fixed size
        
        # Add the chart layout to the main report layout
        report_layout.addLayout(chart_layout)
        
        # Create the Format button
        format_button = QPushButton("Format Report Memory")
        format_button.setFont(QFont('Arial', 20, QFont.Bold))
        format_button.setStyleSheet(f"""
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
        format_button.clicked.connect(self.clear_dispensing_log)  # Connect to the clear function
        
        report_layout.addWidget(format_button)  # Add the button to the layout
        
        # Create the Export button
        export_button = QPushButton("Export as .CSV")
        export_button.setFont(QFont('Arial', 20, QFont.Bold))
        export_button.setStyleSheet(f"""
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
        export_button.clicked.connect(self.export_report_to_csv)  # Connect to the export function
        
        report_layout.addWidget(export_button)  # Add the export button to the layout
        
        report_widget.update()  # Update the report widget
        return report_widget 

    def export_report_to_csv(self):
        """Export the report data to a CSV file."""
        report_data = generate_report()
        items = list(report_data["data"].keys())
        counts = list(report_data["data"].values())
        timestamps = [event['timestamp'] for event in report_data["events"]]  # Get timestamps
        
        # Define the CSV file path
        filename = "ppe_dispensing_report.csv"

        # Construct the path to the JsonSupport directory
        json_support_dir = os.path.join(os.getcwd(), "src", "ppe_gui_package", "gui_package", "main_gui_modules", "jsonSupport")

        # Ensure the directory exists
        os.makedirs(json_support_dir, exist_ok=True)

        # Construct the full file path
        csv_file_path = os.path.join(json_support_dir, filename)
        
        # Write to CSV
        with open(csv_file_path, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['Item', 'Count', 'Timestamp'])  # Write header
            for item, count, timestamp in zip(items, counts, timestamps):
                csv_writer.writerow([item, count, timestamp])  # Write data rows
        
        # Optionally, you can log the export action or update the UI in some way
        print(f"The report has been exported to {csv_file_path}.")  # Log to console for debugging

    def clear_dispensing_log(self):
        """Clear the dispensing log."""
        log_file_path = os.path.join(os.getcwd(), "src", "ppe_gui_package", "gui_package", "main_gui_modules", "jsonSupport", "dispensing_log.json")
        
        # Clear the log file
        with open(log_file_path, 'w') as log_file:
            log_file.write('[]')  # Write an empty JSON array to clear the log
        
        # Update the report tab to reflect the cleared log
        self.update_report_tab() 

    def update_report_tab(self):
        """Update the report tab to reflect the current state of the dispensing log."""
        report_tab_index = 4  # Adjust this index based on your tab order
        self.tabs.setCurrentIndex(report_tab_index)  # Switch to the report tab
        self.tabs.widget(report_tab_index).setParent(None)  # Remove the current report widget
        self.tabs.addTab(self._create_report_tab(), "5.")  # Recreate and add the report tab

    def _create_info_tab(self):
        """Create the info tab for settings content."""
        info_page = QWidget()
        info_layout = QVBoxLayout(info_page)
        
        # Set a background color for the info page
        #info_page.setStyleSheet("background-color: #f0f0f0;")  # Light gray background

        info_label = QLabel("Settings Tabs Overview")
        info_label.setFont(QFont('Arial', 24, QFont.Bold))
        info_label.setStyleSheet("color: #333;")  # Dark text color
        info_layout.addWidget(info_label)

        # List of tabs and their descriptions
        tab_info = [
            ("1. ", "Appearance settings including color schemes."),
            ("2. ", "Inventory management settings."),
            ("3. ", "Timing and delay settings."),
            ("4. ", "Override logging and configuration."),
            ("5. ", "Report generation and analytics.")
        ]

        for tab_name, description in tab_info:
            label = QLabel(f"{tab_name} {description}")
            label.setFont(QFont('Arial', 16))  # Slightly smaller font for descriptions
            label.setStyleSheet("color: #555; padding: 5px;")  # Medium gray text with padding
            info_layout.addWidget(label)
        return info_page