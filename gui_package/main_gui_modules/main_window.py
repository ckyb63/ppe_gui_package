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
                            QApplication, QTableWidgetItem)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from std_msgs.msg import String

from .widgets.sections import TitleSection, StatusSection, CameraSection, PPEGridSection
from .utils.colors import ColorScheme
from .utils.logger import OverrideLogger
from .widgets.override import OverrideContent
from .widgets.help import HelpContent
from .widgets.settings import SettingsContent, ReportTab
from .utils.json_handler import JsonHandler
from .utils.report_handler import ReportHandler
from .utils.settings_handler import SettingsHandler
from .utils.accessibility_handler import AccessibilityHandler

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
        
        # Use the current working directory to construct the path
        self.inventory_log_dir = os.path.join(os.getcwd(), "src", "ppe_gui_package", "gui_package", "main_gui_modules", "jsonSupport")
        self.inventory_file = os.path.join(self.inventory_log_dir, "inventory_data.json")
        
        # Initialize handlers
        self.json_handler = JsonHandler(self.inventory_file, self.inventory_log_dir)
        self.report_handler = ReportHandler()
        self.settings_handler = SettingsHandler(self)
        self.accessibility_handler = AccessibilityHandler(self)
        self.accessibility_mode = self.accessibility_handler.accessibility_mode
        
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
        return HelpContent(self)

    def _create_settings_content(self):
        """Create settings content widget"""
        return SettingsContent(self)

    def _create_override_content(self):
        """Create override confirmation content"""
        return OverrideContent(self)

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

    def _apply_theme_preview(self):
        """Apply theme changes in real-time"""
        settings = self.findChild(SettingsContent)
        if settings:
            colors_tab = settings.tabs.widget(1)  # Colors tab is at index 1
            is_dark = colors_tab.color_scheme_combo.currentText() == "Dark"
            if is_dark != self.colors.is_dark:
                self.colors.is_dark = is_dark
                self.colors.update_colors()
                self.apply_theme()

    def _handle_settings_save(self):
        """Save settings and return to main view"""
        settings = self.findChild(SettingsContent)
        colors_tab = settings.tabs.widget(1)  # Colors tab
        timing_tab = settings.tabs.widget(3)  # Timing tab
        override_tab = settings.tabs.widget(4)  # Override log tab
        
        # Apply color scheme
        is_dark = colors_tab.color_scheme_combo.currentText() == "Dark"
        if is_dark != self.colors.is_dark:
            self.colors.is_dark = is_dark
            self.colors.update_colors()
            self.apply_theme()
        
        # Apply timing settings
        self.override_duration = float(timing_tab.override_duration_spin.value())
        self.dispense_cooldown = float(timing_tab.cooldown_time_spin.value())
        
        # Apply gate override if changed
        if hasattr(override_tab, 'temp_gate_override_button'):
            is_unlocked = override_tab.temp_gate_override_button.isChecked()
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
        settings = self.findChild(SettingsContent)
        if settings:
            override_tab = settings.tabs.widget(4)  # Override log tab
            if hasattr(override_tab, 'temp_gate_override_button'):
                override_tab.temp_gate_override_button.setChecked(not self.safety_gate_locked)
                override_tab._update_gate_override_button_style()
        
        self.switchContent()

    def toggle_accessibility_settings(self):
        """Toggle accessibility mode from settings screen"""
        self.accessibility_mode = self.accessibility_handler.toggle_accessibility_mode()
        self._update_settings_toggle_button_style()

    def _update_settings_toggle_button_style(self):
        """Update settings toggle button styling"""
        settings = self.findChild(SettingsContent)
        if settings:
            colors_tab = settings.tabs.widget(1)  # Colors tab
            colors_tab.settings_toggle_button.setStyleSheet(f"""
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

    def clear_dispensing_log(self):
        """Clear the dispensing log"""
        try:
            # Use JsonHandler to clear the log with proper structure
            self.json_handler.write_to_json(
                "dispensing_log.json", 
                {"events": []}  # Keep the expected structure with empty events array
            )
            self.show_status("Dispensing log cleared successfully", "green")
            
            # Update report display if settings tab is visible
            settings = self.findChild(SettingsContent)
            if settings and settings.isVisible():
                report_tab = settings.tabs.widget(5)  # Report tab is at index 5
                if isinstance(report_tab, ReportTab):
                    report_tab.update_report_display()
                    
            return True, "Log cleared successfully"
        except Exception as e:
            self.show_status(f"Error clearing log: {str(e)}", "red")
            return False, f"Failed to clear log: {str(e)}"

    def export_report_to_csv(self):
        """Export the report to CSV"""
        try:
            # Get the correct path in jsonSupport directory
            csv_filename = f"dispensing_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            csv_path = os.path.join(self.inventory_log_dir, csv_filename)
            
            # Use report handler with correct path
            success, message = self.report_handler.export_report_to_csv(csv_path)
            self.show_status(message, "green" if success else "red")
        except Exception as e:
            self.show_status(f"Error exporting report: {str(e)}", "red")

    def request_inventory_update(self):
        """Request an inventory update from the ROS node"""
        if hasattr(self, 'ros_node'):
            self.ros_node.request_inventory_update()
            self.show_status("Requesting inventory update...", "blue")
        else:
            self.show_status("ROS node not initialized", "red")

    def unlock_gate(self):
        # Publish the gate unlock message
        msg = String()
        msg.data = "OVERRIDE"
        self.override_publisher.publish(msg)
        
    def save_settings(self):
        # Save current settings to a file/database
        # Implement the save logic
        pass

    def publish_gate_command(self, command):
        """Publish gate command to ROS topic"""
        # Convert command string to boolean for gate topic
        is_locked = command == "lock"
        self.ros_node.publish_gate_status(is_locked)

    def log_dispense_event(self, item):
        """Log a dispense event to the JSON file"""
        try:
            event_data = {
                "item": item,
                "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            }
            self.json_handler.append_to_json("dispensing_log.json", event_data)
            
            # Update report display if settings tab is visible
            settings = self.findChild(SettingsContent)
            if settings and settings.isVisible():
                report_tab = settings.tabs.widget(5)  # Report tab is at index 5
                if isinstance(report_tab, ReportTab):
                    report_tab.update_report_display()
        except Exception as e:
            print(f"Error logging dispense event: {e}")