"""
Settings content widget and tabs

Author: Max Chen
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                            QPushButton, QComboBox, QTabWidget, QFrame,
                            QSpinBox, QDoubleSpinBox, QTableWidget, 
                            QTableWidgetItem, QHeaderView)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QPalette, QColor
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from ..jsonSupport.reporting import generate_report
from datetime import datetime

class SettingsContent(QWidget):
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
        
        settings_icon = QLabel("⚙️")
        settings_icon.setFont(QFont('Arial', 48, QFont.Bold))
        settings_icon.setStyleSheet(f"""
            QLabel {{
                color: {self.parent.colors.primary};
                background-color: {self.parent.colors.surface};
                border-radius: 40px;
                padding: 10px;
                min-width: 80px;
                min-height: 80px;
            }}
        """)
        settings_icon.setAlignment(Qt.AlignCenter)
        
        title_text = QLabel("SETTINGS")
        title_text.setFont(QFont('Arial', 36, QFont.Bold))
        title_text.setStyleSheet(f"color: {self.parent.colors.primary};")
        
        title_layout.addWidget(settings_icon)
        title_layout.addWidget(title_text)
        layout.addWidget(title_widget)

        # Create tabs with custom styling
        self.tabs = QTabWidget()
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
        
        # Add tabs
        self.tabs.addTab(InfoTab(self), "Info.")
        self.tabs.addTab(ColorsTab(self), "1.")
        self.tabs.addTab(InventoryTab(self), "2.")
        self.tabs.addTab(TimingTab(self), "3.")
        self.tabs.addTab(OverrideLogTab(self), "4.")
        self.tabs.addTab(ReportTab(self), "5.")
        
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
                background-color: {self.parent.colors.neutral};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: #555;
            }}
        """)
        cancel_button.clicked.connect(self.parent._handle_settings_cancel)
        
        # Save button
        save_button = QPushButton("Save")
        save_button.setFont(QFont('Arial', 20, QFont.Bold))
        save_button.setFixedSize(200, 80)
        save_button.setStyleSheet(f"""
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
        save_button.clicked.connect(self.parent._handle_settings_save)
        
        button_layout.addWidget(cancel_button)
        button_layout.addWidget(save_button)
        layout.addWidget(button_container)

class InfoTab(QWidget):
    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent
        self._init_ui()
        
    def _init_ui(self):
        layout = QVBoxLayout(self)
        
        info_label = QLabel("Settings Tabs Overview")
        info_label.setFont(QFont('Arial', 24, QFont.Bold))
        info_label.setStyleSheet("color: #333;")  # Dark text color
        layout.addWidget(info_label)

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
            layout.addWidget(label)

class ColorsTab(QWidget):
    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent
        self._init_ui()
        
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignTop)
        layout.setSpacing(20)

        # Create tab title
        colors_label = QLabel("Appearance")
        colors_label.setFont(QFont('Arial', 24, QFont.Bold))
        layout.addWidget(colors_label)
        
        # Color scheme section
        scheme_label = QLabel("Color Scheme:")
        scheme_label.setFont(QFont('Arial', 18))
        layout.addWidget(scheme_label)
        
        self.color_scheme_combo = QComboBox()
        self.color_scheme_combo.setObjectName("color_scheme")
        self.color_scheme_combo.setFont(QFont('Arial', 20))
        self.color_scheme_combo.setMinimumHeight(70)
        self.color_scheme_combo.setFixedHeight(50)
        self.color_scheme_combo.setStyleSheet("""
            QComboBox {
                padding: 10px;
                min-width: 200px;
                font-size: 24px;
            }
        """)
        self.color_scheme_combo.addItems(["Light", "Dark"])
        self.color_scheme_combo.setCurrentText("Dark" if self.parent.parent.colors.is_dark else "Light")
        self.color_scheme_combo.currentTextChanged.connect(self.parent.parent._apply_theme_preview)
        layout.addWidget(self.color_scheme_combo)
        
        # Add separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet(f"background-color: {self.parent.parent.colors.neutral};")
        layout.addWidget(separator)
        
        # Accessibility section
        access_label = QLabel("Accessibility:")
        access_label.setFont(QFont('Arial', 18))
        layout.addWidget(access_label)
        
        self.settings_toggle_button = QPushButton(
            "Accessibility O/X OFF" if not self.parent.parent.accessibility_mode else "Accessibility O/X ON"
        )
        self.settings_toggle_button.setFont(QFont('Arial', 24, QFont.Bold))
        self.settings_toggle_button.setFixedHeight(90)
        self._update_toggle_button_style()
        self.settings_toggle_button.clicked.connect(self.toggle_accessibility)
        layout.addWidget(self.settings_toggle_button)
        
        layout.addStretch()

    def _update_toggle_button_style(self):
        """Update settings toggle button styling"""
        self.settings_toggle_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.parent.parent.colors.success if self.parent.parent.accessibility_mode else self.parent.parent.colors.neutral};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.parent.parent.colors.success if self.parent.parent.accessibility_mode else '#555'};
            }}
        """)

    def toggle_accessibility(self):
        self.parent.parent.toggle_accessibility_settings()
        # Update button text after toggle
        self.settings_toggle_button.setText(
            "Accessibility O/X ON" if self.parent.parent.accessibility_mode else "Accessibility O/X OFF"
        )

class TimingTab(QWidget):
    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent
        self._init_ui()
        
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(40)
        layout.setContentsMargins(30, 30, 30, 30)

        # Create tab title
        timing_label = QLabel("Timing")
        timing_label.setFont(QFont('Arial', 24, QFont.Bold))
        layout.addWidget(timing_label)
        
        # Override Duration Section
        override_section = QWidget()
        override_layout = QVBoxLayout(override_section)
        override_layout.setSpacing(10)
        
        override_title = QLabel("Override Duration (s)")
        override_title.setFont(QFont('Arial', 24, QFont.Bold))
        override_title.setPalette(self._create_palette(self.parent.parent.colors.text))
        override_title.setWordWrap(True)
        
        self.override_duration_spin = QSpinBox()
        self.override_duration_spin.setFont(QFont('Arial', 20, QFont.Bold))
        self.override_duration_spin.setMinimumHeight(100)
        self.override_duration_spin.setMinimumWidth(250)
        self.override_duration_spin.setRange(5, 30)
        self.override_duration_spin.setValue(int(self.parent.parent.override_duration))
        self._style_spinbox(self.override_duration_spin)
        
        override_layout.addWidget(override_title)
        override_layout.addWidget(self.override_duration_spin)
        
        # Cooldown Time Section
        cooldown_section = QWidget()
        cooldown_layout = QVBoxLayout(cooldown_section)
        cooldown_layout.setSpacing(10)
        
        cooldown_title = QLabel("PPE Request Cooldown Time (s)")
        cooldown_title.setFont(QFont('Arial', 24, QFont.Bold))
        cooldown_title.setPalette(self._create_palette(self.parent.parent.colors.text))
        cooldown_title.setWordWrap(True)
        
        self.cooldown_time_spin = QDoubleSpinBox()
        self.cooldown_time_spin.setFont(QFont('Arial', 20, QFont.Bold))
        self.cooldown_time_spin.setMinimumHeight(100)
        self.cooldown_time_spin.setMinimumWidth(250)
        self.cooldown_time_spin.setRange(0.5, 5.0)
        self.cooldown_time_spin.setSingleStep(0.5)
        self.cooldown_time_spin.setValue(self.parent.parent.dispense_cooldown)
        self._style_spinbox(self.cooldown_time_spin)
        
        cooldown_layout.addWidget(cooldown_title)
        cooldown_layout.addWidget(self.cooldown_time_spin)

        # Add sections to main layout
        layout.addWidget(override_section)
        layout.addSpacing(30)
        layout.addWidget(cooldown_section)
        layout.addStretch()

    def _style_spinbox(self, spinbox):
        """Apply common styling to spinbox widgets"""
        # Set colors using palette
        palette = QPalette()
        palette.setColor(QPalette.Text, QColor(self.parent.parent.colors.text))
        palette.setColor(QPalette.Base, QColor(self.parent.parent.colors.surface))
        palette.setColor(QPalette.Button, QColor(self.parent.parent.colors.surface))
        spinbox.setPalette(palette)
        
        # Set frame and alignment
        spinbox.setFrame(True)
        spinbox.setAlignment(Qt.AlignCenter)
        
        # Style the buttons
        spinbox.setButtonSymbols(QSpinBox.UpDownArrows)
        
        # Set stylesheet for the increment buttons
        spinbox.setStyleSheet("""
            QSpinBox, QDoubleSpinBox {
                padding: 10px;
            }
            QAbstractSpinBox::up-button {
                width: 50px;
                height: 50px;
                background-color: #28a745;
                color: white;
                border: none;
                border-radius: 5px;
            }
            QAbstractSpinBox::down-button {
                width: 50px;
                height: 50px;
                background-color: #dc3545;
                color: white;
                border: none;
                border-radius: 5px;
            }
            QAbstractSpinBox::up-button:hover {
                background-color: #218838;
            }
            QAbstractSpinBox::down-button:hover {
                background-color: #c82333;
            }
        """)

    def _create_palette(self, color):
        """Create a palette with the specified text color"""
        palette = QPalette()
        palette.setColor(QPalette.WindowText, QColor(color))
        return palette

class InventoryTab(QWidget):
    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent
        self._init_ui()
        
    def _init_ui(self):
        layout = QVBoxLayout(self)
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
                background-color: {self.parent.parent.colors.surface};
                color: {self.parent.parent.colors.text};
                border: 1px solid {self.parent.parent.colors.neutral};
                border-radius: 5px;
                padding: 5px;
            }}
            QHeaderView::section {{
                background-color: {self.parent.parent.colors.primary};
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
            qty_item = QTableWidgetItem(str(self.parent.parent.last_inventory.get(key, '--')))
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
                background-color: {self.parent.parent.colors.primary};
                color: white;
                border: none;
                border-radius: 5px;
                padding: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.parent.parent.colors.primary_dark};
            }}
        """)
        request_button.clicked.connect(self.parent.parent.request_inventory_update)
        layout.addWidget(request_button)
        
        # Last update label - use saved timestamp if available
        self.last_update_label = QLabel(f"Last Update: {self.parent.parent.last_update_time}")
        self.last_update_label.setAlignment(Qt.AlignCenter)
        self.last_update_label.setFont(QFont('Arial', 14))
        self.last_update_label.setStyleSheet(f"color: {self.parent.parent.colors.text_secondary};")
        layout.addWidget(self.last_update_label)

class OverrideLogTab(QWidget):
    def __init__(self, parent=None):
        super().__init__()
        self.parent = parent
        self._init_ui()
        
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(20)

        # Create tab title
        override_label = QLabel("Override Log")
        override_label.setFont(QFont('Arial', 24, QFont.Bold))
        layout.addWidget(override_label)
        
        # Create table
        log_table = QTableWidget()
        log_table.setColumnCount(3)  # Three columns: Timestamp, User, Reason
        log_table.setHorizontalHeaderLabels(['Timestamp', 'User', 'Reason'])
        
        # Set up table properties
        log_table.setFont(QFont('Arial', 12))
        log_table.verticalHeader().setVisible(False)
        log_table.verticalHeader().setDefaultSectionSize(30)
        log_table.horizontalHeader().setMinimumHeight(40)
        log_table.setMinimumHeight(200)
        log_table.setStyleSheet(f"""
            QTableWidget {{
                background-color: {self.parent.parent.colors.surface};
                color: {self.parent.parent.colors.text};
                border: 1px solid {self.parent.parent.colors.neutral};
                border-radius: 5px;
                padding: 5px;
            }}
            QHeaderView::section {{
                background-color: {self.parent.parent.colors.primary};
                color: white;
                padding: 10px;
                font-size: 16px;
                border: none;
            }}
        """)

        # Set column widths
        header = log_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Fixed)
        header.setSectionResizeMode(1, QHeaderView.Fixed)
        header.setSectionResizeMode(2, QHeaderView.Stretch)
        log_table.setColumnWidth(0, 170)
        log_table.setColumnWidth(1, 100)

        # Add log entries
        recent_logs = self.parent.parent.override_logger.get_recent_logs()
        log_table.setRowCount(len(recent_logs))
        
        for i, log in enumerate(recent_logs):
            # Timestamp
            time_item = QTableWidgetItem(log['timestamp'])
            time_item.setFlags(time_item.flags() & ~Qt.ItemIsEditable)
            time_item.setTextAlignment(Qt.AlignLeft)
            log_table.setItem(i, 0, time_item)
            
            # Extract User from Reason
            reason_text = log['reason']
            user = reason_text.split(" - ")[0].replace("User: ", "").strip()
            reason = reason_text.split(" - ")[1].replace("Reason: ", "").strip()
            
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
        separator.setStyleSheet(f"background-color: {self.parent.parent.colors.neutral};")
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
        self.temp_gate_override_button.setChecked(False)  # Start with locked state
        self.temp_gate_override_button.clicked.connect(self._handle_gate_toggle)
        self._update_gate_override_button_style()
        
        gate_override_layout.addWidget(self.temp_gate_override_button)
        layout.addWidget(gate_override_widget)

    def _update_gate_override_button_style(self):
        """Update temporary gate override button style based on state"""
        is_unlocked = self.temp_gate_override_button.isChecked()
        self.temp_gate_override_button.setText("GATE UNLOCKED" if is_unlocked else "GATE LOCKED")
        self.temp_gate_override_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.parent.parent.colors.success if is_unlocked else self.parent.parent.colors.warning};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {'#1b5e20' if is_unlocked else '#f57c00'};
            }}
        """)

    def _handle_gate_toggle(self):
        """Handle gate toggle button click"""
        is_unlocked = self.temp_gate_override_button.isChecked()
        
        if is_unlocked:
            # Send unlock command via ROS
            self.parent.parent.publish_gate_command("unlock")
            self.temp_gate_override_button.setText("GATE UNLOCKED")
        else:
            # Send lock command via ROS
            self.parent.parent.publish_gate_command("lock") 
            self.temp_gate_override_button.setText("GATE LOCKED")
        
        # Log the override using JsonHandler
        action = "unlocked" if is_unlocked else "locked"
        override_data = {
            "item": "OVERRIDE",
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        self.parent.parent.json_handler.append_to_json(
            "dispensing_log.json", 
            override_data
        )
        
        self._update_gate_override_button_style()

class ReportTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self._init_ui()

    def _init_ui(self):
        """Initialize the report tab UI"""
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        
        # Add title for the reporting tab
        report_label = QLabel("PPE Dispensing Report")
        report_label.setFont(QFont('Arial', 24, QFont.Bold))
        self.layout.addWidget(report_label)
        
        self.update_report_display()
        
        # Create the Format button
        format_button = QPushButton("Format Report Memory")
        format_button.setFont(QFont('Arial', 20, QFont.Bold))
        format_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.parent.parent.colors.primary};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.parent.parent.colors.primary_dark};
            }}
        """)
        format_button.clicked.connect(self.parent.parent.clear_dispensing_log)
        self.layout.addWidget(format_button)
        
        # Create the Export button
        export_button = QPushButton("Export as .CSV")
        export_button.setFont(QFont('Arial', 20, QFont.Bold))
        export_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.parent.parent.colors.primary};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.parent.parent.colors.primary_dark};
            }}
        """)
        export_button.clicked.connect(self.parent.parent.export_report_to_csv)
        self.layout.addWidget(export_button)

    def update_report_display(self):
        """Update the report display with current data"""
        # First, remove all widgets except the title and buttons
        while self.layout.count() > 3:  # Keep title and two buttons
            item = self.layout.takeAt(1)  # Remove item after title
            if item.widget():
                item.widget().deleteLater()
            elif item.layout():
                # If it's a layout, remove and delete its widgets too
                while item.layout().count():
                    child = item.layout().takeAt(0)
                    if child.widget():
                        child.widget().deleteLater()
                item.layout().setParent(None)

        # Generate the report data
        report_data = generate_report()
        
        if report_data and report_data.get("data"):
            items = list(report_data["data"].keys())
            counts = list(report_data["data"].values())
            
            if items and counts:
                # Create a vertical layout for the charts
                chart_layout = QVBoxLayout()
                
                # Create a figure for the pie chart
                fig, ax = plt.subplots()
                wedges, texts, autotexts = ax.pie(
                    counts, 
                    labels=items, 
                    autopct=lambda p: f'{p:.1f}%\n({int(p * sum(counts) / 100)})', 
                    startangle=90
                )
                ax.axis('equal')
                
                # Customize the pie chart text
                for text in autotexts:
                    text.set_color('white')
                
                # Create a canvas to display the pie chart
                pie_canvas = FigureCanvas(fig)
                chart_layout.addWidget(pie_canvas)
                
                # Create a bar chart for total counts
                bar_fig, bar_ax = plt.subplots()
                bar_ax.bar(items, counts)
                bar_ax.set_xlabel('PPE Items')
                bar_ax.set_ylabel('Count')
                bar_ax.set_title('Total PPE Dispensed Count')
                
                # Create a canvas to display the bar chart
                bar_canvas = FigureCanvas(bar_fig)
                chart_layout.addWidget(bar_canvas)
                
                # Insert chart layout after the title but before the buttons
                self.layout.insertLayout(1, chart_layout)
        else:
            # Show a message when no data is available
            no_data_label = QLabel("No dispensing data available")
            no_data_label.setAlignment(Qt.AlignCenter)
            self.layout.insertWidget(1, no_data_label)

    def showEvent(self, event):
        """Called when the tab becomes visible"""
        super().showEvent(event)
        self.update_report_display() 