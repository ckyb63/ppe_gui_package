"""
Settings Dialog for PPE GUI

Author: Max Chen
v0.5.0
"""
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                            QPushButton, QComboBox, QWidget, QTextEdit,
                            QTabWidget, QFormLayout, QSpinBox, QDoubleSpinBox,
                            QFrame)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

class SettingsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.setWindowTitle("Settings")
        self.setModal(True)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowCloseButtonHint)
        
        # Store initial values for cancel
        self.initial_settings = {
            'is_dark': parent.colors.is_dark,
            'override_duration': parent.override_duration,
            'dispense_cooldown': parent.dispense_cooldown,
            'accessibility_mode': parent.accessibility_mode
        }
        
        self._init_ui()
        
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(20)
        
        # Create title section
        self._create_title_section(layout)
        
        # Create tabs
        tabs = QTabWidget()
        tabs.setFont(QFont('Arial', 12))
        tabs.addTab(self._create_colors_tab(), "Appearance")
        tabs.addTab(self._create_inventory_tab(), "Inventory")
        tabs.addTab(self._create_timing_tab(), "Timing")
        tabs.addTab(self._create_override_log_tab(), "Override Log")
        
        layout.addWidget(tabs)
        
        # Create button section
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

    def _create_colors_tab(self):
        widget = QWidget()
        widget.setObjectName("colors_tab")
        layout = QVBoxLayout(widget)
        layout.setAlignment(Qt.AlignTop)
        layout.setSpacing(20)
        
        # Color scheme section
        scheme_label = QLabel("Color Scheme:")
        scheme_label.setFont(QFont('Arial', 16))
        layout.addWidget(scheme_label)
        
        self.color_scheme = QComboBox()
        self.color_scheme.setObjectName("color_scheme")
        self.color_scheme.setFont(QFont('Arial', 14))
        self.color_scheme.addItems(["Light", "Dark"])
        self.color_scheme.setCurrentText("Dark" if self.parent.colors.is_dark else "Light")
        self.color_scheme.currentTextChanged.connect(self.apply_theme_preview)
        layout.addWidget(self.color_scheme)
        
        # Add separator
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet(f"background-color: {self.parent.colors.neutral};")
        layout.addWidget(separator)
        
        # Accessibility section
        access_label = QLabel("Accessibility:")
        access_label.setFont(QFont('Arial', 16))
        layout.addWidget(access_label)
        
        self.settings_toggle_button = QPushButton(
            "Accessibility O/X OFF" if not self.parent.accessibility_mode else "Accessibility O/X ON"
        )
        self.settings_toggle_button.setFont(QFont('Arial', 14, QFont.Bold))
        self.settings_toggle_button.setFixedHeight(50)
        self._update_toggle_button_style()
        self.settings_toggle_button.clicked.connect(self.toggle_accessibility)
        layout.addWidget(self.settings_toggle_button)
        
        layout.addStretch()
        return widget

    def _create_timing_tab(self):
        widget = QWidget()
        widget.setObjectName("timing_tab")
        layout = QFormLayout(widget)
        
        # Create spinboxes with labels
        override_label = QLabel("Override Duration (s):")
        override_label.setStyleSheet(f"color: {self.parent.colors.text};")
        self.override_duration_spin = QSpinBox()
        self.override_duration_spin.setObjectName("override_duration")
        self.override_duration_spin.setRange(5, 30)
        self.override_duration_spin.setValue(int(self.parent.override_duration))
        
        cooldown_label = QLabel("Cooldown Time (s):")
        cooldown_label.setStyleSheet(f"color: {self.parent.colors.text};")
        self.cooldown_time_spin = QDoubleSpinBox()
        self.cooldown_time_spin.setObjectName("cooldown_time")
        self.cooldown_time_spin.setRange(0.5, 5.0)
        self.cooldown_time_spin.setValue(self.parent.dispense_cooldown)
        
        layout.addRow(override_label, self.override_duration_spin)
        layout.addRow(cooldown_label, self.cooldown_time_spin)
        
        return widget

    def _create_inventory_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.addWidget(QLabel("Inventory tracking will be implemented here"))
        return widget

    def _create_override_log_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        log_text = QTextEdit()
        log_text.setReadOnly(True)
        log_text.setStyleSheet(f"""
            QTextEdit {{
                background-color: {self.parent.colors.surface};
                color: {self.parent.colors.text};
                border: 1px solid {self.parent.colors.neutral};
                border-radius: 5px;
                padding: 10px;
                font-size: 12pt;
            }}
        """)
        
        recent_logs = self.parent.override_logger.get_recent_logs()
        log_text.setText("\n".join([
            f"{log['timestamp']}: {log['reason']}"
            for log in recent_logs
        ]))
        
        layout.addWidget(log_text)
        return widget

    def _create_button_section(self, layout):
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
        cancel_button.clicked.connect(self.reject)
        
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
        save_button.clicked.connect(self.accept)
        
        button_layout.addWidget(cancel_button)
        button_layout.addWidget(save_button)
        layout.addWidget(button_container)

    def _update_toggle_button_style(self):
        self.settings_toggle_button.setStyleSheet(f"""
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
        self.settings_toggle_button.setText(
            "Accessibility O/X ON" if self.parent.accessibility_mode else "Accessibility O/X OFF"
        )
        self._update_toggle_button_style()
        self.parent.update_status_displays()

    def apply_theme_preview(self):
        """Apply theme changes in real-time"""
        is_dark = self.color_scheme.currentText() == "Dark"
        if is_dark != self.parent.colors.is_dark:
            self.parent.colors.is_dark = is_dark
            self.parent.colors.update_colors()
            self.parent.apply_theme()

    def accept(self):
        """Save settings and close dialog"""
        # Apply timing settings
        self.parent.override_duration = float(self.override_duration_spin.value())
        self.parent.dispense_cooldown = float(self.cooldown_time_spin.value())
        
        super().accept()

    def reject(self):
        """Restore original values and close dialog"""
        # Restore theme
        if self.parent.colors.is_dark != self.initial_settings['is_dark']:
            self.parent.colors.is_dark = self.initial_settings['is_dark']
            self.parent.colors.update_colors()
            self.parent.apply_theme()
        
        # Restore timing values
        self.parent.override_duration = self.initial_settings['override_duration']
        self.parent.dispense_cooldown = self.initial_settings['dispense_cooldown']
        
        # Restore accessibility mode
        if self.parent.accessibility_mode != self.initial_settings['accessibility_mode']:
            self.parent.accessibility_mode = self.initial_settings['accessibility_mode']
            self.parent.update_status_displays()
        
        super().reject() 