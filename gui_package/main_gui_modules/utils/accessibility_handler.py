from ..widgets.settings import SettingsContent

class AccessibilityHandler:
    def __init__(self, main_window):
        self.main_window = main_window
        self.accessibility_mode = False

    def toggle_accessibility_mode(self):
        """Toggle accessibility mode and update UI"""
        self.accessibility_mode = not self.accessibility_mode
        self._sync_accessibility_buttons()
        return self.accessibility_mode

    def _sync_accessibility_buttons(self):
        """Sync all accessibility-related button states"""
        settings = self.main_window.findChild(SettingsContent)
        if settings:
            colors_tab = settings.tabs.widget(1)  # Colors tab
            if hasattr(colors_tab, 'settings_toggle_button'):
                colors_tab.settings_toggle_button.setChecked(self.accessibility_mode)
                self._update_settings_toggle_button_style(colors_tab)

    def _update_settings_toggle_button_style(self, colors_tab):
        """Update settings toggle button styling"""
        colors_tab.settings_toggle_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {self.main_window.colors.success if self.accessibility_mode else self.main_window.colors.neutral};
                color: white;
                border: none;
                border-radius: 10px;
            }}
            QPushButton:hover {{
                background-color: {self.main_window.colors.success if self.accessibility_mode else '#555'};
            }}
        """) 