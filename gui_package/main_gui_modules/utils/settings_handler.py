class SettingsHandler:
    def __init__(self, main_window):
        self.main_window = main_window

    def apply_theme_preview(self, settings, colors):
        """Apply theme changes in real-time"""
        if settings:
            colors_tab = settings.tabs.widget(1)  # Colors tab
            is_dark = colors_tab.color_scheme_combo.currentText() == "Dark"
            if is_dark != colors.is_dark:
                colors.is_dark = is_dark
                colors.update_colors()
                return True
        return False

    def handle_settings_save(self, settings, colors, ros_node, override_logger):
        """Save settings and return to main view"""
        colors_tab = settings.tabs.widget(1)
        timing_tab = settings.tabs.widget(3)
        override_tab = settings.tabs.widget(4)
        
        # Apply color scheme
        is_dark = colors_tab.color_scheme_combo.currentText() == "Dark"
        theme_changed = is_dark != colors.is_dark
        if theme_changed:
            colors.is_dark = is_dark
            colors.update_colors()
        
        # Get timing settings
        override_duration = float(timing_tab.override_duration_spin.value())
        dispense_cooldown = float(timing_tab.cooldown_time_spin.value())
        
        # Handle gate override
        gate_status = None
        if hasattr(override_tab, 'temp_gate_override_button'):
            is_unlocked = override_tab.temp_gate_override_button.isChecked()
            if is_unlocked != (not self.main_window.safety_gate_locked):
                gate_status = not is_unlocked
                ros_node.publish_gate_status(gate_status)
                action = "unlocked" if is_unlocked else "locked"
                override_logger.log_override(
                    f"User: Admin - Reason: Gate manually {action} from settings"
                )
        
        return {
            'theme_changed': theme_changed,
            'override_duration': override_duration,
            'dispense_cooldown': dispense_cooldown,
            'gate_status': gate_status
        } 