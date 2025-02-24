#!/usr/bin/env python3

"""
Override logging functionality

This module provides a logger that overrides the default logging functionality.
It allows for easy logging of overrides to a file.

Author: Max Chen
"""
import os
import json
from datetime import datetime

class OverrideLogger:
    def __init__(self, log_dir="jsonSupport", log_file="override_log.json"):
        # Use the current working directory to construct the path
        self.log_dir = os.path.join(os.getcwd(), "src", "ppe_gui_package", "gui_package", "main_gui_modules", log_dir)
        self.log_file = os.path.join(self.log_dir, log_file)
        self.logs = self._load_logs()
        self._ensure_log_directory_exists()
        
    def _ensure_log_directory_exists(self):
        """Ensure the log directory exists."""
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
    # Load the logs from a JSON file
    def _load_logs(self):
        if os.path.exists(self.log_file):
            try:
                with open(self.log_file, 'r') as f:
                    return json.load(f)
            except:
                return []
        return []
        
    # Log an override with standardized timestamp to a JSON file
    def log_override(self, reason=""):
        """Log an override with standardized timestamp"""
        log_entry = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "reason": reason
        }
        self.logs.append(log_entry)
        self._save_logs()
        
    # Save the logs to a JSON file
    def _save_logs(self):
        with open(self.log_file, 'w') as f:
            json.dump(self.logs, f, indent=2)
            
    # Get the recent logs
    def get_recent_logs(self, limit=10):
        return self.logs[-limit:] 