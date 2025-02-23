import os
import json
from datetime import datetime

class JsonHandler:
    def __init__(self, inventory_file, inventory_log_dir):
        self.inventory_file = inventory_file
        self.inventory_log_dir = inventory_log_dir

    def ensure_directory_exists(self):
        """Ensure the jsonSupport directory exists."""
        if not os.path.exists(self.inventory_log_dir):
            os.makedirs(self.inventory_log_dir)

    def get_default_inventory(self):
        """Get default inventory values"""
        return {
            'hardhat': '--',
            'beardnet': '--',
            'gloves': '--',
            'safetyglasses': '--',
            'earplugs': '--'
        }

    def load_inventory_data(self):
        """Load inventory data from file"""
        try:
            if os.path.exists(self.inventory_file):
                with open(self.inventory_file, 'r') as f:
                    data = json.load(f)
                    last_update = data.get('last_update', "Never")
                    if last_update != "Never" and 'T' in last_update:
                        try:
                            dt = datetime.fromisoformat(last_update)
                            last_update = dt.strftime("%Y-%m-%d %H:%M:%S")
                        except:
                            last_update = "Never"
                    
                    return {
                        'inventory': data.get('inventory', self.get_default_inventory()),
                        'last_update': last_update
                    }
        except Exception as e:
            print(f"Error loading inventory data: {e}")
        
        return {
            'inventory': self.get_default_inventory(),
            'last_update': "Never"
        }

    def save_inventory_data(self, inventory, last_update):
        """Save inventory data to file"""
        try:
            data = {
                'inventory': inventory,
                'last_update': last_update
            }
            with open(self.inventory_file, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print(f"Error saving inventory data: {e}")

    def write_to_json(self, filename, data):
        """Write data to a JSON file in the log directory"""
        try:
            file_path = os.path.join(self.inventory_log_dir, filename)
            
            # Ensure proper structure for dispensing log
            if filename == "dispensing_log.json":
                if not isinstance(data, dict):
                    data = {"events": []}
                elif "events" not in data:
                    data["events"] = []
                    
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=2)
            return True
        except Exception as e:
            print(f"Error writing to JSON file {filename}: {e}")
            return False
            
    def append_to_json(self, filename, new_data):
        """Append data to a JSON file's events array"""
        try:
            file_path = os.path.join(self.inventory_log_dir, filename)
            
            # Read existing data or create new structure
            if os.path.exists(file_path):
                with open(file_path, 'r') as f:
                    try:
                        data = json.load(f)
                    except json.JSONDecodeError:
                        data = {"events": []}
            else:
                data = {"events": []}
            
            # Ensure data has the correct structure
            if not isinstance(data, dict):
                data = {"events": []}
            if "events" not in data:
                data["events"] = []
            
            # Append new data to events array
            data["events"].append(new_data)
            
            # Write back to file
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=2)
            return True
        except Exception as e:
            print(f"Error appending to JSON file {filename}: {e}")
            return False 