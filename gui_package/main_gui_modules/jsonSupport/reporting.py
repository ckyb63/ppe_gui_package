import json
import os
from collections import Counter

def generate_report():
    """Generate a report from the dispensing log"""
    try:
        # Get the log file path using the same path as JsonHandler
        log_file = os.path.join(os.getcwd(), "src", "ppe_gui_package", "gui_package", 
                               "main_gui_modules", "jsonSupport", "dispensing_log.json")
        print(f"Looking for log file at: {log_file}")  # Debug print
        
        if not os.path.exists(log_file):
            print("Log file not found")  # Debug print
            #return {"events": [], "data": {}}
            
        with open(log_file, 'r') as f:
            data = json.load(f)
            #print(f"Loaded data: {data}")  # Debug print
            
        if not isinstance(data, dict) or "events" not in data:
            print("Invalid data structure")  # Debug print
            #return {"events": [], "data": {}}
            
        events = data.get("events", [])
        #print(f"Events found: {events}")  # Debug print
        
        if not events:  # If events is empty
            print("No events found")  # Debug print
            #return {"events": [], "data": {}}
            
        # Count items
        item_counts = Counter()
        for event in events:
            if isinstance(event, dict) and "item" in event:
                if event["item"] != "OVERRIDE":  # Don't count OVERRIDE events
                    item_counts[event["item"]] += 1
                
        result = {
            "events": events,
            "data": dict(item_counts)
        }
        #print(f"Generated report: {result}")  # Debug print
        return result
    except Exception as e:
        print(f"Error generating report: {e}")  # Debug print
        #return {"events": [], "data": {}}