import json
import os
from collections import Counter

def generate_report():
    """Generate a report of dispensing events."""
    log_file_path = os.path.join(os.getcwd(), "src", "ppe_gui_package", "gui_package", "main_gui_modules", "jsonSupport", "dispensing_log.json")
    
    # Check if the log file exists
    if not os.path.exists(log_file_path):
        return {"error": "No dispensing events recorded.", "data": {}, "events": []}

    # Try to read the log file
    try:
        with open(log_file_path, 'r') as log_file:
            events = json.load(log_file)  # Load the entire JSON array
            print(f"Loaded events: {events}")  # Debugging output
    except json.JSONDecodeError:
        return {"error": "Error reading dispensing log. The file may be corrupted.", "data": {}, "events": []}
    except Exception as e:
        return {"error": f"An error occurred: {str(e)}", "data": {}, "events": []}

    # Count items dispensed
    item_counts = Counter(event['item'] for event in events)
    
    # Generate report as a string
    report = "PPE Dispensing Report:\n"
    for item, count in item_counts.items():
        report += f"{item}: {count} times\n"
    
    return {
        "error": None,
        "data": item_counts,
        "report": report,
        "events": events  # Return the events to access timestamps
    }