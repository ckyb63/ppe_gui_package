import csv
import json
from datetime import datetime
from ..jsonSupport.reporting import generate_report
import os

class ReportHandler:
    @staticmethod
    def clear_dispensing_log():
        """Clear the dispensing log file"""
        try:
            with open('dispensing_log.json', 'w') as f:
                json.dump({"events": [], "data": {}}, f)
            return True, "Dispensing log cleared"
        except Exception as e:
            return False, f"Error clearing log: {e}"

    def export_report_to_csv(self, csv_path):
        """Export the dispensing log to CSV format"""
        try:
            # Generate report data
            report_data = generate_report()
            
            if not report_data or not report_data.get("events"):
                return False, "No data to export"

            # Write to CSV file
            with open(csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Write headers
                writer.writerow(['Item', 'Timestamp'])
                # Write data
                for event in report_data["events"]:
                    writer.writerow([event['item'], event['timestamp']])
                    
            return True, f"Report exported successfully to {os.path.basename(csv_path)}"
        except Exception as e:
            return False, f"Failed to export report: {str(e)}" 