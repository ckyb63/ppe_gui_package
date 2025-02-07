# GUI Package for PPE Vending Machine

## Version 0.3.0

A ROS2 package containing a PyQt5-based graphical user interface for controlling and monitoring a PPE (Personal Protective Equipment) vending machine.

## Features

### Core Features
- Real-time PPE detection status monitoring
- Automated safety gate control
- ROS2 integration for vending machine control
- Administrative override system
- Simulation support for testing

### Latest Features (v0.3.0)
- Integrated accessibility mode with O/X indicators directly in buttons
- Added Settings which allows you to change the theme, font size, and more!
- Improved button readability with larger fonts
- Enhanced status message system
- Proper theme handling and settings management
- Added Dark Theme!
- Fixed dialog positioning and content switching
- Override Logging
- Improved Button Styling for touchscreen

### Accessibility Features
- Toggle for O/X status indicators
- Clear text labels
- Consistent button sizing
- Screen reader friendly layout

## Dependencies

- ROS2 Humble
- Python 3.10+
- PyQt5 5.15+
- rclpy
- std_msgs

## Installation

1. Create a ROS2 workspace (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. Clone this package:
```bash
cd src
git clone https://github.com/ckyb63/ppe_gui_package.git
```

3. Install dependencies:
```bash
sudo apt update
sudo apt install python3-pyqt5
```

4. Build and source:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage

### Running the GUI

```bash
# Launch the experimental GUI (recommended)
ros2 run gui_package experimental_gui

# For testing without hardware
ros2 run gui_package dummy_ppe

# Launch the standard Stable GUI
ros2 run gui_package ppe_gui
```

## ROS2 Topics

### Subscribed Topics
- `ppe_status` (std_msgs/String): Receives PPE detection status
  - Format: "hardhat:true, beardnet:false, gloves:true, glasses:true, earplugs:false"

### Published Topics
- `pleaseDispense` (std_msgs/String): Sends dispense requests
  - Values: "hardhat", "beardnet", "gloves", "glasses", "earplugs", "OVERRIDE"
- `gate` (std_msgs/Bool): Controls safety gate status
  - true = locked, false = unlocked

## Development

### File Structure
```
gui_package/
├── gui_package/
│   ├── __init__.py
│   ├── experimental_ppe_gui.py # Main GUI implementation
│   └── dummy_ppe_status.py     # Test publisher
├── package.xml
├── setup.py
└── README.md
```

### Building for Development
```bash
cd ~/ros2_ws
colcon build --packages-select gui_package --symlink-install
```

## Author

- Max Chen
- Email: ckyb63@gmail.com