# GUI Package for PPE Vending Machine

Version: 0.2.0

A ROS2 package containing a PyQt5-based graphical user interface for controlling and monitoring a PPE (Personal Protective Equipment) vending machine.

## Features

### Core Features
- Real-time PPE detection status monitoring
- Automated safety gate control
- ROS2 integration for vending machine control
- Administrative override system
- Simulation support for testing

### New in v0.2.0
- Adaptive layout switching (portrait/landscape)
- Thread-safe GUI updates
- Enhanced status displays with color feedback
- Improved button styling and feedback
- Help system with context-sensitive information
- Configurable window sizing
- Responsive camera feed placeholder

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

2. Clone this package into the workspace src directory:
```bash
cd src
git clone https://github.com/ckyb63/ppe_gui_package.git
```

3. Install dependencies:
```bash
sudo apt update
sudo apt install python3-pyqt5
```

4. Build the workspace:
```bash
cd ~/ros2_ws
colcon build
```

5. Source the workspace:
```bash
source install/setup.bash
```

## Usage

First, source your workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

### Running the GUI

1. Launch the standard GUI:
```bash
ros2 run gui_package ppe_gui
```

2. Launch the experimental GUI with adaptive layout:
```bash
ros2 run gui_package experimental_gui
```

3. For testing without hardware, run the dummy PPE status publisher:
```bash
ros2 run gui_package dummy_ppe
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
│   ├── ppe_gui.py              # Standard GUI implementation
│   ├── experimental_ppe_gui.py # Adaptive layout GUI implementation
│   └── dummy_ppe_status.py     # Test publisher for simulating PPE detection
├── package.xml                 # Package manifest
├── setup.py                   # Package setup
└── README.md
```

### Building for Development
```bash
cd ~/ros2_ws
colcon build --packages-select gui_package --symlink-install
```

## Features in Detail

### Adaptive Layout
- Portrait mode: Traditional vertical layout
- Landscape mode: Side-by-side controls and camera feed
- Automatic switching based on window dimensions
- Maintains proper spacing and proportions

### Status Display
- Color-coded buttons indicate PPE detection status
- Large, clear gate status display
- Countdown timer for override mode
- Status messages with auto-reset

### Thread Safety
- ROS communications handled in separate thread
- Qt updates properly marshalled to GUI thread
- Clean shutdown handling
- Proper resource cleanup

## Author

- Max Chen
- Email: ckyb63@gmail.com
