# GUI Package for PPE Vending Machine

A ROS2 package containing a PyQt5-based graphical user interface for controlling and monitoring a PPE (Personal Protective Equipment) vending machine.

## Features

- Real-time PPE detection status monitoring
- Automated safety gate control
- ROS2 integration for vending machine control
- Administrative override system
- Simulation support for testing

## Dependencies

- ROS2 (tested on Humble)
- Python 3.10+
- PyQt5
- rclpy
- std_msgs

## Installation

1. Create a ROS2 workspace (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. Clone this package into the src directory:
```bash
cd src
git clone https://github.com/maxckyuen/gui_package.git
```

3. Install dependencies:
```bash
sudo apt update
sudo apt install python3-pyqt5
```

4. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select gui_package
```

5. Source the workspace:
```bash
source install/setup.bash
```

## Usage

1. Launch the GUI:
```bash
ros2 run gui_package ppe_gui
```

2. For testing without hardware, run the dummy PPE status publisher in another terminal:
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
│   ├── ppe_gui.py          # Main GUI implementation
│   └── dummy_ppe_status.py # Test publisher for simulating PPE detection
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