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
- Python 3.8+
- PyQt5
- rclpy
- std_msgs

## Installation

1. Clone this package into your ROS2 workspace's src directory:
```bash
cd ~/ros2_ws/src
git clone <repository-url>/gui_package.git
```

2. Install dependencies:
```bash
sudo apt-get update
sudo apt-get install python3-pyqt5
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select gui_package
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

1. Launch the GUI:
```bash
ros2 run gui_package ppe_gui
```

2. For testing without hardware, run the dummy PPE status publisher:
```bash
ros2 run gui_package dummy_ppe
```

## ROS2 Topics

### Subscribed Topics
- `ppe_status` (std_msgs/String): Receives PPE detection status
  - Format: "item1:true, item2:false, ..."

### Published Topics
- `pleaseDispense` (std_msgs/String): Sends dispense requests
  - Values: "hardhat", "beardnet", "gloves", "glasses", "earplugs", "OVERRIDE"
- `gate` (std_msgs/Bool): Controls safety gate status
  - true = locked, false = unlocked

## Development

### File Structure
- `ppe_gui.py`: Main GUI implementation
- `dummy_ppe_status.py`: Test publisher for simulating PPE detection

### Building from Source
```bash
cd ~/ros2_ws
colcon build --packages-select gui_package --symlink-install
```

## License

Apache License 2.0

## Authors

- Max Chen 