# Experimental PPE GUI Module

This is a modular implementation of the PPE Vending Machine GUI. The code has been organized into the following structure:

```
experimental/
├── __init__.py
├── main.py
├── main_window.py
├── ros_node.py
├── utils/
│   ├── __init__.py
│   ├── colors.py
│   ├── context.py
│   └── logger.py
└── widgets/
    ├── __init__.py
    ├── buttons.py
    ├── dialogs.py
    ├── override_dialog.py
    ├── sections.py
    └── settings_dialog.py
```

## Module Structure

- `main.py`: Entry point and application setup
- `main_window.py`: Main window implementation
- `ros_node.py`: ROS2 node implementation

### Utils
- `colors.py`: Color scheme management
- `context.py`: ROS context management
- `logger.py`: Override logging functionality

### Widgets
- `buttons.py`: Custom button implementations
- `dialogs.py`: Help dialog implementation
- `override_dialog.py`: Override confirmation dialog
- `sections.py`: Main GUI section widgets
- `settings_dialog.py`: Settings dialog implementation

## Usage

Run the experimental GUI with:
```bash
ros2 run gui_package experimental_gui
```

Or use the launch file to start both the GUI and dummy PPE publisher:
```bash
ros2 launch gui_package experimental_gui.launch.py
``` 