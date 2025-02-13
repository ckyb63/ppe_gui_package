# Changelog

## [Unreleased]
### Planned
- Camera Feed to the Main GUI window for a Live feed of the user in front of the vending machine
- Upgrade and migrate from PyQt5 to PyQt6
- Scripts cleanup
- Remove window control buttons from the GUI

## [0.5.7] - 2025-02-14
### Added
- Added state logic to the ESP32 code. 
- Added a open gate button to the settings override tab

## [0.5.6] - 2025-02-13
### Added
- Added Safety Gate Controller to the package
- Added ESP32 Bluetooth Communication Code to the package

### Changed
- Improved the overall styling of each tab in the settings dialog for better readability and touch targets

### Fixed
- Fixed the override log display in the settings dialog
- Fixed the styling issues with the settings dialog
- Fixed the ROS2 shutdown issue in the dummy nodes
- Python version and inclusion in scripts

## [0.5.1.2] - 2025-02-12
### Added
- Added dynamic badges to the README.md
- Tested Github Actions workflow to update the version badge   

## [0.5.1] - 2025-02-11
### Added
- Overall fixes and UI improvements
- Added proper inventory update handling
- Added inventory memory
- Added dummy launch file to launch both included dummy nodes

### Changed
- Fixed inventory timestamp display to standard format
- Improved inventory table styling
- Increased overall UI sizes for better readability and touch target sizes
- Moved dummy nodes to dummy_test folder
- Updated screenshots in README.md

### Removed
- Removed experimental_ppe_GUI.py and ppe_gui.py scripts as they are deprecated from the package

### Fixed
- Fixed inventory timestamp display to standard format
- Improved inventory table styling
- Increased overall UI sizes for better readability and touch target sizes
- Fixed ROS shutdown issue in dummy nodes

## [0.5.0] - 2025-02-10
### Added
- Modular code architecture with improved maintainability
- Integrated inventory management system
- Real-time inventory tracking and updates
- Inventory request and response system
- Improved settings interface with tabbed organization

### Changed
- Improved override system with user tracking and reason logging
- Improved override content UI

## [0.3.1] - 2025-02-07
### Added
- Enhanced override system with user authentication
- Added user selection dropdown for overrides
- Added reason tracking for override actions
- Enhanced override dialog UI with themed dropdowns
- Added validation for override user and reason selection

### Fixed
- Improved override logging with detailed user and reason information

## [0.3.0] - 2025-02-07
### Added
- Integrated accessibility mode with O/X indicators in buttons
- Improved button readability with larger fonts and better spacing
- Added settings
- Added cancellable settings
- Added proper theme handling in settings dialog: Dark Theme!
- Added proper window sizing and centering
- Added proper settings cancellation and state restoration
- Added dark theme!
- Added override logging
- Reworked help dialog

### Changed
- Switched from dedicated dialog windows to using the main GUI window
- Improved error handling and widget cleanup

## [0.2.1] - 2025-02-05
### Fixed
- Fixed window resizing issues
- Improved status message display
- Added error handling for ROS communications
- Enhanced thread safety in GUI updates

## [0.2.0] - 2025-02-01
### Added
- Added experimental GUI with adaptive layout
- Implemented portrait/landscape mode switching
- Added thread-safe GUI updates
- Enhanced status displays with color feedback
- Added help system with accessibility options
- Added configurable window sizing
- Implemented responsive camera feed placeholder
- Added basic accessibility mode with O/X indicators for color-blind users located beside the buttons
- GUI development switched to PyQt5

## [0.1.0] - 2025-01-29
### Added
- Initial release
- Basic PPE vending machine GUI
- ROS2 integration for status monitoring
- Safety gate control system
- Administrative override functionality
- Basic status displays
- PPE dispensing controls
- Streamlit GUI