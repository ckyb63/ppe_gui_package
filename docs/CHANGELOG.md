# Changelog

## [Unreleased - Future Plans]

### Planned

- Main GUI Window to contain a live Camera Feed
  - Live Camera Feed to the Main GUI window to show the PPE detection with highlighted PPE items

## [Latest Release]

## [0.9.6] - 2025-03-11

### Notes

- Updated documentation.

## [0.9.5] - 2025-03-04

### Added

- Magnetic Lock pin to the Arduino ESP32 Code. (It is Pin D26)

## [0.9.3] - 2025-03-01

### Notes (Improved)

- Enhanced Error Handling in main_gui.py
- Improved ROS Thread management and Handling
- Improved Context Management
- Improced Markdown files formatting

## [0.9.2] - 2025-02-23

### Added

- Handlers for the settings, json, accessibility, and report files in the utils folder.

### Changed

- Refactored main_window.py to be more modular and easier to maintain.
  - Override Content
  - Help Content
  - Settings Content
  - Several Handler scripts as well
- Changed formatting of the dispensing log.
- Ensured all functions are working as expected.
- Ensured json files are being saved in the proper directory.

### Notes

- All scripts were reworked and is now being checked for any bugs and issues.

## [0.9.1] - 2025-02-22

### Added

- Readme file info.
- Launch file for the Avend Connector Node
- Configuration file for the Avend Connector Node
- Logger for the Avend Connector Node
- The temporary avend node is now complete and will be improved in future releases after testing with real hardware.
- Added a subscriber for the Camera Feed to the main GUI window, but it is not fully implemented.
- Added a graphical chart of the nodes and topic relationships to the README.md file.

### Notes

- The Avend Connector Node is not fully completed, the logger.py is untested.
- The Camera Feed is on hold until the hardware situation is resolved.
- The main_window.py file is undergoing the process of being refactored to be more modular and easier to maintain.

## [0.9.0] - 2025-02-21

### Added

- Added the node for the GUI to interface with the Avend Smart Vending API for Dispensing and Inventory Status. It will be fully implemented in the following releases.

## [0.8.3] - 2025-02-20

### Added

- Linked the Latest Release section from the CHANGELOG.md file in the README.md file.

### Changed

- Moved the location of the README.md file to the docs folder.
- Modified the formating and descriptions of the README.md file.
- Modified the help content text to be more concise and to the point.

### Fixed

- Fixed Versioning again, (MAJOR.Feature_Addition.Fix/Change/File_Update_Addition)
- Fixed changelog from now on to follow the format: Added, Changed, Fixed, Removed, Notes.
- Fixed Github Actions workflow.

## [0.8.2] - 2025-02-19

### Added

- New details to the README.md including a table of contents and a secton on provided nodes and launch files.

### Fixed

- Fixed main_gui launch file to shutdown the three nodes together when the main GUI is closed.
- Reverted some major but kept minor structure changes so the package isn't just completely broken.

## [0.8.1] - 2025-02-19

### Added

- Added a check to only send the command if it has changed to prevent spamming the ESP32 microcontroller.
- Added Proper LED to the ESP32 to better show the state of the gate.
- Added a Node to record the /pleaseDispense topic to a ROS2 bag file (.db3) for later analysis using services such as AWS S3.

### Fixed

- Fixed the issue where the ESP32 would not reset the gate if the command is the same as the current state.

## [0.8.0] - 2025-02-18

### Added

- Added Green and Red LED indicators to the ESP32 code to show the state of the gate.
- Added a report tab to the main GUI window to show the dispensing report.
- Added a Pi chart and a bar chart to show the recorded PPE dispensing events.
- Added a button to export the dispensing report as a .CSV file saved in the jsonSupport folder.
- Added a button to clear the stored dispensing report.
- Added related scripts for generating, and displaying a report on dispensing events, it is currently safed as a .json file; still working on getting the analytical data into a rosbag file for uploading to AWS eventualy.
- Added an info tab to the settings detailing each tab and its purpose.
- Preparatory work for subscribing a camera feed to the main GUI window, it is in sections.py > CameraSection class.

### Changed

- Improved overall styling of the Settings
- Settings tab now has numbers instead of long text labels.
- Updated screenshots in README.md

### Fixed

- Fixed safety glasses button display to update status, this was because it was originally just called glases. Previous update to safety glasses was not fully updated across the scripts.

### Removed

- Removed Camerafeed.py as that is outside the scope of this package.

## [0.7.3] - 2025-02-15

### Added

- Comments to the code and removed unused, and redundant code
- Detailed future plans to the CHANGELOG.md and additional details to the README.md

### Changed

- Revised the version numbers to follow regular versioning (MAJOR.MINOR.PATCH)

## [0.7.2] - 2025-02-14

### Added

- Added a skeleton script for the camera feed

### Changed

- How the .json files for the inventory and PPE status are saved.

### Fixed

- Fixed the inventory display to parse the "Safety Glasses" item correctly
- FIxed Help page's accessibility toggle overlapping with the OK button
- Fixed landscape mode to work properly

## [0.7.1] - 2025-02-14

### Added

- Added state logic to the ESP32 code.
- Added a open gate button to the settings override tab

## [0.7.0] - 2025-02-13

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

## [0.6.1] - 2025-02-12

### Added

- Added dynamic badges to the README.md
- Tested Github Actions workflow to update the version badge

## [0.6.0] - 2025-02-11

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

- Removed experimental_ppe_GUI.py and ppe_gui.py scripts as they are deprecated from the package.

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

- Enhanced override system with user detailed user and reason information logging
  - Added user selection dropdown for overrides
- Added reason tracking for override actions
- Enhanced override dialog UI with themed dropdowns
- Added validation for override user and reason selection

## [0.3.0] - 2025-02-07

### Added

- Integrated accessibility mode with O/X indicators beside the buttons
- Improved button readability with larger fonts and better spacing
- Added settings dialog
- Added memory to settings allowing for settings to be cancelled.
- Added theme handling in settings dialog: Dark Theme!
- Added proper window sizing and centering on startup
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
- GUI development switched to PyQt5 from Streamlit

## [0.1.0] - 2025-01-29

### Added

- Initial release
- Basic PPE vending machine GUI
- Basic status displays
- PPE dispensing controls
- GUI developed with Streamlit
