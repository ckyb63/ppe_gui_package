^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gui_package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.1 (2025-02-11)
------------------
- Overall Fixes and UI Improvements
- Fixed inventory timestamp display to standard format
- Improved inventory table styling
- Added proper inventory update handling
- Added inventory memory
- Increased overall UI sizes for better readability and touch target sizes 
- Added dummy launch file to launch both included dummy nodes


0.5.0 (2025-02-10)
------------------
- Modular code architecture with improved maintainability
- Integrated inventory management system
- Real-time inventory tracking and updates
- Inventory request and response system
- Improved settings interface with tabbed organization
- Improved override system with user tracking and reason logging
- Improved override content UI


0.3.1 (2025-02-07)
------------------
* Enhanced override system with user authentication
* Added user selection dropdown for overrides
* Added reason tracking for override actions
* Improved override logging with detailed user and reason information
* Enhanced override dialog UI with themed dropdowns
* Added validation for override user and reason selection
* Fixed Errors

0.3.0 (2025-02-07)
------------------
* Added integrated accessibility mode with O/X indicators in buttons
* Improved button readability with larger fonts and better spacing
* Fixed status message behavior for dispensing and override states
* Added Settings
* Added cancellable settings 
* Added proper theme handling in settings dialog: Dark Theme!
* Fixed dialog positioning and content switching
* Improved error handling and widget cleanup
* Switched from dedicated Dialog windows to using the main GUI window
* Added proper window sizing and centering
* Added proper settings cancellation and state restoration
* Added Dark Theme!
* Added Override Logging
* Reworked Help Dialog
* Stable GUI Updated to 0.2.0

0.2.1 (2025-02-05)
------------------
* Fixed window resizing issues
* Improved status message display
* Added error handling for ROS communications
* Enhanced thread safety in GUI updates

0.2.0 (2025-02-01)
------------------
* Added experimental GUI with adaptive layout
* Implemented portrait/landscape mode switching
* Added thread-safe GUI updates
* Enhanced status displays with color feedback
* Added help system with accessibility options
* Added configurable window sizing
* Implemented responsive camera feed placeholder
* Added basic accessibility mode with O/X indicators for color-blind users located beside the buttons
* GUI Development switched to PyQt5

0.1.0 (2025-01-29)
------------------
* Initial release
* Basic PPE vending machine GUI
* ROS2 integration for status monitoring
* Safety gate control system
* Administrative override functionality
* Basic status displays
* PPE dispensing controls 
* Streamlit GUI