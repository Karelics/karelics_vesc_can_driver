# Quick start
This is a quickstart guide, for more information look a the documentation in the doc directory.

## Prerequisites
- This driver requires a working socket can setup make sure you have one.
- Make sure you VESC is setup for CAN communication.

## Installation
1. Clone this repository to your workspace https://github.com/Karelics/karelics_vesc_can_driver
2. Run rosdep to install dependencies`rosdep install --from-paths src --ignore-src -r -y`
3. Build and source your workspace
4. Run the can setup script `rosrun karelics_vesc_can_driver setup_can_interface.bash ` or setup you socket can interface manually.
5. Launch the hardware_interface `roslaunch karelics_vesc_can_driver hardware_interface.launch`



## Notes
1. The can setup script assumes a CAN speed of 500kbps and the can0 interface. If you have a different setup change the setup script.
2. To build the documentation use `rosdoc_lite .`

