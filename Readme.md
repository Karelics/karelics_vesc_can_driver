## Licensing
This piece of software is released under the GPL 3.0 license.

## Prerequisites
- This driver requires a working socket can setup so make sure you have one.
- This driver relies on the `ros2_socketcan` package, so make sure you clone the latest stable version of it into your workspace from this link: https://github.com/autowarefoundation/ros2_socketcan
- Make sure your VESC is configured for CAN communication.

## Installation
1. Clone this repository to your workspace https://github.com/Karelics/karelics_vesc_can_driver
2. Run rosdep to install dependencies`rosdep install --from-paths src --ignore-src -r -y`
3. Build and source your workspace
4. Run the can setup script `./setup_can_interface.bash ` or setup you socket can interface manually.
5. Launch the package while rememberin that this package requires specifying what is the gear ratio of the motor mounting system on your robot and the number of poles for the motor/motors that you are using. Launch command example: `ros2 launch karelics_vesc_can_driver karelics_vesc_can_driver.launch.py motor_poles:=8 gear_ratio:=1.0`

## Notes
1. The can setup script assumes a CAN speed of 1 Mbps and the can0 interface. If you have a different setup change the setup script.

