#    This file handles the launching of the karelics_vesc_can_driver ROS
#    node along with the underlying ros2_socketcan driver package.
#
#    Copyright (C) 2022  Karelics Oy
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    launch_list = []

    interface = "can0"
    receiver_interval_sec = "0.01"
    sender_timeout_sec = "0.01"

    emulate_tty = LaunchConfiguration("emulate_tty")

    motor_poles = LaunchConfiguration("motor_poles").perform(context)
    gear_ratio = LaunchConfiguration("gear_ratio").perform(context)

    emulate_tty_declare = DeclareLaunchArgument("emulate_tty", default_value="True")

    socket_can_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros2_socketcan"), "launch", "socket_can_receiver.launch.py"),
        ),
        launch_arguments=dict(
            interface=interface,
            interval_sec=receiver_interval_sec,
        ).items(),
    )

    socket_can_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros2_socketcan"), "launch", "socket_can_sender.launch.py"),
        ),
        launch_arguments=dict(
            interface=interface,
            timeout_sec=sender_timeout_sec,
        ).items(),
    )

    vesc_can_driver = Node(
        package="karelics_vesc_can_driver",
        executable="vesc_can_driver.py",
        name="karelics_vesc_can_driver",
        output="screen",
        emulate_tty=emulate_tty,
        parameters=[{"motor_poles": motor_poles, "gear_ratio": gear_ratio}],
    )

    battery_status = Node(
        package="karelics_vesc_can_driver",
        executable="battery_status.py",
        name="battery_status_node",
        output="screen",
        emulate_tty=emulate_tty,
        parameters=[],
    )

    launch_list += [
        emulate_tty_declare,
        socket_can_receiver,
        socket_can_sender,
        vesc_can_driver,
        battery_status,
    ]

    return launch_list


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("motor_poles"),
            DeclareLaunchArgument("gear_ratio"),
            OpaqueFunction(function=launch_setup),
        ]
    )
