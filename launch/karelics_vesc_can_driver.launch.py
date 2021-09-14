import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    launch_list = []

    interface = LaunchConfiguration('interface')
    receiver_interval_sec = LaunchConfiguration('receiver_interval_sec')
    sender_timeout_sec = LaunchConfiguration('sender_timeout_sec')
    emulate_tty = LaunchConfiguration('emulate_tty')

    motor_poles = LaunchConfiguration('motor_poles').perform(context)
    gear_ratio = LaunchConfiguration('gear_ratio').perform(context)
    continuous_current_limit = LaunchConfiguration('continuous_current_limit').perform(context)

    interface_declare = DeclareLaunchArgument(
        'interface',
        default_value='can0',
        description='The name of the can interface')

    receiver_interval_sec_declare = DeclareLaunchArgument(
        'receiver_interval_sec',
        default_value='0.01')

    sender_timeout_sec_declare = DeclareLaunchArgument(
        'sender_timeout_sec',
        default_value='0.01')

    emulate_tty_declare = DeclareLaunchArgument(
        'emulate_tty',
        default_value='True')

    socket_can_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_socketcan'), 'launch', 'socket_can_receiver.launch.py'),
        ),
        launch_arguments=dict(
            interface=interface,
            interval_sec=receiver_interval_sec,
        ).items()
    )

    socket_can_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_socketcan'), 'launch', 'socket_can_sender.launch.py'),
        ),
        launch_arguments=dict(
            interface=interface,
            timeout_sec=sender_timeout_sec,
        ).items()
    )

    vesc_can_driver = Node(
        package='karelics_vesc_can_driver',
        executable='vesc_can_driver.py',
        name='karelics_vesc_can_driver',
        output='screen',
        emulate_tty=emulate_tty,
        parameters=[{'motor_poles': motor_poles,
                     'gear_ratio': gear_ratio,
                     'continuous_current_limit': continuous_current_limit}],
    )

    launch_list += [
        interface_declare,
        receiver_interval_sec_declare,
        sender_timeout_sec_declare,
        emulate_tty_declare,
        socket_can_receiver,
        socket_can_sender,
        vesc_can_driver,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('motor_poles', default_value='0'),
        DeclareLaunchArgument('gear_ratio', default_value='0.0'),
        DeclareLaunchArgument('continuous_current_limit', default_value='0'),
        OpaqueFunction(function=launch_setup),
    ])
