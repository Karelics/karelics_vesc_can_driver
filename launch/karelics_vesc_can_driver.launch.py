import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_PATH = get_package_share_directory("karelics_vesc_can_driver")
KARELICS_ROBOT = os.environ['KARELICS_ROBOT']


def generate_launch_description():
    interface = LaunchConfiguration('interface')
    receiver_interval_sec = LaunchConfiguration('receiver_interval_sec')
    sender_timeout_sec = LaunchConfiguration('sender_timeout_sec')
    emulate_tty = LaunchConfiguration('emulate_tty')

    if KARELICS_ROBOT == 'sampo':
        motor_poles = '8'
        gear_ratio = '1'
        continuous_current_limit = '120'
    elif KARELICS_ROBOT == 'antero':
        motor_poles = '14'
        gear_ratio = '5.846'
        continuous_current_limit = '120'
    else:
        motor_poles = '14'
        gear_ratio = '5.846'
        continuous_current_limit = '120'

    return LaunchDescription([
        DeclareLaunchArgument(
            'motor_poles',
            default_value=motor_poles
        ),
        DeclareLaunchArgument(
            'gear_ratio',
            default_value=gear_ratio
        ),
        DeclareLaunchArgument(
            'continuous_current_limit',
            default_value=continuous_current_limit
        ),
        DeclareLaunchArgument(
            'interface',
            default_value='can0'
        ),
        DeclareLaunchArgument(
            'receiver_interval_sec',
            default_value='0.01'
        ),
        DeclareLaunchArgument(
            'sender_timeout_sec',
            default_value='0.01'
        ),
        DeclareLaunchArgument(
            'emulate_tty',
            default_value='true'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros2_socketcan'), 'launch', 'socket_can_receiver.launch.py'),
            ),
            launch_arguments={'interface': interface,
                              'interval_sec': receiver_interval_sec}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros2_socketcan'), 'launch', 'socket_can_sender.launch.py'),
            ),
            launch_arguments={'interface': interface,
                              'timeout_sec': sender_timeout_sec}.items()
        ),
        Node(
            package='karelics_vesc_can_driver',
            executable='vesc_can_driver.py',
            name='karelics_vesc_can_driver',
            output='screen',
            emulate_tty=emulate_tty,
            parameters=[{'motor_poles': motor_poles,
                         'gear_ratio': gear_ratio,
                         'continuous_current_limit': continuous_current_limit}]
        )
    ])
