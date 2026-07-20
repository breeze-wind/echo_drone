from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    hardware_launch = PathJoinSubstitution([
        FindPackageShare('robot_bring_up'), 'launch', 'hardware.launch.py'
    ])

    return LaunchDescription([
        LogInfo(msg='serial.launch.py is deprecated; use hardware.launch.py.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            launch_arguments={
                'use_mavros': 'false',
                'use_legacy_mavlink': 'false',
                'use_servo': 'false',
                'use_serial_manager': 'true',
                'dry_run': 'true',
            }.items(),
        ),
    ])
