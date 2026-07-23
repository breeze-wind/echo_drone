"""旧串口入口的兼容包装。

当前串口统一管理已经收敛到 `hardware.launch.py`。这个入口只保留给旧命令
兼容，启动时只运行 serial manager 的 dry-run，不启动 MAVROS、舵机或旧
pymavlink 节点。
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """转发到硬件层入口，并固定为只检查串口。"""
    hardware_launch = PathJoinSubstitution([
        FindPackageShare('robot_bring_up'), 'launch', 'hardware.launch.py'
    ])

    return LaunchDescription([
        LogInfo(msg='serial.launch.py 已废弃；请优先使用 hardware.launch.py。'),
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
