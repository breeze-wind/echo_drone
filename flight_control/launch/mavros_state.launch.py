"""启动最小 MAVROS 节点，用于检查飞控状态和话题。

当前工作区的 Foxy 环境不能稳定解析上游 MAVROS XML launch 文件，因为其中
仍有旧 launch 替换语法。这个 Python 包装只传入 PX4 心跳检查所需的串口
URL、目标 ID、协议参数和本包内的最小 PX4 vision 插件配置，直接启动
`mavros_node`。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mavros_config_file = LaunchConfiguration('config_file')

    # 使用最小 PX4 vision 配置，避免默认加载所有 MAVROS extras。
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        respawn=False,
        parameters=[
            mavros_config_file,
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'target_system_id': ParameterValue(
                    LaunchConfiguration('tgt_system'),
                    value_type=int,
                ),
                'target_component_id': ParameterValue(
                    LaunchConfiguration('tgt_component'),
                    value_type=int,
                ),
                'fcu_protocol': LaunchConfiguration('fcu_protocol'),
                'gcs_url': LaunchConfiguration('gcs_url'),
            },
        ],
    )

    return LaunchDescription([
        # 默认支持 udev 管理的飞控链接；WSL/USBIP 下 udev 可能未生效，
        # 所以也允许直接使用 /dev/ttyACM0。
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyACM0:230400',
            description='MAVROS 连接飞控的串口 URL。',
        ),
        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='MAVROS 目标系统 ID。',
        ),
        DeclareLaunchArgument(
            'tgt_component',
            default_value='1',
            description='MAVROS 目标组件 ID。',
        ),
        DeclareLaunchArgument(
            'fcu_protocol',
            default_value='v2.0',
            description='MAVROS 使用的 MAVLink 协议版本。',
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='MAVROS router 转发给 QGC/GCS 的 MAVLink URL，默认关闭。',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('flight_control'), 'config',
                'mavros_vision_px4.yaml',
            ]),
            description='MAVROS PX4 插件和坐标系配置文件。',
        ),
        mavros_node,
    ])
