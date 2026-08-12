# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """Return the behavior-control launch description."""
    default_yaml_path = os.path.join(
        get_package_share_directory('robot_bring_up'),
        'config',
        'drone.yaml'
    )

    yaml_path = LaunchConfiguration(
        'params_file',
        default=default_yaml_path
    )
    declare_yaml_path = DeclareLaunchArgument(
        'params_file',
        default_value=yaml_path,
        description='Full path to the pcd2pgm configuration file to load'
    )
    dry_run = LaunchConfiguration('dry_run')
    autostart = LaunchConfiguration('autostart')
    start_state = LaunchConfiguration('start_state')
    declare_dry_run = DeclareLaunchArgument(
        'dry_run', default_value='false',
        description='安全调试模式：禁止飞控、导航和舵机任务命令输出')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='是否启动后立即运行；false 时等待 /mission/start')
    declare_start_state = DeclareLaunchArgument(
        'start_state', default_value='legacy_default',
        description='/mission/start 使用的命名状态或旧数字 step')

    node_01 = Node(
        package="behavior_control",
        executable="behavior_control_node",
        output="screen",
        parameters=[yaml_path, {
            'dry_run': ParameterValue(dry_run, value_type=bool),
            'autostart': ParameterValue(autostart, value_type=bool),
            'start_state': ParameterValue(start_state, value_type=str),
        }],
        name="behavior_control_node",
        respawn=True  # 重启
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [declare_yaml_path, declare_dry_run, declare_autostart,
         declare_start_state, node_01]
    )
    # 返回让ROS2根据launch描述执行节点
    return launch_description
