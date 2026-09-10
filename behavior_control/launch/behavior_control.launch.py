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
    navigation_execution_mode = LaunchConfiguration('navigation_execution_mode')
    sls_nav_goal_topic = LaunchConfiguration('sls_nav_goal_topic')
    takeoff_circle_enabled = LaunchConfiguration('takeoff_circle_enabled')
    declare_dry_run = DeclareLaunchArgument(
        'dry_run', default_value='false',
        description='安全调试模式：禁止飞控、导航和舵机任务命令输出')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='是否启动后立即运行；false 时等待 /mission/start')
    declare_start_state = DeclareLaunchArgument(
        'start_state', default_value='legacy_default',
        description='/mission/start 使用的命名状态或旧数字 step')
    declare_navigation_execution_mode = DeclareLaunchArgument(
        'navigation_execution_mode', default_value='nav2',
        description='nav2 使用 NavigateToPose；sls_goal 只把最终目标发送给 SLS。')
    declare_sls_nav_goal_topic = DeclareLaunchArgument(
        'sls_nav_goal_topic', default_value='/sls_circle/nav_goal',
        description='navigation_execution_mode=sls_goal 时的 SLS 最终目标话题。')
    declare_takeoff_circle_enabled = DeclareLaunchArgument(
        'takeoff_circle_enabled', default_value='true',
        description='false 时进入完整任务流程；true 时起飞后保持圆周调试。')

    node_01 = Node(
        package="behavior_control",
        executable="behavior_control_node",
        output="screen",
        parameters=[yaml_path, {
            'dry_run': ParameterValue(dry_run, value_type=bool),
            'autostart': ParameterValue(autostart, value_type=bool),
            'start_state': ParameterValue(start_state, value_type=str),
            'navigation_execution_mode': ParameterValue(
                navigation_execution_mode, value_type=str),
            'sls_nav_goal_topic': ParameterValue(sls_nav_goal_topic, value_type=str),
            'takeoff_circle_enabled': ParameterValue(
                takeoff_circle_enabled, value_type=bool),
        }],
        name="behavior_control_node",
        respawn=True  # 重启
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [declare_yaml_path, declare_dry_run, declare_autostart,
         declare_start_state, declare_navigation_execution_mode,
         declare_sls_nav_goal_topic, declare_takeoff_circle_enabled, node_01]
    )
    # 返回让ROS2根据launch描述执行节点
    return launch_description
