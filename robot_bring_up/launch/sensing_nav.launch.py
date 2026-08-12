"""Single-lidar Point-LIO, rolling costmaps, and Nav2 action services."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_bringup_path = get_package_share_directory('robot_bring_up')
    launch_path = os.path.join(robot_bringup_path, 'launch')
    default_params = os.path.join(robot_bringup_path, 'config', 'drone.yaml')

    params_file = LaunchConfiguration('params_file')
    livox_config = LaunchConfiguration('livox_config')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_livox = LaunchConfiguration('launch_livox')
    launch_pointlio = LaunchConfiguration('launch_pointlio')
    launch_static_tf = LaunchConfiguration('launch_static_tf')
    pointlio_delay = LaunchConfiguration('pointlio_delay')
    costmap_delay = LaunchConfiguration('costmap_delay')
    nav_delay = LaunchConfiguration('nav_delay')
    is_map = LaunchConfiguration('is_map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    sensing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_path, 'sensing.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'livox_config': livox_config,
            'launch_livox': launch_livox,
            'launch_pointlio': launch_pointlio,
            'launch_static_tf': launch_static_tf,
            'launch_rviz': launch_rviz,
            'launch_costmap': 'false',
            'pointlio_delay': pointlio_delay,
            'costmap_delay': costmap_delay,
            'is_map': is_map,
        }.items(),
    )

    lidar_nav_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_path, 'lidar_nav_stack.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'log_level': log_level,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Point-LIO 和 Nav2 使用的主参数文件。'),
        DeclareLaunchArgument(
            'livox_config',
            default_value=os.path.join(
                get_package_share_directory('livox_ros_driver2'),
                'config',
                'MID360s_config.json',
            ),
            description='Livox JSON 配置；VMware 入口传入按 guest IP 生成的本地文件。'),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='是否启动 RViz。'),
        DeclareLaunchArgument(
            'launch_livox',
            default_value='true',
            description='是否启动 Livox 驱动。'),
        DeclareLaunchArgument(
            'launch_pointlio',
            default_value='true',
            description='是否启动 Point-LIO。'),
        DeclareLaunchArgument(
            'launch_static_tf',
            default_value='true',
            description='是否发布感知静态 TF。'),
        DeclareLaunchArgument(
            'pointlio_delay',
            default_value='8.0',
            description='Livox 启动后等待多少秒再启动 Point-LIO。'),
        DeclareLaunchArgument(
            'costmap_delay',
            default_value='12.0',
            description='Livox 启动后等待多少秒再启动 rolling costmap。'),
        DeclareLaunchArgument(
            'nav_delay',
            default_value='18.0',
            description='启动后等待多少秒再追加 BT Navigator 等 Nav2 action 服务。'),
        DeclareLaunchArgument(
            'is_map',
            default_value='false',
            description='是否使用先验 PCD map。实机单雷达测试默认关闭。'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间。'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='是否自动激活 Nav2 lifecycle 节点。'),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='日志级别。'),

        sensing_launch,
        TimerAction(
            period=nav_delay,
            actions=[lidar_nav_stack_launch],
        ),
    ])
