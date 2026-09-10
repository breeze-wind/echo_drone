"""Rolling-costmap Nav2 stack for Point-LIO localization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_bringup_path = get_package_share_directory('robot_bring_up')
    obstacle_segmentation_path = get_package_share_directory('obstacle_segmentation')

    default_params = os.path.join(robot_bringup_path, 'config', 'drone.yaml')
    default_costmap_params = os.path.join(robot_bringup_path, 'config', 'sensing_costmap.yaml')
    default_bt_xml = os.path.join(
        robot_bringup_path,
        'behavior_trees',
        'navigate_drone_replanning.xml',
    )

    params_file = LaunchConfiguration('params_file')
    costmap_params_file = LaunchConfiguration('costmap_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'recoveries_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]
    recovery_remappings = remappings + [('cmd_vel', '/nav/recovery_cmd_vel')]

    nav_params = [
        params_file,
        {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
    ]

    obstacle_segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [obstacle_segmentation_path, '/launch', '/obstacle_segmentation.launch.py']
        ),
        launch_arguments={
            'params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 主参数文件。'),
        DeclareLaunchArgument(
            'costmap_params_file',
            default_value=default_costmap_params,
            description='单雷达 rolling costmap overlay 参数文件。'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否使用仿真时间。'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='是否自动激活 Nav2 lifecycle 节点。'),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            description='节点崩溃后是否自动重启。'),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='日志级别。'),
        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=default_bt_xml,
            description='NavigateToPose 默认行为树 XML。'),

        obstacle_segmentation_launch,

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[params_file, costmap_params_file],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[params_file, costmap_params_file],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=nav_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=recovery_remappings),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[
                params_file,
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'default_bt_xml_filename': default_bt_xml_filename},
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=nav_params,
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                {'autostart': ParameterValue(autostart, value_type=bool)},
                {'node_names': lifecycle_nodes},
            ]),
    ])
