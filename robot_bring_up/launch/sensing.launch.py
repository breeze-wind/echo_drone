"""实机感知最小入口：Livox、Point-LIO 和静态 TF。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_bringup_path = get_package_share_directory('robot_bring_up')
    point_lio_path = get_package_share_directory('point_lio')
    livox_driver_path = get_package_share_directory('livox_ros_driver2')
    obstacle_segmentation_path = get_package_share_directory('obstacle_segmentation')

    default_params = os.path.join(robot_bringup_path, 'config', 'drone.yaml')
    default_costmap_params = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        'config',
        'sensing_costmap.yaml',
    )

    params_file = LaunchConfiguration('params_file')
    livox_config = LaunchConfiguration('livox_config')
    costmap_params_file = LaunchConfiguration('costmap_params_file')
    launch_livox = LaunchConfiguration('launch_livox')
    launch_pointlio = LaunchConfiguration('launch_pointlio')
    launch_static_tf = LaunchConfiguration('launch_static_tf')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_costmap = LaunchConfiguration('launch_costmap')
    pointlio_delay = LaunchConfiguration('pointlio_delay')
    costmap_delay = LaunchConfiguration('costmap_delay')
    is_map = LaunchConfiguration('is_map')

    livox_driver_launch = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            # MID360S and Point-LIO now use the same aligned LiDAR frame.
            'frame_id': 'livox',
            'user_config_path': livox_config,
            'cmdline_input_bd_code': 'livox0000000001',
        }],
        condition=IfCondition(launch_livox),
    )

    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [point_lio_path, '/launch', '/pointlio.launch.py']
        ),
        launch_arguments={
            'config_path': params_file,
            'rviz': launch_rviz,
            'is_map': is_map,
        }.items(),
        condition=IfCondition(launch_pointlio),
    )

    obstacle_segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [obstacle_segmentation_path, '/launch', '/obstacle_segmentation.launch.py']
        ),
        launch_arguments={
            'params_file': params_file,
        }.items(),
        condition=IfCondition(launch_costmap),
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, costmap_params_file],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=IfCondition(launch_costmap),
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, costmap_params_file],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=IfCondition(launch_costmap),
    )

    lifecycle_manager_costmap = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['controller_server', 'planner_server']},
        ],
        condition=IfCondition(launch_costmap),
    )

    # 这些 TF 原来散落在 drone.launch.py 中；感知最小流程单独带上，避免
    # 只跑雷达和 LIO 时出现 frame 不完整。
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_broadcaster',
        arguments=[
            '0.0', '0.0', '0.39',
            '0.0', '0.0', '0.0', '1.0',
            'map', 'odom',
        ],
        condition=IfCondition(launch_static_tf),
    )

    livox_to_mavlink_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_mavlink_body_broadcaster',
        arguments=[
            '0.0', '0.0', '-0.08',
            # Keep the original 180deg roll axis conversion and add a +45deg
            # yaw correction for the MID360s mounting offset.
            '0.9238795', '0.3826834', '0.0', '0.0',
            'livox', 'mavlink_body',
        ],
        condition=IfCondition(launch_static_tf),
    )

    livox_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_camera_broadcaster',
        arguments=[
            '0.065', '-0.065', '-0.26',
            '0.3827', '0.9239', '0.0', '0.0',
            'livox', 'camera_link',
        ],
        condition=IfCondition(launch_static_tf),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Point-LIO 和后续感知链路使用的主参数文件。',
        ),
        DeclareLaunchArgument(
            'launch_livox',
            default_value='true',
            description='是否启动 Livox MID360 驱动。',
        ),
        DeclareLaunchArgument(
            'livox_config',
            default_value=os.path.join(livox_driver_path, 'config', 'MID360s_config.json'),
            description='Livox JSON 配置；VMware 入口传入按 guest IP 生成的本地文件。',
        ),
        DeclareLaunchArgument(
            'launch_pointlio',
            default_value='true',
            description='是否启动 Point-LIO。',
        ),
        DeclareLaunchArgument(
            'launch_static_tf',
            default_value='true',
            description='是否发布感知链路需要的静态 TF。',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='是否启动 RViz 直接查看点云。',
        ),
        DeclareLaunchArgument(
            'launch_costmap',
            default_value='false',
            description='是否启动障碍物分割和 Nav2 costmap 节点。',
        ),
        DeclareLaunchArgument(
            'costmap_params_file',
            default_value=default_costmap_params,
            description='无静态地图的实机 costmap overlay 参数文件。',
        ),
        DeclareLaunchArgument(
            'pointlio_delay',
            default_value='8.0',
            description='Livox 启动后等待多少秒再启动 Point-LIO。',
        ),
        DeclareLaunchArgument(
            'costmap_delay',
            default_value='12.0',
            description='Livox 启动后等待多少秒再启动 costmap 链路。',
        ),
        DeclareLaunchArgument(
            'is_map',
            default_value='false',
            description='是否让 Point-LIO 使用先验 PCD map 初始化。',
        ),
        map_to_odom,
        livox_to_mavlink_body,
        livox_to_camera,
        livox_driver_launch,
        TimerAction(
            period=pointlio_delay,
            actions=[point_lio_launch],
        ),
        TimerAction(
            period=costmap_delay,
            actions=[
                obstacle_segmentation_launch,
                controller_server,
                planner_server,
                lifecycle_manager_costmap,
            ],
        ),
    ])
