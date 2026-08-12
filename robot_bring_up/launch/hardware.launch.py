"""只启动硬件相关节点，用于台架和试飞前检查。

这个 launch 文件是硬件层边界。dry_run 模式下可以启动串口管理、舵机节点
和 MAVROS adapter，但不启动真正连接飞控的 MAVROS，使操作员能在任何飞控
服务调用发生前先验证 ROS 侧连线。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _true_and_not_dry_run(flag_name):
    """仅在对应开关为 true 且 dry_run=false 时启动真实硬件节点。"""
    return PythonExpression([
        "'", LaunchConfiguration(flag_name), "'.lower() == 'true' and '",
        LaunchConfiguration('dry_run'), "'.lower() != 'true'"
    ])


def _true_and_dry_run(flag_name):
    """真实硬件节点因 dry-run 被跳过时输出提示。"""
    return PythonExpression([
        "'", LaunchConfiguration(flag_name), "'.lower() == 'true' and '",
        LaunchConfiguration('dry_run'), "'.lower() == 'true'"
    ])


def generate_launch_description():
    hardware_config_dir = PathJoinSubstitution([
        FindPackageShare('robot_bring_up'), 'config', 'hardware'
    ])

    ports_file = LaunchConfiguration('ports_file')
    servo_file = LaunchConfiguration('servo_file')
    mavros_config_file = LaunchConfiguration('mavros_config_file')
    mavros_gcs_url = LaunchConfiguration('gcs_url')
    mavros_adapter_file = LaunchConfiguration('mavros_adapter_file')
    dry_run = LaunchConfiguration('dry_run')
    dry_run_bool = ParameterValue(dry_run, value_type=bool)
    adapter_dry_run = LaunchConfiguration('adapter_dry_run')
    adapter_dry_run_bool = ParameterValue(adapter_dry_run, value_type=bool)

    # 统一串口清单节点；dry-run 下可安全用于设备发现和配置校验。
    serial_manager_node = Node(
        package='robot_serial_manager',
        executable='robot_serial_manager_node',
        name='robot_serial_manager',
        output='screen',
        parameters=[ports_file, {'dry_run': dry_run_bool}],
        condition=IfCondition(LaunchConfiguration('use_serial_manager')),
    )

    # 舵机节点带独立 dry-run 开关，台架测试时保持实飞 launch 形状但不动作。
    servo_node = Node(
        package='servo_node',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[servo_file, {'dry_run': dry_run_bool}],
        respawn=True,
        condition=IfCondition(LaunchConfiguration('use_servo')),
    )

    # 使用本地 Python MAVROS 包装，避免上游 XML launch 在 Foxy 的 x86 和
    # ARM 镜像上出现替换语法差异。
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('flight_control'), 'launch',
            'mavros_state.launch.py'
        ])),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'tgt_system': LaunchConfiguration('target_system'),
            'tgt_component': LaunchConfiguration('target_component'),
            'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            'config_file': mavros_config_file,
            'gcs_url': mavros_gcs_url,
        }.items(),
        condition=IfCondition(_true_and_not_dry_run('use_mavros')),
    )

    # adapter 在 dry-run 和真实模式都可运行。adapter_dry_run 可单独保持
    # 服务调用抑制，同时允许 MAVROS 真实连接飞控接收 vision pose。
    mavros_adapter_node = Node(
        package='flight_control',
        executable='mavros_adapter_node',
        name='mavros_adapter',
        output='screen',
        parameters=[mavros_adapter_file, {'dry_run': adapter_dry_run_bool}],
        respawn=True,
        condition=IfCondition(LaunchConfiguration('use_mavros')),
    )

    # 旧 pymavlink 节点只作为显式 fallback 保留，默认关闭，主线验证 MAVROS。
    legacy_mavlink_node = Node(
        package='mavlink_control',
        executable='mavlink_control_node',
        name='mavlink_control_node',
        output='screen',
        parameters=[LaunchConfiguration('legacy_mavlink_params_file')],
        respawn=True,
        condition=IfCondition(_true_and_not_dry_run('use_legacy_mavlink')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ports_file',
            default_value=PathJoinSubstitution([hardware_config_dir, 'ports.yaml']),
            description='串口设备清单和占用关系配置。',
        ),
        DeclareLaunchArgument(
            'servo_file',
            default_value=PathJoinSubstitution([hardware_config_dir, 'servo.yaml']),
            description='舵机串口驱动配置。',
        ),
        DeclareLaunchArgument(
            'legacy_mavlink_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_bring_up'), 'config', 'drone.yaml'
            ]),
            description='旧 pymavlink fallback 节点参数文件。',
        ),
        DeclareLaunchArgument(
            'mavros_adapter_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('flight_control'), 'config',
                'mavros_adapter.yaml'
            ]),
            description='MAVROS adapter 节点参数文件。',
        ),
        DeclareLaunchArgument(
            'mavros_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('flight_control'), 'config',
                'mavros_vision_px4.yaml'
            ]),
            description='MAVROS PX4 vision 插件和坐标系配置文件。',
        ),
        DeclareLaunchArgument(
            'dry_run', default_value='true',
            description='是否进入 dry-run；true 时不启动真实 MAVROS 连接。'),
        DeclareLaunchArgument(
            'adapter_dry_run',
            default_value=dry_run,
            description='是否只抑制 MAVROS adapter 的服务调用；默认跟随 dry_run。'),
        DeclareLaunchArgument(
            'use_serial_manager', default_value='true',
            description='是否启动串口管理节点。'),
        DeclareLaunchArgument(
            'use_servo', default_value='true',
            description='是否启动舵机节点。'),
        DeclareLaunchArgument(
            'use_mavros', default_value='true',
            description='是否启用 MAVROS 及其 adapter。'),
        DeclareLaunchArgument(
            'use_legacy_mavlink', default_value='false',
            description='是否启动旧 pymavlink fallback 节点。'),
        DeclareLaunchArgument(
            'use_openmv', default_value='false',
            description='是否尝试启用 OpenMV 串口驱动；当前只提示未实现。'),
        DeclareLaunchArgument(
            'fcu_url', default_value='/dev/px4_fcu:230400',
            description='MAVROS 连接飞控的串口 URL。'),
        DeclareLaunchArgument(
            'gcs_url', default_value='',
            description='MAVROS router 转发给 QGC/GCS 的 MAVLink URL，默认关闭。'),
        DeclareLaunchArgument(
            'target_system', default_value='1',
            description='MAVROS 目标系统 ID。'),
        DeclareLaunchArgument(
            'target_component', default_value='1',
            description='MAVROS 目标组件 ID。'),
        DeclareLaunchArgument(
            'fcu_protocol', default_value='v2.0',
            description='MAVROS 使用的 MAVLink 协议版本。'),
        LogInfo(
            msg=(
                'hardware.launch.py dry-run: MAVROS is configured but not '
                'started; mavros_adapter runs in ROS-only mode.'
            ),
            condition=IfCondition(_true_and_dry_run('use_mavros')),
        ),
        LogInfo(
            msg='hardware.launch.py dry-run: legacy pymavlink fallback is skipped.',
            condition=IfCondition(_true_and_dry_run('use_legacy_mavlink')),
        ),
        serial_manager_node,
        servo_node,
        mavros_launch,
        mavros_adapter_node,
        legacy_mavlink_node,
        LogInfo(
            msg='OpenMV serial driver is not implemented in this baseline.',
            condition=IfCondition(LaunchConfiguration('use_openmv')),
        ),
    ])
