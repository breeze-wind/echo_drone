from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _true_and_not_dry_run(flag_name):
    return PythonExpression([
        "'", LaunchConfiguration(flag_name), "'.lower() == 'true' and '",
        LaunchConfiguration('dry_run'), "'.lower() != 'true'"
    ])


def _true_and_dry_run(flag_name):
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
    dry_run = LaunchConfiguration('dry_run')

    serial_manager_node = Node(
        package='robot_serial_manager',
        executable='robot_serial_manager_node',
        name='robot_serial_manager',
        output='screen',
        parameters=[ports_file, {'dry_run': dry_run}],
        condition=IfCondition(LaunchConfiguration('use_serial_manager')),
    )

    servo_node = Node(
        package='servo_node',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[servo_file, {'dry_run': dry_run}],
        respawn=True,
        condition=IfCondition(LaunchConfiguration('use_servo')),
    )

    mavros_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mavros'), 'launch', 'px4.launch'
        ])),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('target_system'),
            'tgt_component': LaunchConfiguration('target_component'),
            'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            'respawn_mavros': LaunchConfiguration('respawn_mavros'),
            'log_output': 'screen',
        }.items(),
        condition=IfCondition(_true_and_not_dry_run('use_mavros')),
    )

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
            description='Serial device inventory and ownership configuration.',
        ),
        DeclareLaunchArgument(
            'servo_file',
            default_value=PathJoinSubstitution([hardware_config_dir, 'servo.yaml']),
            description='Servo serial driver configuration.',
        ),
        DeclareLaunchArgument(
            'legacy_mavlink_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_bring_up'), 'config', 'drone.yaml'
            ]),
            description='Parameter file for the legacy pymavlink fallback node.',
        ),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('use_serial_manager', default_value='true'),
        DeclareLaunchArgument('use_servo', default_value='true'),
        DeclareLaunchArgument('use_mavros', default_value='true'),
        DeclareLaunchArgument('use_legacy_mavlink', default_value='false'),
        DeclareLaunchArgument('use_openmv', default_value='false'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/px4_fcu:230400'),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('target_system', default_value='1'),
        DeclareLaunchArgument('target_component', default_value='1'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        DeclareLaunchArgument('respawn_mavros', default_value='true'),
        LogInfo(
            msg='hardware.launch.py dry-run: MAVROS is configured but not started.',
            condition=IfCondition(_true_and_dry_run('use_mavros')),
        ),
        LogInfo(
            msg='hardware.launch.py dry-run: legacy pymavlink fallback is skipped.',
            condition=IfCondition(_true_and_dry_run('use_legacy_mavlink')),
        ),
        serial_manager_node,
        servo_node,
        mavros_launch,
        legacy_mavlink_node,
        LogInfo(
            msg='OpenMV serial driver is not implemented in this baseline.',
            condition=IfCondition(LaunchConfiguration('use_openmv')),
        ),
    ])
