"""启动 ROS2 SLS 圆周控制器。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def controller_arguments(params_file, override_names):
    # 控制器大部分调试参数允许从 ros2 launch 命令行覆盖，便于现场快速切模式。
    arguments = ['--ros-args', '--params-file', params_file]
    for name in override_names:
        arguments.extend(['-p', [name, ':=', LaunchConfiguration(name)]])
    return arguments


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    # 这些参数直接透传到 C++ 控制器，覆盖 config/sls_circle.yaml 的默认值。
    override_names = [
        'dry_run',
        'enable_real_setpoint',
        'pose_topic',
        'velocity_topic',
        'load_pose_topic',
        'state_topic',
        'real_attitude_topic',
        'takeoff_pose_topic',
        'reference_pose_topic',
        'status_topic',
        'control_rate',
        'use_velocity_topic',
        'velocity_stale_timeout',
        'mission_mode',
        'controller_mode',
        'enable_anti_wind',
        'wind_estimator_mode',
        'wind_observer_bandwidth',
        'wind_estimate_filter_tau',
        'wind_estimate_force_limit',
        'wind_compensation_gain',
        'wind_compensation_warmup_time',
        'wind_compensation_ramp_time',
        'wind_integral_gain',
        'require_connected',
        'require_offboard',
        'require_armed',
        'enable_mavros_services',
        'auto_offboard',
        'auto_arm',
        'use_load_pose',
        'qsf_reference_is_load',
        'preflight_setpoint_time',
        'takeoff_altitude',
        'takeoff_z_tolerance',
        'takeoff_settle_time',
        'post_takeoff_hold_time',
        'radius',
        'angular_velocity',
        'circle_loops',
        'mav_mass',
        'load_mass',
        'cable_length',
    ]

    controller_node = Node(
        # 当前主路径使用 C++ 控制器，Python 版节点只保留作历史对照。
        package='sls_circle_controller_cpp',
        executable='sls_circle_controller_node_cpp',
        output='screen',
        arguments=controller_arguments(params_file, override_names),
    )

    return LaunchDescription([
        # 单节点入口默认 dry-run，必须显式关闭 dry_run 并启用 real_setpoint 才会写 MAVROS。
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('sls_circle_controller'), 'config',
                'sls_circle.yaml',
            ]),
            description='SLS 圆周控制器参数文件。',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='true',
            description='true 时只发布调试 setpoint，不写真实 MAVROS 控制话题。',
        ),
        DeclareLaunchArgument(
            'enable_real_setpoint',
            default_value='false',
            description='必须和 dry_run=false 同时设置才会写 /mavros/setpoint_raw/attitude。',
        ),
        DeclareLaunchArgument('pose_topic', default_value='/mavros/local_position/pose'),
        DeclareLaunchArgument(
            'velocity_topic',
            default_value='/mavros/local_position/velocity_local',
        ),
        DeclareLaunchArgument('load_pose_topic', default_value='/sls_circle/load_pose'),
        DeclareLaunchArgument('state_topic', default_value='/mavros/state'),
        DeclareLaunchArgument(
            'real_attitude_topic',
            default_value='/mavros/setpoint_raw/attitude',
        ),
        DeclareLaunchArgument(
            'takeoff_pose_topic',
            default_value='/mavros/setpoint_position/local',
        ),
        DeclareLaunchArgument('reference_pose_topic', default_value='/sls_circle/reference_pose'),
        DeclareLaunchArgument('status_topic', default_value='/sls_circle/status'),
        DeclareLaunchArgument(
            'control_rate',
            default_value='100.0',
            description='控制器定时循环频率，实机可临时设为 50.0、100.0 或 150.0。',
        ),
        DeclareLaunchArgument('use_velocity_topic', default_value='true'),
        DeclareLaunchArgument('velocity_stale_timeout', default_value='0.5'),
        DeclareLaunchArgument(
            'mission_mode',
            default_value='circle_only',
            description='circle_only、takeoff_then_circle 或 takeoff_then_hold。',
        ),
        DeclareLaunchArgument(
            'controller_mode',
            default_value='qsf',
            description='pd、leso_pd、qsf 或 qsf_integral。',
        ),
        DeclareLaunchArgument(
            'enable_anti_wind',
            default_value='true',
            description='true 时启用 LESO 加残差积分抗风扰。',
        ),
        DeclareLaunchArgument('wind_estimator_mode', default_value='residual'),
        DeclareLaunchArgument('wind_observer_bandwidth', default_value='4.0'),
        DeclareLaunchArgument('wind_estimate_filter_tau', default_value='0.5'),
        DeclareLaunchArgument('wind_estimate_force_limit', default_value='5.0'),
        DeclareLaunchArgument('wind_compensation_gain', default_value='1.0'),
        DeclareLaunchArgument('wind_compensation_warmup_time', default_value='5.0'),
        DeclareLaunchArgument('wind_compensation_ramp_time', default_value='5.0'),
        DeclareLaunchArgument('wind_integral_gain', default_value='0.0'),
        DeclareLaunchArgument('require_connected', default_value='true'),
        DeclareLaunchArgument('require_offboard', default_value='false'),
        DeclareLaunchArgument('require_armed', default_value='false'),
        DeclareLaunchArgument('enable_mavros_services', default_value='false'),
        DeclareLaunchArgument('auto_offboard', default_value='false'),
        DeclareLaunchArgument('auto_arm', default_value='false'),
        DeclareLaunchArgument('preflight_setpoint_time', default_value='2.0'),
        DeclareLaunchArgument('takeoff_altitude', default_value='1.0'),
        DeclareLaunchArgument('takeoff_z_tolerance', default_value='0.15'),
        DeclareLaunchArgument('takeoff_settle_time', default_value='2.0'),
        DeclareLaunchArgument(
            'post_takeoff_hold_time',
            default_value='5.0',
            description='takeoff_then_circle 中起飞确认后的定点等待时间。',
        ),
        DeclareLaunchArgument('radius', default_value='1.0'),
        DeclareLaunchArgument('angular_velocity', default_value='0.35'),
        DeclareLaunchArgument('circle_loops', default_value='0.0'),
        DeclareLaunchArgument('use_load_pose', default_value='false'),
        DeclareLaunchArgument('mav_mass', default_value='1.56'),
        DeclareLaunchArgument('load_mass', default_value='0.25'),
        DeclareLaunchArgument('cable_length', default_value='0.85'),
        DeclareLaunchArgument('qsf_reference_is_load', default_value='false'),
        controller_node,
    ])
