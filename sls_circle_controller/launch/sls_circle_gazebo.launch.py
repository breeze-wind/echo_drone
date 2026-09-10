"""启动 Gazebo 物理烟测、桥接节点和 SLS 圆周控制器。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def controller_arguments(params_file, override_names):
    # Gazebo 烟测需要频繁从命令行调增益、风力和 mission_mode，所以集中生成覆盖参数。
    arguments = ['--ros-args', '--params-file', params_file]
    for name in override_names:
        arguments.extend(['-p', [name, ':=', LaunchConfiguration(name)]])
    return arguments


def bridge_arguments(params_file):
    # 桥接节点的参数主要控制等效质量、力限幅、状态频率和代码内模拟风。
    return [
        '--ros-args',
        '--params-file',
        params_file,
        '-p',
        ['mass:=', LaunchConfiguration('bridge_mass')],
        '-p',
        ['max_force:=', LaunchConfiguration('max_force')],
        '-p',
        ['attitude_kp:=', LaunchConfiguration('attitude_kp')],
        '-p',
        ['attitude_kd:=', LaunchConfiguration('attitude_kd')],
        '-p',
        ['max_torque:=', LaunchConfiguration('max_torque')],
        '-p',
        ['state_rate:=', LaunchConfiguration('state_rate')],
        '-p',
        ['enable_wind:=', LaunchConfiguration('enable_wind')],
        '-p',
        ['wind_force_x:=', LaunchConfiguration('wind_force_x')],
        '-p',
        ['wind_force_y:=', LaunchConfiguration('wind_force_y')],
        '-p',
        ['wind_force_z:=', LaunchConfiguration('wind_force_z')],
        '-p',
        ['wind_turbulence:=', LaunchConfiguration('wind_turbulence')],
        '-p',
        ['wind_change_rate:=', LaunchConfiguration('wind_change_rate')],
    ]


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    controller_override_names = [
        # 只列出 Gazebo 烟测常用覆盖项，避免 launch 参数无限膨胀。
        'dry_run',
        'enable_real_setpoint',
        'require_connected',
        'require_offboard',
        'require_armed',
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
        'use_load_pose',
        'qsf_reference_is_load',
        'qsf_kp_x',
        'qsf_kv_x',
        'qsf_ka_x',
        'qsf_kj_x',
        'qsf_kp_y',
        'qsf_kv_y',
        'qsf_ka_y',
        'qsf_kj_y',
        'qsf_kp_z',
        'qsf_kv_z',
        'preflight_setpoint_time',
        'takeoff_z_tolerance',
        'takeoff_settle_time',
        'mav_mass',
        'load_mass',
        'cable_length',
        'radius',
        'angular_velocity',
        'post_takeoff_hold_time',
        'max_acc_xy',
        'max_acc_z',
        'max_total_acc',
        'max_tilt_deg',
        'wind_compensation_gain',
        'wind_compensation_warmup_time',
        'wind_compensation_ramp_time',
        'wind_integral_gain',
    ]
    gzserver = IncludeLaunchDescription(
        # 使用 gazebo_ros 官方 gzserver.launch.py，这样 ROS 插件和 /spawn_entity 服务能正常初始化。
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch/gzserver.launch.py',
        ]),
        launch_arguments={
            'world': world,
            'verbose': LaunchConfiguration('verbose'),
            'pause': 'false',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('gazebo_ros'), '/launch/gzclient.launch.py',
        ]),
        condition=IfCondition(gui),
        launch_arguments={
            'verbose': LaunchConfiguration('verbose'),
        }.items(),
    )

    bridge_node = Node(
        # 桥接节点把 Gazebo odom/force 与 MAVROS 风格话题互相转换。
        package='sls_circle_controller',
        executable='gazebo_mavros_bridge_node',
        name='gazebo_mavros_bridge',
        output='screen',
        arguments=bridge_arguments(params_file),
    )

    controller_node = Node(
        # Gazebo 烟测仍跑 C++ 主控制器，只是默认不写真实 /mavros/setpoint_raw/attitude。
        package='sls_circle_controller_cpp',
        executable='sls_circle_controller_node_cpp',
        output='screen',
        arguments=controller_arguments(params_file, controller_override_names),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('sls_circle_controller'), 'config',
                'sls_circle.yaml',
            ]),
            description='SLS 圆周控制器和 Gazebo 桥接参数文件。',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('sls_circle_controller'), 'worlds',
                'sls_circle_force.world',
            ]),
            description='Gazebo world 文件。',
        ),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='true 时打开 gzserver 详细日志。',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='true 时启动 gzclient 图形界面；false 时只跑 headless 烟测。',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='true',
            description='Gazebo 默认只走调试 setpoint，不写真实 MAVROS 控制话题。',
        ),
        DeclareLaunchArgument('enable_real_setpoint', default_value='false'),
        DeclareLaunchArgument('require_connected', default_value='false'),
        DeclareLaunchArgument('require_offboard', default_value='false'),
        DeclareLaunchArgument('require_armed', default_value='false'),
        DeclareLaunchArgument('control_rate', default_value='100.0'),
        DeclareLaunchArgument('use_velocity_topic', default_value='true'),
        DeclareLaunchArgument('velocity_stale_timeout', default_value='0.5'),
        DeclareLaunchArgument('mission_mode', default_value='circle_only'),
        DeclareLaunchArgument('controller_mode', default_value='qsf'),
        DeclareLaunchArgument('enable_anti_wind', default_value='true'),
        DeclareLaunchArgument(
            'use_load_pose',
            default_value='false',
            description='默认使用虚拟垂直负载，true 时接 Gazebo 负载位姿做完整 QSF 试验。',
        ),
        DeclareLaunchArgument('mav_mass', default_value='1.56'),
        DeclareLaunchArgument('load_mass', default_value='0.25'),
        DeclareLaunchArgument('cable_length', default_value='0.85'),
        DeclareLaunchArgument('qsf_reference_is_load', default_value='false'),
        DeclareLaunchArgument('qsf_kp_x', default_value='10.0'),
        DeclareLaunchArgument('qsf_kv_x', default_value='5.0'),
        DeclareLaunchArgument('qsf_ka_x', default_value='0.0'),
        DeclareLaunchArgument('qsf_kj_x', default_value='0.0'),
        DeclareLaunchArgument('qsf_kp_y', default_value='10.0'),
        DeclareLaunchArgument('qsf_kv_y', default_value='5.0'),
        DeclareLaunchArgument('qsf_ka_y', default_value='0.0'),
        DeclareLaunchArgument('qsf_kj_y', default_value='0.0'),
        DeclareLaunchArgument('qsf_kp_z', default_value='20.0'),
        DeclareLaunchArgument('qsf_kv_z', default_value='10.0'),
        DeclareLaunchArgument('preflight_setpoint_time', default_value='2.0'),
        DeclareLaunchArgument('takeoff_z_tolerance', default_value='0.15'),
        DeclareLaunchArgument('takeoff_settle_time', default_value='2.0'),
        DeclareLaunchArgument('radius', default_value='0.8'),
        DeclareLaunchArgument('angular_velocity', default_value='0.25'),
        DeclareLaunchArgument('post_takeoff_hold_time', default_value='5.0'),
        DeclareLaunchArgument('max_acc_xy', default_value='1.2'),
        DeclareLaunchArgument('max_acc_z', default_value='1.2'),
        DeclareLaunchArgument('max_total_acc', default_value='2.0'),
        DeclareLaunchArgument('max_tilt_deg', default_value='12.0'),
        DeclareLaunchArgument('wind_estimator_mode', default_value='residual'),
        DeclareLaunchArgument('wind_observer_bandwidth', default_value='4.0'),
        DeclareLaunchArgument('wind_estimate_filter_tau', default_value='0.2'),
        DeclareLaunchArgument('wind_estimate_force_limit', default_value='5.0'),
        DeclareLaunchArgument('wind_compensation_gain', default_value='0.0'),
        DeclareLaunchArgument('wind_compensation_warmup_time', default_value='10.0'),
        DeclareLaunchArgument('wind_compensation_ramp_time', default_value='4.0'),
        DeclareLaunchArgument('wind_integral_gain', default_value='0.0'),
        # force 插件对整个机体+吊绳+负载系统施力，悬停换算必须使用总质量。
        DeclareLaunchArgument('bridge_mass', default_value='1.82'),
        DeclareLaunchArgument('max_force', default_value='40.0'),
        DeclareLaunchArgument('attitude_kp', default_value='1.2'),
        DeclareLaunchArgument('attitude_kd', default_value='0.30'),
        DeclareLaunchArgument('max_torque', default_value='1.0'),
        DeclareLaunchArgument('state_rate', default_value='100.0'),
        DeclareLaunchArgument('enable_wind', default_value='true'),
        DeclareLaunchArgument('wind_force_x', default_value='0.10'),
        DeclareLaunchArgument('wind_force_y', default_value='0.05'),
        DeclareLaunchArgument('wind_force_z', default_value='0.0'),
        DeclareLaunchArgument('wind_turbulence', default_value='0.01'),
        DeclareLaunchArgument('wind_change_rate', default_value='0.2'),
        gzserver,
        gzclient,
        bridge_node,
        controller_node,
    ])
