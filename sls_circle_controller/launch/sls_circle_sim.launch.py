"""启动轻量 MAVROS 仿真和 SLS 圆周控制器。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def controller_arguments(params_file):
    # 轻量仿真永远走 dry-run，只向 /sls_circle/debug/attitude_target 发调试 setpoint。
    return [
        '--ros-args',
        '--params-file',
        params_file,
        '-p',
        ['dry_run:=true'],
        '-p',
        ['enable_real_setpoint:=false'],
        '-p',
        ['require_connected:=false'],
        '-p',
        ['control_rate:=', LaunchConfiguration('control_rate')],
        '-p',
        ['controller_mode:=', LaunchConfiguration('controller_mode')],
        '-p',
        ['mission_mode:=', LaunchConfiguration('mission_mode')],
        '-p',
        ['use_load_pose:=false'],
        '-p',
        ['load_mass:=0.0'],
        '-p',
        ['enable_anti_wind:=', LaunchConfiguration('enable_anti_wind')],
        '-p',
        ['wind_estimator_mode:=', LaunchConfiguration('wind_estimator_mode')],
        '-p',
        ['wind_estimate_filter_tau:=', LaunchConfiguration('wind_estimate_filter_tau')],
        '-p',
        ['wind_compensation_warmup_time:=', LaunchConfiguration('wind_compensation_warmup_time')],
        '-p',
        ['wind_compensation_ramp_time:=', LaunchConfiguration('wind_compensation_ramp_time')],
        '-p',
        ['preflight_setpoint_time:=', LaunchConfiguration('preflight_setpoint_time')],
        '-p',
        ['takeoff_z_tolerance:=', LaunchConfiguration('takeoff_z_tolerance')],
        '-p',
        ['takeoff_settle_time:=', LaunchConfiguration('takeoff_settle_time')],
        '-p',
        ['post_takeoff_hold_time:=', LaunchConfiguration('post_takeoff_hold_time')],
    ]


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    sim_node = Node(
        # fake_mavros_sim_node 提供最小 MAVROS 状态和位姿闭环，不依赖 Gazebo 或 PX4。
        package='sls_circle_controller',
        executable='fake_mavros_sim_node',
        name='fake_mavros_sim',
        output='screen',
        parameters=[params_file],
    )

    controller_node = Node(
        # C++ 控制器读取 fake MAVROS 输出，用于快速验证轨迹和控制频率。
        package='sls_circle_controller_cpp',
        executable='sls_circle_controller_node_cpp',
        output='screen',
        arguments=controller_arguments(params_file),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('sls_circle_controller'), 'config',
                'sls_circle.yaml',
            ]),
            description='SLS 圆周控制器和 fake MAVROS 仿真参数文件。',
        ),
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
        DeclareLaunchArgument('control_rate', default_value='100.0'),
        DeclareLaunchArgument('enable_anti_wind', default_value='true'),
        DeclareLaunchArgument('wind_estimator_mode', default_value='residual'),
        DeclareLaunchArgument('wind_estimate_filter_tau', default_value='0.2'),
        DeclareLaunchArgument('wind_compensation_warmup_time', default_value='10.0'),
        DeclareLaunchArgument('wind_compensation_ramp_time', default_value='4.0'),
        DeclareLaunchArgument('preflight_setpoint_time', default_value='2.0'),
        DeclareLaunchArgument('takeoff_z_tolerance', default_value='0.15'),
        DeclareLaunchArgument('takeoff_settle_time', default_value='2.0'),
        DeclareLaunchArgument('post_takeoff_hold_time', default_value='5.0'),
        sim_node,
        controller_node,
    ])
