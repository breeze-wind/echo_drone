"""用 ROS2 包装原版 PX4/Gazebo 吊载仿真链路。"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _existing_paths(paths):
    return [str(path) for path in paths if path and Path(path).exists()]


def _joined_paths(paths):
    return os.pathsep.join([path for path in paths if path])


def _default_px4_bin(px4_root):
    # 兼容 PX4 v1.13 常见构建产物路径，避免每次手写 PX4_BIN。
    if not px4_root:
        return 'px4'
    candidates = [
        Path(px4_root) / 'build' / 'px4_sitl_default' / 'bin' / 'px4',
        Path(px4_root) / 'build' / 'px4_sitl_default' / 'px4',
        Path(px4_root) / 'px4',
    ]
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return str(candidates[0])


def _default_px4_etc(px4_root):
    if not px4_root:
        return ''
    return str(Path(px4_root) / 'build' / 'px4_sitl_default' / 'etc')


def _setup(context):
    # 需要运行时拼出 Gazebo/PX4 路径，所以这里用 OpaqueFunction 读取 LaunchConfiguration。
    package_share = Path(get_package_share_directory('sls_circle_controller'))
    px4_sitl_dir = package_share / 'px4_sitl'
    models_dir = px4_sitl_dir / 'models'
    worlds_dir = px4_sitl_dir / 'worlds'
    gazebo_default_model_paths = [
        Path('/usr/share/gazebo-11/models'),
        Path('/usr/share/gazebo/models'),
    ]
    gazebo_default_plugin_paths = [
        Path('/opt/ros/foxy/lib'),
        Path('/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'),
    ]
    gazebo_default_resource_paths = [
        Path('/usr/share/gazebo-11'),
        Path('/usr/share/gazebo'),
    ]

    px4_root = LaunchConfiguration('px4_root').perform(context).strip()
    px4_bin = LaunchConfiguration('px4_bin').perform(context).strip() or _default_px4_bin(px4_root)
    px4_etc = LaunchConfiguration('px4_etc').perform(context).strip() or _default_px4_etc(px4_root)
    px4_model_path = LaunchConfiguration('px4_model_path').perform(context).strip()
    px4_plugin_path = LaunchConfiguration('px4_plugin_path').perform(context).strip()
    extra_model_path = LaunchConfiguration('extra_model_path').perform(context).strip()
    extra_plugin_path = LaunchConfiguration('extra_plugin_path').perform(context).strip()

    px4_builtin_model_paths = []
    px4_builtin_plugin_paths = []
    if px4_root:
        root = Path(px4_root)
        px4_builtin_model_paths = [
            root / 'Tools' / 'sitl_gazebo' / 'models',
            root / 'Tools' / 'simulation' / 'gazebo-classic' / 'sitl_gazebo-classic' / 'models',
        ]
        px4_builtin_plugin_paths = [
            root / 'build' / 'px4_sitl_default' / 'build_gazebo',
            root / 'build' / 'px4_sitl_default' / 'build_gazebo-classic',
            root / 'build' / 'px4_sitl_default' / 'build_gazebo-classic' / 'src',
            root / 'Tools' / 'sitl_gazebo' / 'build',
            root / 'Tools' / 'simulation' / 'gazebo-classic' / 'sitl_gazebo-classic' / 'build',
        ]

    gazebo_model_path = _joined_paths(
        # 搜索顺序优先当前包内模型，再追加用户指定路径、系统 Gazebo 模型和 PX4 自带模型。
        [str(models_dir), px4_model_path, extra_model_path]
        + _existing_paths(gazebo_default_model_paths)
        + _existing_paths(px4_builtin_model_paths)
        + [os.environ.get('GAZEBO_MODEL_PATH', '')]
    )
    gazebo_plugin_path = _joined_paths(
        # PX4 的 gazebo_mavlink_interface 和 motor_model 必须在插件路径里，否则模型只会显示不闭环。
        [px4_plugin_path, extra_plugin_path]
        + _existing_paths(gazebo_default_plugin_paths)
        + _existing_paths(px4_builtin_plugin_paths)
        + [os.environ.get('GAZEBO_PLUGIN_PATH', '')]
    )
    gazebo_resource_path = _joined_paths(
        [str(px4_sitl_dir)]
        + _existing_paths(gazebo_default_resource_paths)
        + [os.environ.get('GAZEBO_RESOURCE_PATH', '')]
    )

    world = LaunchConfiguration('world').perform(context).strip()
    if not world:
        world = str(worlds_dir / 'empty.world')

    sdf = LaunchConfiguration('sdf').perform(context).strip()
    if not sdf:
        model_variant = LaunchConfiguration('model_variant').perform(context).strip()
        sdf = str(models_dir / model_variant / (model_variant + '.sdf'))

    actions = [
        # 禁用在线模型库，避免 Gazebo 在离线或代理异常时卡住。
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', gazebo_plugin_path),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', gazebo_resource_path),
        SetEnvironmentVariable('PX4_SIM_MODEL', LaunchConfiguration('px4_sim_model')),
        SetEnvironmentVariable('PX4_ESTIMATOR', LaunchConfiguration('px4_estimator')),
        LogInfo(msg='PX4 SITL Gazebo model path: ' + gazebo_model_path),
        LogInfo(msg='PX4 SITL Gazebo plugin path: ' + gazebo_plugin_path),
        LogInfo(msg='PX4 SITL world: ' + world),
        LogInfo(msg='PX4 SITL SDF: ' + sdf),
    ]

    gzserver_cmd = ['gzserver', world]
    if _as_bool(LaunchConfiguration('paused').perform(context)):
        gzserver_cmd.append('-u')
    if _as_bool(LaunchConfiguration('verbose').perform(context)):
        gzserver_cmd.append('--verbose')
    gzserver_cmd.extend([
        '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so',
    ])
    actions.append(ExecuteProcess(
        cmd=gzserver_cmd,
        output='screen',
        additional_env={
            'GAZEBO_MODEL_DATABASE_URI': '',
            'GAZEBO_MODEL_PATH': gazebo_model_path,
            'GAZEBO_PLUGIN_PATH': gazebo_plugin_path,
            'GAZEBO_RESOURCE_PATH': gazebo_resource_path,
        },
    ))

    actions.append(ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        additional_env={
            'GAZEBO_MODEL_DATABASE_URI': '',
            'GAZEBO_MODEL_PATH': gazebo_model_path,
            'GAZEBO_PLUGIN_PATH': gazebo_plugin_path,
            'GAZEBO_RESOURCE_PATH': gazebo_resource_path,
        },
        condition=IfCondition(LaunchConfiguration('gui')),
    ))

    if _as_bool(LaunchConfiguration('start_px4').perform(context)):
        # PX4 进程必须晚于 gzserver 可接受连接前后启动，SITL 会等待 TCP 4560。
        if Path(px4_bin).exists() and px4_etc and Path(px4_etc).exists():
            px4_cmd = [px4_bin, px4_etc, '-s', 'etc/init.d-posix/rcS']
            if not _as_bool(LaunchConfiguration('interactive').perform(context)):
                px4_cmd.append('-d')
            actions.append(ExecuteProcess(
                cmd=px4_cmd,
                cwd=px4_root if px4_root else None,
                output='screen',
                additional_env={
                    'PX4_SIM_MODEL': LaunchConfiguration('px4_sim_model').perform(context),
                    'PX4_ESTIMATOR': LaunchConfiguration('px4_estimator').perform(context),
                },
            ))
        else:
            actions.append(LogInfo(
                msg=(
                    'PX4 SITL not started: set px4_root:=/path/to/PX4-Autopilot '
                    'or px4_bin:=... px4_etc:=...'
                )
            ))

    if _as_bool(LaunchConfiguration('spawn_vehicle').perform(context)):
        # spawn_entity 把本包内的 SDF 注入 Gazebo world，entity_name 对应 Gazebo 里的模型名。
        actions.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_px4_sls_vehicle',
            output='screen',
            arguments=[
                '-entity', LaunchConfiguration('entity_name'),
                '-file', sdf,
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('roll'),
                '-P', LaunchConfiguration('pitch'),
                '-Y', LaunchConfiguration('yaw'),
            ],
        ))

    if _as_bool(LaunchConfiguration('start_mavros').perform(context)):
        # PX4 SITL 默认通过 UDP 14540/14580 与 MAVROS 通信，不走真实串口。
        mavros_config = LaunchConfiguration('mavros_config').perform(context).strip()
        mavros_params = []
        if mavros_config:
            mavros_params.append(mavros_config)
        mavros_params.append({
            'fcu_url': LaunchConfiguration('fcu_url'),
            'target_system_id': int(LaunchConfiguration('tgt_system').perform(context)),
            'target_component_id': int(LaunchConfiguration('tgt_component').perform(context)),
            'fcu_protocol': LaunchConfiguration('fcu_protocol'),
        })
        actions.append(Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=mavros_params,
        ))

    if _as_bool(LaunchConfiguration('start_controller').perform(context)):
        # 这里启动的是实控语义的 C++ 控制器，默认会发布真实 MAVROS attitude setpoint。
        controller_params = LaunchConfiguration('params_file').perform(context)
        actions.append(Node(
            package='sls_circle_controller_cpp',
            executable='sls_circle_controller_node_cpp',
            name='sls_circle_controller',
            output='screen',
            arguments=[
                '--ros-args',
                '--params-file', controller_params,
                '-p', 'dry_run:=' + LaunchConfiguration('controller_dry_run').perform(context),
                '-p', 'enable_real_setpoint:=' + LaunchConfiguration('enable_real_setpoint').perform(context),
                '-p', 'mission_mode:=' + LaunchConfiguration('mission_mode').perform(context),
                '-p', 'controller_mode:=' + LaunchConfiguration('controller_mode').perform(context),
                '-p', 'enable_anti_wind:=' + LaunchConfiguration('enable_anti_wind').perform(context),
                '-p', 'require_connected:=' + LaunchConfiguration('require_connected').perform(context),
                '-p', 'require_offboard:=' + LaunchConfiguration('require_offboard').perform(context),
                '-p', 'require_armed:=' + LaunchConfiguration('require_armed').perform(context),
                '-p', 'enable_mavros_services:=' + LaunchConfiguration('enable_mavros_services').perform(context),
                '-p', 'auto_offboard:=' + LaunchConfiguration('auto_offboard').perform(context),
                '-p', 'auto_arm:=' + LaunchConfiguration('auto_arm').perform(context),
                '-p', 'post_takeoff_hold_time:=' + LaunchConfiguration('post_takeoff_hold_time').perform(context),
            ],
        ))

    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory('sls_circle_controller'))
    default_params = str(package_share / 'config' / 'sls_circle.yaml')
    default_mavros_config = str(package_share / 'config' / 'mavros_px4_sitl.yaml')

    return LaunchDescription([
        # PX4/Gazebo 资源路径参数。
        DeclareLaunchArgument('px4_root', default_value=os.environ.get('PX4_DIR', '')),
        DeclareLaunchArgument('px4_bin', default_value=os.environ.get('PX4_BIN', '')),
        DeclareLaunchArgument('px4_etc', default_value=os.environ.get('PX4_ETC', '')),
        DeclareLaunchArgument('px4_model_path', default_value=os.environ.get('PX4_GAZEBO_MODEL_PATH', '')),
        DeclareLaunchArgument('px4_plugin_path', default_value=os.environ.get('PX4_GAZEBO_PLUGIN_PATH', '')),
        DeclareLaunchArgument('extra_model_path', default_value=''),
        DeclareLaunchArgument('extra_plugin_path', default_value=''),
        # PX4 SITL 和 Gazebo 模型参数。
        DeclareLaunchArgument('px4_sim_model', default_value='quadrotor_x'),
        DeclareLaunchArgument('px4_estimator', default_value='ekf2'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('model_variant', default_value='px4vision_sls'),
        DeclareLaunchArgument('sdf', default_value=''),
        DeclareLaunchArgument('entity_name', default_value='px4vision_0'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('interactive', default_value='true'),
        # 分段启动开关，用于先排查 Gazebo、PX4、MAVROS，再接控制器。
        DeclareLaunchArgument('start_px4', default_value='true'),
        DeclareLaunchArgument('spawn_vehicle', default_value='true'),
        DeclareLaunchArgument('start_mavros', default_value='true'),
        DeclareLaunchArgument('start_controller', default_value='true'),
        DeclareLaunchArgument('mavros_config', default_value=default_mavros_config),
        # MAVROS SITL 默认 UDP 链路，和实机 FCU_URL 的串口写法不同。
        DeclareLaunchArgument('fcu_url', default_value='udp://:14540@localhost:14580'),
        DeclareLaunchArgument('tgt_system', default_value='1'),
        DeclareLaunchArgument('tgt_component', default_value='1'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        # 控制器默认是真实 setpoint 语义，PX4 SITL 下会尝试 OFFBOARD/ARM 并执行起飞绕圈。
        DeclareLaunchArgument('controller_dry_run', default_value='false'),
        DeclareLaunchArgument('enable_real_setpoint', default_value='true'),
        DeclareLaunchArgument('mission_mode', default_value='takeoff_then_circle'),
        DeclareLaunchArgument('controller_mode', default_value='qsf'),
        DeclareLaunchArgument('enable_anti_wind', default_value='true'),
        DeclareLaunchArgument('require_connected', default_value='true'),
        DeclareLaunchArgument('require_offboard', default_value='true'),
        DeclareLaunchArgument('require_armed', default_value='true'),
        DeclareLaunchArgument('enable_mavros_services', default_value='true'),
        DeclareLaunchArgument('auto_offboard', default_value='true'),
        DeclareLaunchArgument('auto_arm', default_value='true'),
        DeclareLaunchArgument('post_takeoff_hold_time', default_value='5.0'),
        OpaqueFunction(function=_setup),
    ])
