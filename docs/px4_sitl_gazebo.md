# PX4 SITL Gazebo 移植说明

本文记录从 ROS1 原版 `controller_sitl_gazebo` 抄入当前 ROS2 Foxy 工作区的仿真资源和启动方式。

> 文档状态：2026-10-27 按 `px4_sitl_sls.launch.py` 和 `config/mavros_px4_sitl.yaml` 核对了默认参数。本文中的 PX4 路径是示例，不代表当前机器已完成 PX4 SITL 构建或运行验证。

## 已抄入的原版内容

- 原版模型目录：`sls_circle_controller/px4_sitl/models/`
- 原版 world 目录：`sls_circle_controller/px4_sitl/worlds/`
- 原版 ROS1 launch 归档：`sls_circle_controller/px4_sitl/ros1_launch/`
- 原版脚本归档：`sls_circle_controller/px4_sitl/scripts/`
- 原版 Gazebo 插件和 QSF 代码归档：`sls_circle_controller/px4_sitl/legacy_src/`

这些文件作为仿真资源和对照源码保留，不作为独立 ROS1 catkin 包在当前 colcon 工作区里构建。

## 新 ROS2 启动入口

主入口：

```bash
./run_echo_drone.sh sls-px4-sitl
```

等价 ROS2 launch：

```bash
ros2 launch sls_circle_controller px4_sitl_sls.launch.py
```

默认链路：

```text
Gazebo Classic
  -> PX4 原版多旋翼/吊载 SDF
  -> PX4 gazebo_mavlink_interface + motor_model
  -> PX4 SITL
  -> MAVROS
  -> sls_circle_controller_cpp
  -> /mavros/setpoint_raw/attitude
```

这条链路用于正式仿真验证，区别于 `sls-circle-gazebo` 的简化外力烟测。

## PX4 依赖与本机状态

当前仓库已经包含原版吊载模型，但原版模型还依赖 PX4/Gazebo Classic 自带资源：

- `libgazebo_mavlink_interface.so`
- `libgazebo_motor_model.so`
- `libgazebo_multirotor_base_plugin.so`
- `libgazebo_imu_plugin.so`
- `px4vision`、`iris`、`gps`、`asphalt_plane` 等 Gazebo 模型资源
- `build/px4_sitl_default/bin/px4`
- `build/px4_sitl_default/etc`

历史运行记录使用 PX4-Autopilot `v1.13.2` 和 `px4_sitl_default gazebo` 构建。目标机应确认以下产物存在于实际 `PX4_DIR` 下：

- `$PX4_DIR/build/px4_sitl_default/bin/px4`
- `$PX4_DIR/build/px4_sitl_default/etc`
- `$PX4_DIR/build/px4_sitl_default/build_gazebo/libgazebo_mavlink_interface.so`
- `$PX4_DIR/build/px4_sitl_default/build_gazebo/libgazebo_motor_model.so`
- `$PX4_DIR/build/px4_sitl_default/build_gazebo/libgazebo_imu_plugin.so`
- `$PX4_DIR/build/px4_sitl_default/build_gazebo/libgazebo_gps_plugin.so`

如果使用 PX4 Python 构建虚拟环境，应把它与实际 `PX4_DIR` 对应，避免污染 ROS2 Foxy 的系统 Python 环境。

后续重新构建 PX4 时使用：

```bash
cd "$PX4_DIR"
PATH="$PX4_DIR/.venv_px4/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
PYTHONNOUSERSITE=1 \
make px4_sitl_default gazebo
```

当前仓库额外放了 `asphalt_plane`、`gps`、`px4vision`、`iris` 的最小本地 fallback 资源，目的是在没有 PX4 模型库时至少能加载 world 和看到基本机体外形。

这些 fallback 不包含 PX4 的电机、MAVLink 或传感器动力学插件，真正起飞闭环仍需要 PX4-Autopilot 的 Gazebo Classic 插件。

当前推荐这样启动：

```bash
PX4_DIR=/path/to/PX4-Autopilot ./run_echo_drone.sh sls-px4-sitl
```

如果 PX4 的插件或模型路径不在默认位置，可以显式指定：

```bash
./run_echo_drone.sh sls-px4-sitl \
  px4_root:=/path/to/PX4-Autopilot \
  px4_model_path:=/path/to/px4/gazebo/models \
  px4_plugin_path:=/path/to/px4/gazebo/plugins
```

## 常用参数

- `gui:=true|false`：是否打开 Gazebo GUI。
- `world:=/path/to/world`：指定 world，默认使用抄入的 `empty.world`。
- `model_variant:=px4vision_sls`：选择抄入的模型目录。
- `sdf:=/path/to/model.sdf`：直接指定 SDF 文件。
- `start_px4:=true|false`：是否启动 PX4 SITL。
- `start_mavros:=true|false`：是否启动 MAVROS。
- `start_controller:=true|false`：是否启动新版 C++ SLS 控制器。
- `px4_sim_model:=gazebo-classic_iris`：`px4_sitl_sls.launch.py` 当前默认值。该机型用于先保证 PX4 SITL 和 Gazebo 电机插件闭环成立；若切换为 `px4vision` 相关机型，需要单独核对 v1.13.2 板卡参数兼容性。
- `mavros_config:=...`：MAVROS SITL 参数文件，默认使用 `config/mavros_px4_sitl.yaml`。
- `fcu_url:=udp://:14540@localhost:14580`：MAVROS 连接 PX4 SITL 的 UDP URL。

注意：默认仍生成 `px4vision_sls` 吊载模型，但 PX4 autostart 当前使用 `gazebo-classic_iris`，这样先保证 PX4 内环和 Gazebo 电机插件闭环成立。后续如果要完全复刻原版 `px4vision` 参数，需要单独清理 `4016_holybro_px4vision` 中不属于 SITL 构建的真实板卡参数。

`config/mavros_px4_sitl.yaml` 当前使用 `plugin_allowlist`，只启用 `sys_status`、`sys_time`、`command`、`local_position`、`setpoint_position`、`setpoint_raw` 和 `imu`；`odometry` 不在 allowlist 中，从而避免 Gazebo SITL 阶段额外的 TF 断树日志。

## 调试建议

先只看模型和插件加载：

```bash
./run_echo_drone.sh sls-px4-sitl start_px4:=false start_mavros:=false start_controller:=false
```

再启动 PX4 和 MAVROS：

```bash
PX4_DIR=/path/to/PX4-Autopilot ./run_echo_drone.sh sls-px4-sitl start_controller:=false
```

最后启动控制器起飞后绕圈：

```bash
PX4_DIR=/path/to/PX4-Autopilot ./run_echo_drone.sh sls-px4-sitl
```

如果 Gazebo 报找不到 `model://px4vision` 或 `libgazebo_motor_model.so`，说明 PX4 Gazebo Classic 模型路径或插件路径还没有配置到 launch 参数里。
