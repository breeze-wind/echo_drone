# SLS 圆周控制器接入说明

## 当前定位

`sls_circle_controller` 是 `drown` 悬挂负载控制算法在当前 ROS2 Foxy/MAVROS 主线里的接入层。

当前已经把 `drown` 里的 MATLAB Coder 生成算法拆成 `sls_qsf_core` 共享库，实机主控节点默认由 `sls_circle_controller_cpp` 提供。

原 `sls_circle_controller` 包仍保留 Python fake MAVROS、Gazebo 桥接和历史 Python 控制器，主要用于软件仿真和回退排查。

这条链路的目标分成三层：

- `sls-circle-sim`：不依赖 Gazebo，用 fake MAVROS 点质量模型验证算法接口、圆周参考和状态机。
- `sls-circle-gazebo`：用 Gazebo 力输入模型看闭环运动、负载模型和风扰话题，不等价于 PX4/SITL。
- `real-wind`：真实上机时只通过 MAVROS 读状态和写 setpoint，不再裸连 MAVLink。

## 算法接入

`sls_qsf_core` 提供三组 C ABI：

- `sls_qsf_controller`
- `sls_qsf_integral_controller`
- `sls_qsf_geometric_controller`

`sls_circle_controller_cpp/src/sls_circle_controller_node.cpp` 直接链接 `sls_qsf_core`，主控制节点通过 `controller_mode` 选择算法：

- `pd`：普通位置 PD，主要用于回退和排查。
- `leso_pd` 或 `leso`：PD 加 LESO 扰动估计。
- `qsf`：默认模式，调用原 `QSFController`。
- `qsf_integral`：调用原 `QSFIntegralController`，带内部积分状态。

QSF 生成库按原算法使用 NED/负载语义，ROS2 控制节点内部使用 ENU/MAVROS 语义，所以节点里集中完成了轴向交换、Z 轴取反、重力项和负载参考高度转换。

## ROS1 公式核对

已对照 `/home/sfx/drown_ws/src/drown/mrotor_controller/src/mrotor_controller/sls_controller.cpp` 和 Gazebo SLS 插件核对核心公式。

坐标映射保持一致：

```text
SLS state = [load_y, load_x, -load_z, pend_y, pend_x, -pend_z, load_vy, load_vx, -load_vz, pend_rate_y, pend_rate_x, -pend_rate_z]
QSF ref_x = ROS ENU y
QSF ref_y = ROS ENU x
QSF ref_z = -ROS ENU z
ENU thrust_accel = [F_ned_y, F_ned_x, -F_ned_z] / mav_mass
```

ROS1 `mrotor_controller` 中先得到 `a_des=[F_y,F_x,-F_z]/m`，再使用 `a_fb=a_des+gravity_` 做限幅，最后返回给几何控制器的是净加速度。

当前 C++ 节点采用同一约定：QSF 输出先转为 ENU 总推力加速度，再减去 `gravity` 存为 MAVROS 姿态 setpoint 的净加速度；构造姿态和推力时再把 `gravity` 加回。

抗风扰当前使用 LESO 加残差积分后级补偿，接口和 ROS1 LESO 段一致，输出单位是 ENU 加速度补偿，并额外打印等效力；它不是 ROS1 Gazebo 插件里的滑模外力补偿 `-K_sm_force * sat(...)`。

抗风扰只允许修正 ENU 的 x/y 平面，z 方向的实际风、估计风和补偿风均固定为 0，不会把 z 补偿加到控制加速度里。

现场核对坐标系时直接看 `/sls_circle/status` 里的字段：

- `qsf_state_ned_load_semantics`：送入 QSF 的 12 维状态，顺序是原 ROS1 的 NED/负载语义。
- `qsf_ref_x_north`：QSF 北向参考，对应 ROS ENU 的 `y`。
- `qsf_ref_y_east`：QSF 东向参考，对应 ROS ENU 的 `x`。
- `qsf_ref_z_down`：QSF 下向参考，对应 ROS ENU 的 `-z`。
- `qsf_force_ned`：QSF 输出的 NED 力。
- `qsf_thrust_accel_enu`：由 QSF 力换回 ROS ENU 的总推力加速度。
- `qsf_net_accel_enu`：送入 MAVROS 姿态目标生成前的净加速度。

## 抗风扰

抗风扰现在放在控制节点后级补偿里，入口参数是：

- `enable_anti_wind`
- `wind_estimator_mode`
- `wind_estimate_filter_tau`
- `wind_estimate_force_limit`
- `wind_observer_bandwidth`
- `wind_compensation_gain`
- `wind_integral_gain`
- `wind_integral_limit`

控制节点发布 `/sls_circle/wind_estimate`，单位是 N，Gazebo 桥接节点发布 `/sls_circle/wind_actual`。

`wind_estimator_mode` 当前有三种取值：

- `actual_feedback`：仿真对照模式，订阅 `/sls_circle/wind_actual` 后直接跟踪代码注入风力，只用于核对打印和坐标，不作为默认预测方式。
- `residual`：默认预测模式，用速度/位姿差分得到的观测加速度，叠加负载加速度补偿后再低通，作为外扰估计。
- `leso`：三阶位置 LESO，主要保留给对照调参。

默认都使用 `residual`，因为这才是实际要验证的预测链路。

`actual_feedback` 只建议在 Gazebo 里临时打开，用来检查 `/sls_circle/wind_actual` 的真值打印是否正常。

Gazebo 风扰是桥接节点在 `/sls_circle/gazebo_force` 上叠加的小外力，用来做软件烟测，不代表真实空气动力模型。

Gazebo 风扰同样强制只施加 x/y 平面力，`wind_force_z` 目前为兼容参数，实际输出固定为 0。

## 安全默认值

- `dry_run: true`
- `enable_real_setpoint: false`
- 默认只发布 `/sls_circle/debug/attitude_target`
- 默认不会写 `/mavros/setpoint_raw/attitude`

真实 setpoint 必须同时满足 `dry_run=false` 和 `enable_real_setpoint=true`。

起飞和定点阶段使用 `/mavros/setpoint_position/local`，进入 `circle` 后才会按安全门输出 `/mavros/setpoint_raw/attitude`。

推荐实机入口已经收敛为 `real-point`、`real-swing`、`real-wind` 这类启动指令，脚本内部会设置必要确认。

兼容入口 `sls-circle-real` 和 `sls-circle-real-auto` 仍保留旧环境变量安全门，主要给脚本化和保守联调用。

## 主要话题

- 输入：`/mavros/local_position/pose`
- 输入：`/mavros/state`
- 可选输入：`/sls_circle/load_pose`
- 调试输出：`/sls_circle/reference_pose`
- 调试输出：`/sls_circle/debug/attitude_target`
- 状态输出：`/sls_circle/status`
- 风扰估计：`/sls_circle/wind_estimate`
- Gazebo 实际风扰：`/sls_circle/wind_actual`
- 真实输出：`/mavros/setpoint_raw/attitude`

## 构建

```bash
cd /home/sfx/echo_drone
source /opt/ros/foxy/setup.bash
colcon build --packages-select sls_qsf_core sls_circle_controller_cpp sls_circle_controller --symlink-install
source install/setup.bash
```

## 启动方法

轻量闭环仿真。

```bash
./run_echo_drone.sh sls-circle-sim
```

Gazebo 图形仿真。

```bash
./run_echo_drone.sh sls-circle-gazebo
```

原版 PX4 SITL/Gazebo 多旋翼吊载闭环仿真。

```bash
PX4_DIR=/home/sfx/PX4-Autopilot ./run_echo_drone.sh sls-px4-sitl
```

Gazebo headless 烟测。

```bash
./run_echo_drone.sh sls-circle-gazebo gui:=false
```

Gazebo 高频率烟测。

```bash
./run_echo_drone.sh sls-circle-gazebo gui:=false control_rate:=150.0 state_rate:=150.0
```

Gazebo 的机体和负载 p3d 里程计插件固定为 200Hz，`control_rate` 继续提高时要看 `pose_input_hz` 是否跟得上。

Gazebo 完整负载位姿反馈试验。

```bash
./run_echo_drone.sh sls-circle-gazebo use_load_pose:=true gui:=false
```

只接真实 MAVROS 位姿但不下发控制。

```bash
./run_echo_drone.sh sls-circle-dry
```

dry-run 检查起飞后绕圈状态机。

```bash
./run_echo_drone.sh sls-circle-mission-dry
```

dry-run 检查起飞后只定点悬停。

```bash
./run_echo_drone.sh sls-takeoff-hold-dry
```

真实起飞后只定点悬停，不进入绕圈。

```bash
./run_echo_drone.sh real-hold
```

真实 setpoint 输出，不自动 OFFBOARD/解锁。

```bash
./run_echo_drone.sh real-circle
```

自动请求 OFFBOARD 和解锁的真实入口。

```bash
./run_echo_drone.sh real-circle-auto
```

## Gazebo 当前边界

`sls_circle_gazebo.launch.py` 默认启动 `gzserver`、`gzclient`、`gazebo_mavros_bridge_node` 和圆周控制器。

默认 Gazebo 参数采用保守可视化配置：

- `controller_mode:=qsf`
- `wind_estimator_mode:=residual`
- `wind_estimate_filter_tau:=0.2`
- `use_load_pose:=false`
- `radius:=0.8`
- `angular_velocity:=0.25`
- `post_takeoff_hold_time:=5.0`
- `max_tilt_deg:=12.0`
- `control_rate:=100.0`
- `state_rate:=100.0`
- Gazebo 机体和负载 odom 插件为 200Hz
- `enable_wind:=true`
- `wind_force_z:=0.0` 且实际 z 风扰固定为 0

这里默认关闭真实负载位姿反馈，是因为 `sls-circle-gazebo` 只是把 MAVROS 姿态目标转换成世界系外力，不包含 PX4 姿态环、机体阻尼和真实电机模型。

需要检查真实多旋翼姿态闭环时，使用 `sls-px4-sitl`，它会加载从原版抄入的 PX4/Gazebo 吊载模型，并通过 PX4 SITL、MAVROS 和新版 C++ SLS 控制器组成闭环。

需要检查完整 QSF 负载反馈时再显式加 `use_load_pose:=true`。

## 验证指标

- `/sls_circle/status` 中 `controller_mode` 应为 `qsf`。
- `/sls_circle/status` 中 `qsf_core_loaded` 应为 `true`。
- `takeoff_then_hold` 模式中 `/sls_circle/status` 的 `flight_stage` 应停在 `hold`。
- `takeoff_then_circle` 模式中 `/sls_circle/status` 的 `flight_stage` 应先进入 `hold`，等待 `post_takeoff_hold_time` 后再进入 `circle`。
- `/sls_circle/reference_pose` 应按圆周变化。
- `/sls_circle/debug/attitude_target` 应稳定输出。
- `/sls_circle/status` 中 `actual_control_hz`、`command_publish_hz`、`pose_input_hz`、`actual_wind_input_hz` 应接近目标频率。
- 轻量仿真中 `/mavros/local_position/pose` 应跟随参考轨迹绕圈。
- Gazebo 烟测中 `/sls_circle/gazebo_odom`、`/sls_circle/gazebo_force`、`/sls_circle/wind_actual` 应持续刷新。
- Gazebo 固定风烟测中，`wind_estimator_mode:=residual` 时 `/sls_circle/status` 的 `wind_estimate_rel_error_xy_max` 稳态应小于 `0.3`。

风扰调试日志还会打印 `rt_hz target/control/cmd/pose/load_pose/actual_wind`，Gazebo 桥接日志会打印 `rt_hz force/odom/load_odom/state`。

## 上机顺序

完整实机启动说明见：

- `docs/sls_real_startup.md`

上机前不要直接跑 `real-circle-auto`。

建议顺序：

```bash
./run_echo_drone.sh check
./run_echo_drone.sh mavros-state
./run_echo_drone.sh pointlio
./run_echo_drone.sh sls-takeoff-hold-dry
./run_echo_drone.sh sls-circle-dry
./run_echo_drone.sh real-hold
```

`real-circle` 默认要求飞控已经 connected、OFFBOARD、armed 后才会真正发布姿态 setpoint。

如果要让节点自己请求 OFFBOARD 和解锁，再换成 `real-circle-auto`。
