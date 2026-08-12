# QSF 起飞后绕圈实机启动流程

## 目标

这份流程用于实机启动新接入的 `drown` QSF/SLS 控制算法，并先执行“起飞确认后定点”，确认稳定后再进入匀速圆周运动。

控制主线只通过 MAVROS 读取飞控状态和发布 setpoint，不再裸连 MAVLink。

## 启动前提

实机启动前必须先确认下面三条链路已经通。

```bash
cd /home/sfx/echo_drone
./run_echo_drone.sh field-precheck
./run_echo_drone.sh mavros-check
CHECK_SECONDS=30 ./run_echo_drone.sh sensing-check
```

通过标准：

- `/mavros/state` 能采到消息，且 `connected: true`。
- `/livox/lidar` 和 `/livox/imu` 能持续发布。
- `/Odometry` 能持续发布。
- `/cloud_registered_body` 能持续发布。
- `/tf_static` 里有 `map -> odom`、`livox_raw -> livox`、`livox -> mavlink_body`。

## 本地变量整合

`run_echo_drone.sh` 会自动从 `/dev/px4_fcu`、`/dev/ttyACM*`、`/dev/ttyUSB*` 中选择飞控串口，并默认使用 `230400` 波特率。

如果现场设备名固定，可以在工作区根目录的 `.echo_drone.env` 中写入下面内容，脚本会自动读取。

```bash
FCU_URL=/dev/ttyACM0:230400
```

如果只想改波特率，不想固定设备名，可以只写下面内容。

```bash
FCU_BAUD=230400
```

## 三档实机测试入口

当前实机流程拆成三档，先验证 MAVROS 发送链路，再逐步加入控制算法。

| 推荐入口 | 兼容入口 | 输出到 MAVROS | 控制含义 | 自动 OFFBOARD/解锁 |
| --- | --- | --- | --- | --- |
| `real-point` | `sls-point-real` | `/mavros/setpoint_position/local` | 普通位置点，不启用 QSF 和抗风扰 | 否 |
| `real-swing` | `sls-swing-real` | 起飞段位置点，绕圈段 `/mavros/setpoint_raw/attitude` | QSF 绳摆抑制，抗风扰关闭 | 否 |
| `real-wind` | `sls-swing-wind-real` | 起飞段位置点，绕圈段 `/mavros/setpoint_raw/attitude` | QSF 绳摆抑制加残差抗风扰 | 否 |

对应的自动入口是 `real-point-auto`、`real-swing-auto` 和 `real-wind-auto`。

自动入口会请求 `/mavros/set_mode` 和 `/mavros/cmd/arming`。

旧入口 `sls-takeoff-hold-real` 保留为起飞后定点别名，旧入口 `sls-circle-real` 保留为 `real-wind` 的兼容入口。

## 当前算法

三档入口的默认参数如下。

```text
real-point: controller_mode:=pd, enable_anti_wind:=false, mission_mode:=takeoff_then_hold
real-swing: controller_mode:=qsf, enable_anti_wind:=false, mission_mode:=takeoff_then_circle
real-wind: controller_mode:=qsf, enable_anti_wind:=true, mission_mode:=takeoff_then_circle
```

算法来源：

- `sls_qsf_core`：从 `drown` 拆出的 QSF/SLS 共享库。
- `sls_circle_controller_cpp`：默认 C++ ROS2/MAVROS 控制节点。
- `sls_circle_controller`：保留 Python fake MAVROS、Gazebo 桥接和历史 Python 控制器，用于仿真辅助。

可选算法模式：

- `qsf`：默认 QSF 控制。
- `qsf_integral`：QSF 积分版本。
- `pd`：普通 PD 回退。
- `leso_pd`：PD 加扰动估计。

## 决策状态机

实机圆周任务使用 `mission_mode:=takeoff_then_circle`。

状态顺序：

```text
preflight -> takeoff -> hold -> circle
```

状态含义：

- `preflight`：启动后的预热阶段，持续 `preflight_setpoint_time` 秒。
- `takeoff`：发布起飞位置参考，目标高度为 `takeoff_altitude`。
- `hold`：高度达到 `takeoff_altitude - takeoff_z_tolerance` 后，先按 `takeoff_settle_time + post_takeoff_hold_time` 定点。
- `circle`：定点时间完成后锁定当前位置为圆心并开始匀速绕圈。

如果只想检查能不能定住，使用 `mission_mode:=takeoff_then_hold`，状态机会停在 `hold`，不会进入 `circle`。

进入 `circle` 后，节点会在日志里打印：

```text
takeoff hold confirmed; entering circle tracking
```

## 安全行为

`real-point`、`real-swing` 和 `real-wind` 不会自动请求 OFFBOARD，也不会自动解锁。

`real-point` 只发布 `/mavros/setpoint_position/local`，用于验证“点位 setpoint -> MAVROS -> 飞控”的基础链路。

`real-swing` 和 `real-wind` 在起飞和定点阶段会发布 `/mavros/setpoint_position/local`，进入绕圈阶段后才会按安全门发布 `/mavros/setpoint_raw/attitude`。

`sls-takeoff-hold-real` 只发布起飞和定点位置 setpoint，不进入绕圈，也不会写 `/mavros/setpoint_raw/attitude`。

绕圈姿态 setpoint 必须同时满足：

```text
dry_run=false
enable_real_setpoint=true
/mavros/state connected=true
/mavros/state mode=OFFBOARD
/mavros/state armed=true
flight_stage=circle
```

如果飞控没有进入 OFFBOARD 或未解锁，节点仍会发布调试话题，但不会把绕圈姿态 setpoint 写到真实 MAVROS 控制话题。

## 推荐实机启动

第一终端，启动 MAVROS 和 adapter。

```bash
cd /home/sfx/echo_drone
./run_echo_drone.sh mavros-real
```

第二终端，启动 Livox 和 Point-LIO。

```bash
cd /home/sfx/echo_drone
./run_echo_drone.sh sensing
```

第三终端，先 dry-run 看状态机和算法输出。

```bash
cd /home/sfx/echo_drone
./run_echo_drone.sh sls-takeoff-hold-dry
./run_echo_drone.sh sls-circle-mission-dry
```

第四终端，只验证位置点到 MAVROS 的链路。

```bash
cd /home/sfx/echo_drone
./run_echo_drone.sh real-point
```

第五终端，确认点位链路和定点状态稳定后，再试只消绳摆的 QSF 绕圈。

```bash
cd /home/sfx/echo_drone
./run_echo_drone.sh real-swing
```

第六终端，确认只消绳摆版本稳定后，再试绳摆加抗风扰。

```bash
cd /home/sfx/echo_drone
./run_echo_drone.sh real-wind
```

这三个非 auto 入口都要求飞控已经由遥控器、QGC 或地面站切到 OFFBOARD 并解锁。

运行过程中可以用只读检查入口采样关键话题。

```bash
cd /home/sfx/echo_drone
./run_echo_drone.sh sls-real-check
```

## 自动 OFFBOARD 和解锁入口

只有在台架和安全流程都确认后，才使用自动入口。

```bash
cd /home/sfx/echo_drone
./run_echo_drone.sh real-point-auto
```

然后再按风险顺序试 `real-swing-auto` 和 `real-wind-auto`。

这些入口会请求 `/mavros/set_mode` 和 `/mavros/cmd/arming`。

## 常用参数覆盖

降低绕圈速度和半径：

```bash
./run_echo_drone.sh real-circle radius:=0.5 angular_velocity:=0.2
```

提高起飞高度：

```bash
./run_echo_drone.sh real-circle takeoff_altitude:=1.2
```

延长起飞后的定点时间：

```bash
./run_echo_drone.sh real-circle post_takeoff_hold_time:=10.0
```

临时提高控制器频率：

```bash
./run_echo_drone.sh real-circle control_rate:=150.0
```

只做起飞后定点：

```bash
./run_echo_drone.sh real-point
```

切换到 QSF 积分版本：

```bash
./run_echo_drone.sh real-circle controller_mode:=qsf_integral
```

临时关闭抗风扰：

```bash
./run_echo_drone.sh real-swing
```

抗风扰开启时只修正 ENU 的 x/y 平面，z 方向的实际风、估计风和补偿风均固定为 0。

实机入口默认 `wind_estimator_mode:=residual`，输出的是由运动残差和负载耦合补偿推断出的等效外扰力，不是直接测得的真实风。

Gazebo 里如果临时切 `wind_estimator_mode:=actual_feedback`，它会直接跟踪 `/sls_circle/wind_actual` 这个代码真值，只用于仿真核对，不作为默认预测模式。

## 现场观察命令

看 MAVROS 状态。

```bash
ros2 topic echo /mavros/state mavros_msgs/msg/State
```

看 Point-LIO 里程计。

```bash
ros2 topic echo /Odometry nav_msgs/msg/Odometry
```

看控制器状态。

```bash
ros2 topic echo /sls_circle/status std_msgs/msg/String
```

重点看 `actual_control_hz`、`command_publish_hz` 和 `pose_input_hz`。

`actual_wind_input_hz` 只在 Gazebo 有 `/sls_circle/wind_actual` 时有效，实机通常应为 0。

看真实姿态 setpoint。

```bash
ros2 topic echo /mavros/setpoint_raw/attitude mavros_msgs/msg/AttitudeTarget
```

看调试姿态 setpoint。

```bash
ros2 topic echo /sls_circle/debug/attitude_target mavros_msgs/msg/AttitudeTarget
```

看参考圆周轨迹。

```bash
ros2 topic echo /sls_circle/reference_pose geometry_msgs/msg/PoseStamped
```

看抗风扰估计。

```bash
ros2 topic echo /sls_circle/wind_estimate geometry_msgs/msg/Vector3Stamped
```

这个话题单位是 N，x/y 是估计外扰力，z 固定为 0。

如果现场 MAVROS 位姿话题不是默认名，可以在入口后直接覆盖话题。

```bash
./run_echo_drone.sh real-point pose_topic:=/mavros/mavros/pose
```

## 失败时先看什么

如果 `/sls_circle/status` 里 `reason` 是 `waiting_for_pose`，先查 `/mavros/local_position/pose`。

如果 `reason` 是 `pose_stale`，说明 MAVROS 位姿断流。

如果 `reason` 是 `not_offboard`，说明飞控还没进入 OFFBOARD。

如果 `reason` 是 `not_armed`，说明飞控还没解锁。

如果 `qsf_core_loaded` 是 `false`，重新构建 `sls_qsf_core` 和 `sls_circle_controller`。

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select sls_qsf_core sls_circle_controller_cpp sls_circle_controller --symlink-install
source install/setup.bash
```

## 禁止跳步

不要在 `/mavros/state connected` 不是 `true` 时跑真实圆周控制。

不要在 `/Odometry` 不稳定时上真实圆周控制。

不要直接从 `real-wind-auto` 或 `real-circle-auto` 开始试飞。

不要在装桨状态下做首次参数验证。
