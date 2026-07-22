# MAVROS Runtime Guide

本文记录当前仓库从旧 `mavlink_control` 直连 MAVLink 迁移到 MAVROS 后的实际运行方式、实测话题和待处理问题。

## 当前实测状态

测试日期：2026-07-23。

测试环境：

- Ubuntu 20.04 / ROS2 Foxy。
- 当前机器是 WSL2 x86_64，不是 ARM 试验机。
- 飞控通过 Windows `usbipd` 透传到 WSL。
- WSL 侧飞控串口为 `/dev/ttyACM0`。
- MAVROS 连接参数为 `/dev/ttyACM0:230400`。

只读连接测试命令：

```bash
cd /home/sfx/echo_drone
FCU_URL=/dev/ttyACM0:230400 ./run_echo_drone.sh mavros-state
```

实测成功日志：

```text
link[1000] opened successfully
link[1000] detected remote address 1.1
CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot
```

结论：

- USB 透传已经可用。
- 串口权限已经临时放开过。
- 波特率 `230400` 可连接当前 PX4 飞控。
- MAVROS 可以收到 PX4 heartbeat。
- 该测试没有启动 `mavros_adapter`，没有解锁，没有切模式，没有发布运动指令。

## Windows/WSL 串口准备

管理员 PowerShell：

```powershell
usbipd list
usbipd bind --force --busid 1-3
usbipd attach --wsl --busid 1-3
```

WSL：

```bash
cd /home/sfx/echo_drone
./run_echo_drone.sh check
ls -l /dev/ttyACM* /dev/ttyUSB*
```

如果用户不在 `dialout` 组，临时测试可以执行：

```bash
sudo chmod a+rw /dev/ttyACM0
```

长期处理方式：

```bash
sudo usermod -aG dialout sfx
```

执行后需要重启 WSL 或重新登录，才能让用户组生效。

## 旧接口到 MAVROS 的映射

旧链路：

```text
behavior_control -> /robot/target_pose or /cmd_vel -> mavlink_control -> pymavlink -> FCU
```

新链路：

```text
behavior_control -> /robot/target_pose or /cmd_vel -> flight_control/mavros_adapter -> MAVROS -> FCU
```

`flight_control/mavros_adapter_node` 不打开串口，只通过 MAVROS topic 和 service 工作。

| 旧接口 | 新 MAVROS 接口 | 方向 | 说明 |
|---|---|---|---|
| `/robot/current_pose` | `/mavros/vision_pose/pose` | adapter -> MAVROS | 向 PX4 喂视觉位姿 |
| `/robot/target_pose` | `/mavros/setpoint_position/local` | adapter -> MAVROS | 非导航模式的位置设定点 |
| `/cmd_vel` | `/mavros/setpoint_velocity/cmd_vel` | adapter -> MAVROS | 导航模式速度设定点 |
| `/mavros/state` | `/robot/arm_state` | MAVROS -> adapter -> legacy | 兼容旧决策代码的解锁状态 |
| `/mavros/state` | `/flight_control/status` | MAVROS -> adapter -> debug | JSON 调试状态 |
| `/mavros/rc/in` | `/mavros/cmd/arming` | MAVROS -> adapter -> MAVROS service | 摇杆组合触发解锁，`dry_run=false` 时才会调用 |
| adapter 内部状态 | `/mavros/set_mode` | adapter -> MAVROS service | `auto_set_mode=true` 且 `dry_run=false` 时才会调用 |

## MAVROS 实测话题

当前只读连接测试中，`ros2 topic list` 看到的关键 MAVROS 话题如下。

核心状态：

- `/mavros/state`
- `/mavros/battery`
- `/mavros/extended_state`
- `/mavros/estimator_status`
- `/mavros/statustext/recv`
- `/mavros/statustext/send`
- `/mavros/time_reference`
- `/mavros/timesync_status`
- `/mavros/vfr_hud`
- `/mavros/radio_status`

位姿、定位、里程计相关：

- `/mavros/local_position/pose`
- `/mavros/global_position/global`
- `/mavros/global_position/gp_origin`
- `/mavros/home_position/home`
- `/mavros/mavros/odom`
- `/mavros/mavros/local`
- `/mavros/mavros/global`
- `/mavros/mavros/velocity_local`
- `/mavros/mavros/velocity_body`
- `/mavros/mavros/imu_ned`

控制和设定点相关：

- `/mavros/actuator_control`
- `/mavros/target_actuator_control`
- `/mavros/mavros/target_local`
- `/mavros/mavros/target_global`
- `/mavros/mavros/target_attitude`
- `/mavros/mavros/set`
- `/mavros/mavros/set_gp_origin`
- `/mavros/mavros/rc_inputs`
- `/mavros/play_tune`

MAVLink router 端点：

- `/uas1/mavlink_source`
- `/uas1/mavlink_sink`

注意：

- `/mavros/state` 实测类型为 `mavros_msgs/msg/State`，发布者为 namespace `/mavros` 下的 `mavros` 节点。
- 当前还观察到了大量 `/mavros/mavros/...` 嵌套话题，这是 MAVROS 2.4.0 在当前启动方式下的实际表现。
- 这些嵌套话题不要先当成稳定业务接口，业务代码优先使用 `mavros_adapter.yaml` 里显式配置的 MAVROS topic。

查看状态建议使用：

```bash
source /opt/ros/foxy/setup.bash
source /home/sfx/echo_drone/install/setup.bash
ros2 topic info -v /mavros/state
ros2 topic echo /mavros/state mavros_msgs/msg/State --qos-durability transient_local
```

## 未解决问题

1. MAVROS 默认插件会刷 TF 错误。

当前日志中高频出现：

```text
ODOM: Ex: Could not find a connection between 'map' and 'base_link_frd' because they are not part of the same tree.
```

这说明 MAVROS odometry 插件在找 `map -> base_link_frd`，但当前只读测试没有启动完整 TF 树。

2. MAVROS 插件白名单尚未可靠收窄。

ROS2 Foxy 下直接用 `/opt/ros/foxy/share/mavros/launch/px4.launch` 会失败，因为该文件仍带 ROS1 风格 `$(find mavros)` 写法。

直接把 `px4_pluginlists.yaml` 当 ROS2 参数加载也会失败，因为它不是 `node: ros__parameters:` 格式。

目前 `mavros_state.launch.py` 使用原生 Python `Node` 启动 `mavros_node`，可以连通飞控，但插件过滤还没有最终定型。

3. `hardware-real` 已改为走 `flight_control` 的 Python MAVROS wrapper，但还没有做完整无桨实机联调。

已经验证的是 `mavros-state` 只读连通。

还没有验证的是 `hardware-real` 同时启动 `mavros_adapter` 后的 setpoint、arming service、mode service 和失联保护。

4. 坐标和高度偏移仍需实机校验。

`mavros_adapter` 当前默认 `coordinate_mode=legacy_ned_compatible`，目的是兼容旧 `mavlink_control` 的符号和高度语义。

但仓库里同时存在 `map -> odom` 静态 z 偏移、Livox 安装偏移、`vision_pose_z_offset`、`fcu_reference_z_offset` 等配置，后续必须在无桨状态下核对高度和 ENU/NED 方向。

5. WSL 串口权限是临时处理。

当前测试中用过 `chmod a+rw /dev/ttyACM0`。

重启、重插、重新 attach 后权限可能恢复为 `root:dialout`，应使用 `dialout` 用户组或 udev 规则长期处理。

6. 真机控制前还缺少安全门。

`mavros_adapter` 已有 `dry_run`、状态发布和 setpoint 新鲜度检查，但还需要在实机前确认：

- MAVROS disconnected 时拒绝进入任务。
- 起飞前 setpoint 预热。
- OFFBOARD 切换策略。
- 解锁触发条件。
- 失去 `/robot/current_pose` 后的行为。

## 所有启动方法

所有命令默认从仓库根目录执行：

```bash
cd /home/sfx/echo_drone
```

### 环境和构建

检查环境、包列表和串口：

```bash
./run_echo_drone.sh check
```

构建整个 workspace：

```bash
./run_echo_drone.sh build
```

查看当前 ROS topic：

```bash
./run_echo_drone.sh topics
```

### 低风险 dry-run

启动串口管理器、舵机节点和 MAVROS adapter，但不连接真实 MAVROS，不写真实舵机串口：

```bash
./run_echo_drone.sh hardware-dry
```

只启动串口管理器：

```bash
./run_echo_drone.sh serial-dry
```

只启动舵机节点 dry-run：

```bash
./run_echo_drone.sh servo-dry
```

只启动 MAVROS adapter dry-run：

```bash
./run_echo_drone.sh adapter-dry
```

### 飞控只读连接

只启动 MAVROS，不启动 adapter，不发控制指令：

```bash
FCU_URL=/dev/ttyACM0:230400 ./run_echo_drone.sh mavros-state
```

如果设备名不同：

```bash
FCU_URL=/dev/ttyUSB0:230400 ./run_echo_drone.sh mavros-state
```

等价原始 launch：

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch flight_control mavros_state.launch.py fcu_url:=/dev/ttyACM0:230400
```

### 实机硬件层

启动串口管理器、MAVROS 和 MAVROS adapter，但不启动舵机：

```bash
FCU_URL=/dev/ttyACM0:230400 ./run_echo_drone.sh mavros-real
```

启动串口管理器、舵机、MAVROS 和 MAVROS adapter：

```bash
FCU_URL=/dev/ttyACM0:230400 ./run_echo_drone.sh hardware-real
```

只启动真实舵机节点：

```bash
./run_echo_drone.sh servo-real
```

等价原始 launch：

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch robot_bring_up hardware.launch.py dry_run:=false use_mavros:=true use_servo:=false fcu_url:=/dev/ttyACM0:230400
```

### 传感器、里程计、点云和导航

启动 Livox MID360 驱动：

```bash
./run_echo_drone.sh livox
```

启动 Point-LIO：

```bash
./run_echo_drone.sh pointlio
```

启动点云分割：

```bash
./run_echo_drone.sh obstacle
```

启动任务决策：

```bash
./run_echo_drone.sh behavior
```

启动 Nav2：

```bash
./run_echo_drone.sh nav
```

启动当前整机入口并关闭 RViz：

```bash
./run_echo_drone.sh full
```

### 建议联调顺序

1. `./run_echo_drone.sh check`
2. `./run_echo_drone.sh hardware-dry`
3. `FCU_URL=/dev/ttyACM0:230400 ./run_echo_drone.sh mavros-state`
4. `./run_echo_drone.sh livox`
5. `./run_echo_drone.sh pointlio`
6. `./run_echo_drone.sh obstacle`
7. `./run_echo_drone.sh behavior`
8. `FCU_URL=/dev/ttyACM0:230400 ./run_echo_drone.sh mavros-real`
9. `FCU_URL=/dev/ttyACM0:230400 ./run_echo_drone.sh hardware-real`

不要在 `mavros-state` 和 TF 检查未通过前直接跑 `full` 做飞行联调。
