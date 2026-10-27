# 无人机系统人类操作手册

本文面向使用工作区进行检查、仿真和现场联调的操作者。按“检查环境 → 验证单个子系统 → 连接实机 → 运行任务”的顺序操作。真实飞行须遵守现场飞行负责人和飞控的安全流程；本文不能代替场地审批、飞行检查单或 RC 接管训练。

## 1. 操作前先认识几件事

- 所有命令都从仓库根目录运行。脚本会加载 ROS 2 Foxy 和当前工作区环境，并尊重外部 `ECHO_DRONE_WS`：

  ```bash
  cd /path/to/echo_drone
  echo "$ECHO_DRONE_WS"   # 如果指向其他工作区，用下面带 ECHO_DRONE_WS=$PWD 的命令
  ECHO_DRONE_WS=$PWD ./run_echo_drone.sh help
  ```

- 直接输入 `ros2 topic ...` 的终端也需要加载工作区环境。先执行 `source tools/echo_drone_env.bash`；`./run_echo_drone.sh` 子命令则会自动加载。
- 每个启动命令会占用当前终端并持续输出日志。要看状态时，另开终端运行查询命令。
- 下文把 `check`、`topics`、`sensing-check` 等检查模式视为只读/采样；`*-dry` 用于不向真实硬件写控制的流程演练；带 `real` 的模式可能发布真实 setpoint 或控制舵机。
- `real-point`、`real-swing`、`real-wind` 不会自动 OFFBOARD 或解锁，但需要操作者按现场程序在地面站/遥控器切换模式并解锁。`*-auto` 会请求模式切换和解锁，风险更高。
- 每次只启动一份 MAVROS。确认没有旧 launch 仍占用飞控串口，再启动新的 MAVROS 模式。

## 2. 启动前检查

### 2.1 软件和工作区

```bash
cd /path/to/echo_drone
./run_echo_drone.sh check
```

如果工作区尚未构建，先构建：

```bash
./run_echo_drone.sh build
```

检查当前脚本支持的模式：

```bash
./run_echo_drone.sh help
```

### 2.2 硬件、串口和网络

```bash
./run_echo_drone.sh field-precheck
```

检查项包括串口、设备配置、网络和参数文件位置。接入飞控前确认 Linux 能看到对应设备：

```bash
./run_echo_drone.sh field-precheck
```

现场设备路径或波特率不同时，可在仓库根目录的 `.echo_drone.env` 设置，例如：

```bash
FCU_URL=/dev/ttyACM0:230400
```

`.echo_drone.env` 是本地设置，不要提交个人设备路径、网络地址或凭据。若命令提示无串口权限，检查当前用户是否有设备访问权限；不要让多个程序同时打开同一飞控串口。

### 2.3 Livox 网络

MID360 必须能通过当前 Linux 网络接口与主机通信。运行：

```bash
ip -br addr
ip route
```

主机地址须与 `third_party/livox_ros_driver2/config/MID360_config.json` 中的 host IP 配置相符。WSL、VMware 和实体 Linux 的网卡地址不同，不能照搬别人的地址。配置说明见[现场硬件发布流程](field_hardware_flow.md)和[VMware/Livox 指南](vmware_livox_integration.md)。

## 3. 低风险演练

### 3.1 检查硬件层 dry-run

```bash
./run_echo_drone.sh hardware-dry
```

此模式启动串口状态管理、舵机节点和 MAVROS adapter 的 dry-run 配置。没有硬件时也可检查启动和状态输出。结束时按 `Ctrl+C`。

### 3.2 演练任务决策

```bash
./run_echo_drone.sh mission-dry
```

另开终端查看任务状态：

```bash
cd /path/to/echo_drone
source tools/echo_drone_env.bash
ros2 topic echo /mission/status
```

查看任务图可再开一个终端运行：

```bash
./run_echo_drone.sh mission-graph
```

浏览器访问 `http://127.0.0.1:8765`。dry-run 默认不自动开始任务；按[任务图调试指南](mission_debugging.md)使用 Start、暂停、单步和状态跳转。只有确认 `dry_run=true` 才做硬跳调试。

### 3.3 运行仿真

| 目的 | 命令 | 说明 |
|---|---|---|
| 快速检查 SLS 控制闭环 | `./run_echo_drone.sh sls-circle-sim` | fake MAVROS 点质量仿真，不启动 PX4/Gazebo |
| 查看简化刚体/吊载趋势 | `./run_echo_drone.sh sls-circle-gazebo gui:=true` | Gazebo 力输入模型，不等同于 PX4 飞行器模型 |
| 检查 PX4 Offboard 链路 | `./run_echo_drone.sh sls-px4-sitl` | 启动 PX4 SITL、Gazebo、MAVROS 和控制器；先查看本机 PX4 依赖与参数 |

仿真运行细节及各层覆盖范围见[仿真代码导读](simulation_code_guide.md)和[PX4 SITL 指南](px4_sitl_gazebo.md)。不要把简化 Gazebo 稳定等同于 PX4 SITL 或真实飞行稳定。

## 4. 连接实机：先读状态，再启动传感器

建议每个持续运行的组件单独占一个终端，并记录启动终端与模式。

### 第一步：只读检查 MAVROS

```bash
./run_echo_drone.sh mavros-check
```

确认输出中 `/mavros/state` 有样本且 `connected: true`。若无连接，不要继续启动飞控控制器；先检查串口路径、波特率、飞控端口和 MAVLink heartbeat。详见[MAVROS 运行指南](mavros_runtime_guide.md)。

### 第二步：启动 Livox 和 Point-LIO

在一个终端启动：

```bash
./run_echo_drone.sh sensing
```

另开终端加载环境并查看话题：

```bash
source tools/echo_drone_env.bash
ros2 topic list -t
ros2 topic echo /Odometry
```

若要做限时启动检查，可不启动上面的常驻 `sensing`，单独运行 `CHECK_SECONDS=30 ./run_echo_drone.sh sensing-check`；它会自己启动并结束 sensing 检查流程。`sensing-check` 的默认采样时间是 25 秒，可用 `CHECK_SECONDS` 覆盖。

确认至少能看到下列话题持续更新：

- `/livox/lidar`
- `/livox/imu`
- `/Odometry`
- `/cloud_registered_body`
- `/tf_static`

查看话题和节点：

```bash
./run_echo_drone.sh topics
./run_echo_drone.sh nodes
```

`/Odometry` 没有持续更新或 TF 不完整时，先解决雷达网络、IMU、Point-LIO 参数和 frame 问题。不要把无效/陈旧位姿送入真实控制链路。

### 第三步：需要时启动 MAVROS 与 adapter

当 MAVROS 只读连接正常后，可用以下模式启动硬件层 MAVROS 和 adapter（不启动舵机）：

```bash
./run_echo_drone.sh mavros-real
```

这是与 `mavros-check` 不同的持续运行模式。若已运行其他 MAVROS launch，应先结束旧进程并确认串口释放。adapter 的状态可查看：

```bash
ros2 topic echo /flight_control/status
```

## 5. SLS/QSF 实机操作分级

此流程仅适用于已完成场地准备、无桨检查和飞控安全检查的团队。建议先阅读完整的[SLS 实机启动流程](sls_real_startup.md)。

### 5.1 接入前 dry-run

```bash
./run_echo_drone.sh sls-takeoff-hold-dry
./run_echo_drone.sh sls-circle-mission-dry
```

分别检查起飞后定点状态和起飞后绕圈状态机，不向真实 MAVROS 发布 setpoint。

### 5.2 实机控制档位

| 档位 | 命令 | 功能 |
|---|---|---|
| 位置点 | `./run_echo_drone.sh real-point` | 位置 setpoint，不启用 QSF/抗风补偿，不自动切模式或解锁 |
| 绳摆抑制 | `./run_echo_drone.sh real-swing` | QSF 绕圈，抗风补偿关闭，不自动切模式或解锁 |
| 绳摆抑制与抗风 | `./run_echo_drone.sh real-wind` | QSF 绕圈并启用抗风补偿，不自动切模式或解锁 |

三个入口都可能向真实 MAVROS 发布 setpoint。开始前确认 `/mavros/state connected: true`、位姿新鲜且稳定、遥控器接管有效、桨叶/负载/测试区域检查完成；由操作者按已批准的飞行流程切 OFFBOARD 和解锁。先从位置点档验证，再逐级增加控制功能。

执行中开另一个终端观察：

```bash
./run_echo_drone.sh sls-real-check
source tools/echo_drone_env.bash
ros2 topic echo /sls_circle/status
```

重点关注 MAVROS 连接与模式、位姿输入是否持续、控制器状态的 `reason`、`pose_input_hz`、`command_publish_hz`。如果状态提示 `waiting_for_pose` 或 `pose_stale`，不要继续任务；按现场程序接管并安全结束飞行。

`real-point-auto`、`real-swing-auto`、`real-wind-auto` 会请求 OFFBOARD 和解锁。只有经过独立无桨验证并由飞行负责人明确批准后才使用；不要将它们作为首次实机操作入口。

## 6. 运行时常用观察命令

```bash
./run_echo_drone.sh topics
./run_echo_drone.sh nodes
./run_echo_drone.sh graph
```

关键话题可以单独查看：

```bash
source tools/echo_drone_env.bash
ros2 topic echo /mavros/state
ros2 topic echo /Odometry
ros2 topic echo /robot/current_pose
ros2 topic echo /flight_control/status
ros2 topic echo /hardware/serial_status
ros2 topic echo /servo/status
```

若 ROS CLI 看不到刚启动的节点，确认查询终端和启动终端使用相同工作区、`ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY` 与 RMW 环境；优先使用脚本的 `topics`/`nodes`/`graph` 模式。

## 7. 停止与异常处置

### 仿真、dry-run 或只读检查

在各自启动终端按 `Ctrl+C`，等待 launch 退出后再关闭终端。用下面命令确认节点已退出：

```bash
./run_echo_drone.sh nodes
```

### 真实飞行期间

不要把关闭终端或按 `Ctrl+C` 当作飞行停止按钮。先按现场飞行程序由操作者切换到 RC/安全模式、稳定并降落；确认飞行器已安全后，再停止控制器、adapter 和传感器 launch。异常情况下由飞行负责人执行 RC 接管/急停流程。

### 常见现象

| 现象 | 先检查 |
|---|---|
| `serial:open` / 无 heartbeat | `/dev` 设备是否存在、串口是否被其他程序占用、波特率/飞控端口配置是否匹配；再跑 `mavros-check` |
| 有雷达但无里程计 | Livox host IP 和网卡、`/livox/imu` 是否更新、Point-LIO 日志与输入话题 |
| `/sls_circle/status` 为 `waiting_for_pose` | MAVROS 位姿输入是否存在；`real-*` 入口默认查 `/mavros/mavros/pose`，单独 `sls_circle.launch.py` 默认查 `/mavros/local_position/pose` |
| `pose_stale` | 位姿是否断流、ROS 域/网络是否一致 |
| `not_offboard` / `not_armed` | 控制器未满足输出门槛；由操作者检查飞控状态，不要绕过门控 |
| ROS 图看不到节点 | 两终端 ROS 环境是否一致；再检查是否有残留 daemon 或 launch |

## 8. 按任务查专题文档

- 实机准备和 Livox 网络：[现场硬件发布流程](field_hardware_flow.md)
- MAVROS 连接与话题：[MAVROS 运行指南](mavros_runtime_guide.md)
- SLS/QSF 参数和真实控制：[SLS 实机启动流程](sls_real_startup.md)
- 串口配置与设备状态：[串口管理](serial_hardware_management.md)
- 任务状态图和调试控制：[任务图调试](mission_debugging.md)
- 仿真入口：[仿真代码导读](simulation_code_guide.md)

## 9. 文档适用范围

命令和模式以当前工作区 `./run_echo_drone.sh help` 及对应 launch 实现为准。2026-10-27 的文档与源码一致性结论见[文档一致性审查](documentation_consistency_audit.md)。不同电脑的安装目录、ROS 环境、串口名、雷达网卡和 PX4 路径可能不同。本文是操作步骤索引，不声称本机当前已接入硬件，也不构成构建、飞行或算法性能验证结果。
