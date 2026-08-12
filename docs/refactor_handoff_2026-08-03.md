# Echo Drone 重构交接上下文

本文用于在新的 AI 上下文中继续 `/home/sfx/echo_drone` 的重构任务。

生成时间：2026-08-03。

## 1. 新上下文优先阅读顺序

1. 先读本文，建立当前状态和风险边界。
2. 再读 `docs/refactor_execution_checklist.md`，这是按硬约束整理的主执行清单。
3. 再读 `docs/ARCHITECTURE.md`，区分当前实装架构和后续 HSM 设计稿。
4. 实机链路继续读 `docs/field_hardware_flow.md`、`docs/mavros_runtime_guide.md`、`docs/sls_real_startup.md`。
5. 仿真链路继续读 `docs/px4_sitl_gazebo.md`、`docs/sls_circle_controller.md`。
6. 串口统一管理继续读 `docs/serial_hardware_management.md`。
7. 原始重构设想继续读 `REFACTORING_PLAN.md`，但不要把它当作已完成事实。

## 2. 用户硬约束

- MAVLink 部分必须通过 MAVROS 包装，尽量不用 MAVLink 直连。
- 决策层必须便于调试，后续应支持状态可观察、暂停、单步、回放和上下文导出。
- 清理旧 BT、自定义 navigator、重复包和版本遗留问题。
- TF 单独管理，静态外参、frame 语义和高度偏置不能散落在业务节点中。
- launch 和 config 必须齐全，入口要能覆盖 dry-run、实机、感知、导航、SLS 仿真和 SLS 实机。
- 必须考虑 ROS2 Foxy、Ubuntu 20.04、ARM64 试验机、WSL2 开发机的差异。
- 实机测试要分层推进，先无桨、只读、dry-run，再允许真实 setpoint、解锁、起飞。

## 3. 当前仓库状态速览

- 工作区路径是 `/home/sfx/echo_drone`。
- 当前环境主要是 WSL2 / Ubuntu 20.04 / ROS2 Foxy / amd64，目标试验机预计是 ARM64。
- `run_echo_drone.sh` 是人工联调主入口，负责 source 环境、读取 `.echo_drone.env`、封装常用 launch。
- `.echo_drone.env.example` 提供本地硬件环境变量模板，真实 `.echo_drone.env` 被 git 忽略。
- `echo_drone_v2/` 已删除，不能再把它当作迁移成果。
- 旧 `mavlink_control` 保留为 fallback，但默认路径应优先使用 MAVROS adapter。
- `robot_serial_manager` 已存在，用于统一串口配置、状态发布和 dry-run 占位。
- `sls_circle_controller_cpp` 与 `sls_qsf_core` 已加入，SLS/QSF 控制核心已经从 Python 试验节点推进到 C++ 节点。
- `sls_circle_controller/px4_sitl/` 保存了从 ROS1/drown 迁移来的 Gazebo/PX4 资源和对照源码。

## 4. 已完成或基本落地的重构成果

### 4.1 构建和包清理

- ROS2 Foxy、MAVROS、Nav2、PCL、g2o 等基础依赖在当前开发机上已配置过。
- `third_party/livox_ros_driver2` 已接入，Livox-SDK2 曾安装到 `/usr/local`。
- `third_party/Livox-SDK2` 已用 `COLCON_IGNORE` 避免作为 ROS 包构建。
- 旧 BT navigator、旧 behavior tree、旧 action 包已通过 `COLCON_IGNORE` 从主构建索引移除。
- TEB、Point-LIO、PCL/TF include、`merge_pcd` 等 Foxy 构建问题曾被修复。
- Point-LIO 当前活动树已经做过 Foxy `SharedPtr` 和 callback group 兼容处理。

### 4.2 MAVROS 适配层

- 新增 `flight_control` 包和 `mavros_adapter_node`。
- `robot_bring_up/launch/hardware.launch.py` 可启动 MAVROS、MAVROS adapter、串口管理和旧 fallback。
- `/robot/current_pose`、`/robot/target_pose`、`/cmd_vel`、`/robot/nav_state`、`/robot/passing_door_state`、`/robot/turning_state` 已接入适配层。
- Adapter 支持 `dry_run`，dry-run 下不应调用真实 arming/mode 服务。
- Adapter 发布 `/flight_control/status`。
- 当前仍需要在真实飞控上验证 setpoint、mode、arming、失联保护和 RC 接管。

### 4.3 串口统一管理

- `robot_serial_manager` 已建包，核心节点是 `robot_serial_manager/robot_serial_manager/robot_serial_manager_node.py`。
- 端口配置在 `robot_bring_up/config/hardware/ports.yaml`。
- 当前设计把 PX4 FCU、STM32 舵机、OpenMV 等串口纳入统一配置和状态发布。
- `dry_run` 用于不打开真实串口时发布占位状态，适合无硬件或只查流程。
- WSL2 下飞控通过 `usbipd.exe` 进入 Linux 后可能显示为 `/dev/ttyACM0` 或 `/dev/ttyUSB0`。
- `run_echo_drone.sh` 已有 FCU 设备自动检测逻辑和 `.echo_drone.env` 覆盖机制。
- 曾出现 `/dev/ttyUSB0` 能打开但没有 MAVLink heartbeat 的情况，下一步要检查波特率、飞控端口、MAVLink 参数和 TX/RX/GND。

### 4.4 感知、Point-LIO 和导航

- `robot_bring_up/launch/sensing.launch.py` 是感知/里程计的分层入口。
- 推荐链路是 Livox MID360 -> livox_ros_driver2 -> Point-LIO -> `/Odometry`、`/cloud_registered`、`/path`。
- WSL2 下 Livox 网络曾验证过一个可行方向：mirrored networking、Windows Ethernet `192.168.1.5/24`、MID360 `192.168.1.12`。
- `lidar_nav_stack.launch.py` 曾用于统一 Nav2 lifecycle，避免分裂 lifecycle manager 卡住。
- RViz 目标工具曾从 ROS1 风格 `/move_base_simple/goal` 调整到 Nav2 `/navigate_to_pose`。
- 最终真实导航目标闭环仍需要现场重新验证。

### 4.5 SLS/QSF 控制与仿真

- `sls_circle_controller_cpp` 是当前优先使用的新版 C++ 控制器。
- `sls_qsf_core` 保存由原算法移植或生成出的 QSF 控制核心。
- `sls-circle-gazebo` 是简化 Gazebo 力输入烟测，不包含 PX4 内环和电机模型。
- `sls-px4-sitl` 是 Gazebo Classic + PX4 SITL + MAVROS + SLS 控制器链路。
- `sls-px4-sitl` 默认 `start_controller:=true`、`mission_mode:=takeoff_then_circle`、`controller_mode:=qsf`、`enable_real_setpoint:=true`、`auto_offboard:=true`、`auto_arm:=true`。
- `sls-px4-sitl` 默认带起飞后绕圈流程，但抗风补偿增益是否生效要继续核对 launch 是否透传 `wind_compensation_gain`。
- 风估计默认采用 residual 思路，仿真比较模式和真实补偿模式需要继续区分。
- 当前要求已经明确：风力禁止对 z 轴修正，LESO/抗风估计也应禁用 z 轴输出。

## 5. 仍未完成的主线任务

### 5.1 MAVROS 实机链路

- 实机上确认 MAVROS heartbeat。
- 固化 `FCU_URL`、波特率、飞控 MAVLink 端口参数。
- 验证 `/mavros/state`、`/mavros/local_position/pose`、`/mavros/setpoint_*`、arming、set_mode。
- 无桨验证 OFFBOARD、arm、takeoff setpoint、position setpoint、velocity setpoint。
- 建议保留三个实机任务档：单纯发点、只消绳摆、绳摆加抗风。

### 5.2 TF 单独管理

- 还没有完成 `robot_tf_manager` 独立包。
- 需要收敛 `map -> odom`、`odom -> livox_raw`、`livox_raw -> livox`、`livox -> camera_link`、`livox -> mavlink_body`。
- 需要清理 `behavior_control.cpp`、`mavlink_control_node.py`、`obstacle_segmentation.cpp`、launch 文件中的高度偏置和静态 TF 数字。
- 需要提供 `/tf_manager/status` 或等价健康检查，TF 缺失时禁止进入真实飞控控制。

### 5.3 决策层重构

- `behavior_control.cpp` 仍是大文件 step 状态机，尚未完成 HSM 化。
- 后续目标是 `mission_control` 或重构后的 `behavior_control`，拆成状态机、上下文、目标管理、导航客户端、飞控客户端、舵机客户端、调试发布器。
- 必须保留旧 `/robot/*` 接口，避免一次性破坏飞控和导航联调。
- 应新增 `/mission/status`、`/mission/event`、`/mission/start`、`/mission/pause`、`/mission/resume`、`/mission/step_once`、`/mission/abort`、`/mission/dump_context`。
- `tick()` 不应阻塞等待 service/action，所有动作应异步化。

### 5.4 SLS 算法和 PX4 SITL

- 需要把 `sls-px4-sitl` 拆出更明确的启动档位。
- 建议新增或确认以下档位：`sls-px4-point`、`sls-px4-swing`、`sls-px4-wind`。
- `sls-px4-point` 只做起飞、定点、绕圈发点或可先只定点，不施加 QSF/抗风补偿。
- `sls-px4-swing` 使用 QSF/绳摆抑制，不启用风扰补偿。
- `sls-px4-wind` 使用 QSF/绳摆抑制和 XY 风扰估计补偿，z 轴风补偿保持 0。
- 需要验证 launch 透传 `wind_compensation_gain`、`wind_estimator_mode`、`use_load_pose`、`qsf_reference_is_load` 等参数。
- 需要用 ROS1/drown 版本对照公式、坐标系、质量参数、绳长、推力归一化和 ENU/NED 映射。

### 5.5 ARM64 和试验机适配

- 当前开发机不是试验机，不能把 WSL2 通过结果当作实机结果。
- ARM64 上需要确认 MAVROS、Livox-SDK2、livox_ros_driver2、Point-LIO、Nav2、TEB、Gazebo 依赖。
- 试验机建议先按 “只读 -> dry-run -> 无桨 -> 系绳低高度 -> 正式绕圈” 逐级验证。

## 6. 推荐下一步执行顺序

1. 先冻结当前文档和脚本入口，确保新上下文不会重复做已经做过的清理。
2. 修正 `sls-px4-sitl` 参数透传，让 PX4 SITL 明确支持 point、swing、wind 三档。
3. 跑一遍 headless PX4 SITL，采样 `/mavros/state`、`/sls_circle/status`、`/mavros/setpoint_raw/attitude`。
4. 若 PX4 SITL 能起飞，再观察绕圈是否进入、是否发散、是否有 z 轴风补偿残留。
5. 回到实机链路，只连飞控时先解决 MAVROS heartbeat 和 baud 问题。
6. 飞控 heartbeat 稳定后，再接 Point-LIO，验证 `/Odometry` 到 MAVROS vision pose 的坐标映射。
7. 最后再动 `behavior_control.cpp` 的大重构，不要在硬件链路未稳定前大范围改任务状态机。

## 7. 常用入口

```bash
./run_echo_drone.sh check
./run_echo_drone.sh mavros-real
./run_echo_drone.sh mavros-baud-scan
./run_echo_drone.sh hardware-dry
./run_echo_drone.sh sensing
./run_echo_drone.sh lidar-nav
./run_echo_drone.sh sls-circle-gazebo
PX4_DIR=/home/sfx/PX4-Autopilot ./run_echo_drone.sh sls-px4-sitl
./run_echo_drone.sh real-point
./run_echo_drone.sh real-swing
./run_echo_drone.sh real-wind
```

## 8. 现场硬件提醒

- Windows 端用 `usbipd.exe list` 看 BUSID 和 COM 号。
- 地面站在 Windows 里选 COM 号，例如 `COM17`，不是 BUSID。
- WSL2 里 attach 后看 `/dev/ttyACM*` 或 `/dev/ttyUSB*`。
- 如果 MAVROS 能 open 串口但没有 `CON: Got HEARTBEAT`，说明串口设备存在但 MAVLink 没通。
- 先查 baud，再查是否接错端口，再查 TX/RX/GND，再查飞控 MAVLink 参数。
- 若端口要还给 Windows 地面站，先停止 WSL 内 MAVROS，再 detach/unbind 对应 usbipd 设备。

## 9. 记忆上下文摘要

以下内容来自此前 Codex 记忆和本仓库文件状态，可能随仓库变动过期，新上下文必须用当前文件和运行结果复核。

- 记忆显示，MID360/Point-LIO/RViz/Nav2 曾在 WSL2 mirrored networking 下被分层调通过，Livox host IP 使用过 `192.168.1.5`，雷达 IP 使用过 `192.168.1.12`。
- 记忆显示，`sensing.launch.py`、`lidar_nav_stack.launch.py`、`tools/echo_drone_env.bash`、`.echo_drone.env` 是后续感知和导航联调的重要入口。
- 记忆显示，原始 V2/HSM 重构文档和 `echo_drone_v2/` 不代表已完成迁移，报告和方案必须区分计划、脚手架、已验证代码。
- 记忆显示，SLS residual 风估计曾在禁用补偿时达到过 x/y 相对误差小于 30% 的目标，但闭环补偿仍需要 warmup、ramp、限幅和稳定性验证。
- 记忆显示，Gazebo `127.0.0.1:7890` 代理问题曾通过 `prepare_gazebo_env()` 对 SLS Gazebo workflow 做过窄范围处理。
- 记忆显示，Point-LIO Foxy 兼容的活跃树是 `/home/sfx/echo_drone/Point-LIO`，不要误改其他工作区里的 Point-LIO 副本。
- 记忆显示，用户对 MAVROS 迁移的硬要求是“不裸连”，恢复任务时必须先定位并拒绝新增 raw MAVLink 直连。

## 10. 新上下文开场建议

把下面这段直接交给新的上下文即可：

```text
请先阅读 /home/sfx/echo_drone/docs/refactor_handoff_2026-08-03.md，然后按文档里的优先级继续重构。
不要把 REFACTORING_PLAN.md 当作已完成事实。
当前硬约束是 MAVROS 包装、不裸连 MAVLink、决策层可调试、TF 单独管理、清理旧 BT/遗留包、完善 launch/config、兼顾 Foxy 和 ARM64。
下一步优先把 sls-px4-sitl 拆成单纯发点、只消摆、消摆加抗风三档，并验证 PX4 SITL 的 MAVROS 状态、setpoint、起飞、定点和绕圈。
实机链路优先解决 MAVROS heartbeat、FCU_URL、波特率和 Point-LIO 外部里程计输入。
```

