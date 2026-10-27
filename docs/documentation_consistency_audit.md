# 文档一致性审查（2026-10-27）

## 审查目的与方法

本文记录一次“文档对源码”的静态核对，目标是回答三个问题：

1. 当前系统实际由哪些 ROS 2 包、launch、topic、TF 和配置组成；
2. `docs/` 中哪些文档描述当前实现，哪些只是设计稿、历史记录或操作计划；
3. 文档中的命令、接口名和安全边界是否与当前代码一致。

审查依据按优先级如下：

1. `run_echo_drone.sh help`、launch 文件和节点源码；
2. `package.xml`、`CMakeLists.txt`、`setup.py` 和 YAML 参数；
3. 当前 `docs/` 内部说明及其标注的日期；
4. 历史交接记录和设计稿只作为背景，不作为当前状态证明。

本次没有连接飞控、雷达或舵机，没有执行实机控制、完整全栈启动或飞行验证。因此本文只给出静态代码与文档的一致性结论，不证明硬件链路可用。

## 当前环境快照

以下环境信息由本次审查命令直接得到：

| 项目 | 当前值 |
|---|---|
| 系统 | Ubuntu 20.04.6 LTS |
| 架构 | `aarch64`（Tegra 内核） |
| Python | 3.8.10 |
| ROS 2 | Foxy |
| 工作区 ROS 包数 | 19 个 `colcon list` 可见包 |
| 已接入硬件 | 本次检查未发现匹配串口设备，未做硬件验证 |

当前 `colcon list --names-only` 结果：

```text
behavior_control
costmap_converter
costmap_converter_msgs
flight_control
livox_ros_driver2
mavlink_control
merge_pcd
obstacle_segmentation
pcd2pgm
point_lio
robot_bring_up
robot_interfaces
robot_serial_manager
servo_node
sls_circle_controller
sls_circle_controller_cpp
sls_qsf_core
teb_local_planner
teb_msgs
```

注意：`system_contract.md` 等较早文档中的“14 个包”和“WSL2/amd64 当前环境”是历史快照，不是现在的环境结论。

## 当前源码确认的核心接口

### 感知与定位

- `Point-LIO` 发布 `/Odometry`、`/robot/current_pose`、`/cloud_registered`、`/cloud_registered_body`、`/cloud_obstacle_new`、`/Laser_map`、`/path` 和 `planner_normal`。
- `drone.yaml` 当前设置 `laser_mapping.common.map_frame: "odom"`、`common.odom_frame: "livox"`。因此 Point-LIO 广播和 `/robot/current_pose` 的实际父/子 frame 是 `odom -> livox`，不是 `map -> livox`。
- `sensing.launch.py` 额外发布静态 TF：`map -> odom`（z=0.39）、`livox -> mavlink_body`（含旋转修正）、`livox -> camera_link`。当前没有 `livox_raw -> livox` 静态 TF。

### 飞控适配

- 当前默认飞控主线是 `flight_control/mavros_adapter_node`，不是 `mavlink_control`；后者保留为显式 fallback。
- `flight_control/config/mavros_adapter.yaml` 当前实际端点为：
  - `/robot/current_pose` -> `/mavros/mavros/pose`
  - `/robot/target_pose` -> `/mavros/mavros/local`
  - `/cmd_vel` -> `/mavros/mavros/cmd_vel`
  - 解锁服务 `/mavros/mavros/arming`
  - 模式服务 `/mavros/set_mode`
- `/mavros/vision_pose/pose`、`/mavros/setpoint_position/local`、`/mavros/setpoint_velocity/cmd_vel` 是代码内某些默认参数或旧文档使用的通用名称，但不是当前 `mavros_adapter.yaml` 的默认输出端点。现场必须以 `ros2 topic info -v` 的结果为准。

### 导航与感知组合

- `sensing.launch.py` 的 `launch_costmap:=true` 组合启动障碍物分割、controller/planner 和 `sensing_costmap.yaml`。该 YAML 的 obstacle layer 当前直接订阅 `/cloud_registered_body`，不订阅 `/cloud_obstacle`。
- `sensing_nav.launch.py` -> `lidar_nav_stack.launch.py` 使用 `drone.yaml` 中的 costmap 参数，obstacle layer 订阅 `/cloud_obstacle`。因此只有 `sensing-nav` 路径实际把障碍物分割结果接入 Nav2 costmap。
- `obstacle_segmentation` 当前使用 `/robot/obstacle_height` 做高度筛选；`/robot/clear_state` 只被回调缓存，`clear_state` 的布尔值目前没有参与清图逻辑。`base_frame` 和 `use_downsample` 参数也未被实际使用。

### 任务决策

- `behavior_control` 当前仍由两个 250 ms/100 ms 定时器推进：
  - `step_timer_callback()`：旧数字 step 的条件转移；
  - `mission_timer_callback()`：按当前 step 发布目标、Action 请求、模式标志和舵机命令。
- 新增的命名图、`state_id`、任务状态/事件发布和调试服务已经落地；但正常任务转移仍由旧 step 分支决定，图主要用于观测和安全硬跳。
- `dry_run=true` 时飞控、导航和舵机命令被禁止；硬跳还要求执行器处于 `idle` 或 `paused`。

### SLS/QSF

- 当前主控制节点是 `sls_circle_controller_cpp/src/sls_circle_controller_node.cpp`。
- 单独 `sls_circle.launch.py` 的默认端点仍是 `/mavros/local_position/pose`、`/mavros/setpoint_position/local` 和 `/mavros/setpoint_raw/attitude`。
- `run_echo_drone.sh` 的 `real-*`/`sls-*real` 模式会把 `pose_topic` 覆盖为 `/mavros/mavros/pose`，把 `takeoff_pose_topic` 覆盖为 `/mavros/mavros/local`；部分自动入口还会覆盖 arming service。
- `sls-real-check` 当前仍采样旧通用端点 `/mavros/local_position/pose` 和 `/mavros/setpoint_position/local`，无法完整覆盖 `real-*` 入口实际使用的 `/mavros/mavros/*` 端点。这是已发现的工具与启动入口不一致点。

### 自定义 ROS 接口

`robot_interfaces` 当前生成：

- `msg/ImageLocation.msg`
- `msg/ServoPos.msg`
- `msg/MissionStatus.msg`
- `msg/MissionEvent.msg`
- `srv/JumpMissionState.srv`
- `srv/ShiftMissionState.srv`

旧文档只列前两个消息，已补充完整。

## 文档分类与当前性

| 类别 | 文档 | 使用方式 |
|---|---|---|
| 入口与索引 | `README.md`、`docs/README.md` | 从这里进入文档；启动模式以 `run_echo_drone.sh help` 为准 |
| 当前系统说明 | `project_overview_zh.md`、`system_contract.md`、`data_flow_audit.md` | 说明当前包、接口、TF 和数据流；本次已按源码核对 |
| 当前操作指南 | `operator_manual_zh.md`、`field_hardware_flow.md`、`serial_hardware_management.md`、`mission_debugging.md` | 按其中步骤操作；硬件结论必须在目标机复核 |
| 专题指南 | `mavros_runtime_guide.md`、`sls_real_startup.md`、`sls_circle_controller.md`、`simulation_code_guide.md`、`px4_sitl_gazebo.md` | 保留历史实测/环境记录；当前端点已尽量按源码标注 |
| 环境专用记录 | `vmware_livox_integration.md` | 只适用于对应 VMware/Livox 接入环境 |
| 设计/计划/交接 | `ARCHITECTURE.md`（后半部分）、`refactor_execution_checklist.md`、`DEBUG_PLAN.md`、`refactor_handoff_2026-08-03.md`、根目录 `REFACTORING_PLAN.md`、`REASONIX.md` | 只作为目标、背景或历史快照，不代表当前实现 |

## 本次已修正的文档问题

| 文档 | 原问题 | 修正 |
|---|---|---|
| `system_contract.md` | 环境写成 WSL2/amd64；核心链路仍写 `mavlink_control`；包数和接口过时 | 改为当前 aarch64/Foxy 快照、19 包和 MAVROS adapter 主线；补充完整接口、TF、launch 和已知缺口 |
| `mavros_runtime_guide.md` | 将旧通用 MAVROS 端点写成当前 adapter 输出 | 增加历史实测说明，并按 `mavros_adapter.yaml` 修正端点表 |
| `field_hardware_flow.md` | 要求检查不存在的 `livox_raw -> livox`；配置文件名写错；路径写死 | 改为当前 TF、`MID360s_config.json` 和可移植路径 |
| `sls_circle_controller.md`、`sls_real_startup.md` | 将 `real-*` 入口写成发布 `/mavros/setpoint_position/local` | 区分 launch 默认端点和 `run_echo_drone.sh` 的实机覆盖端点 |
| `px4_sitl_gazebo.md` | `px4_sim_model` 默认值和 MAVROS 插件配置方式写错 | 按 `px4_sitl_sls.launch.py` 与 `mavros_px4_sitl.yaml` 修正 |
| `data_flow_audit.md` | 把 `/cloud_obstacle` 说成所有 Nav2 costmap 的固定输入；夸大 `/robot/clear_state` 的作用 | 按 `sensing-costmap` 与 `sensing-nav` 两条启动路径分开描述，并标注未使用参数 |
| `ARCHITECTURE.md` | “当前实装架构”中的 MAVROS 端点和决策层状态描述过时 | 更新当前实装说明，保留后续 HSM/服务层设计稿定位 |
| `refactor_execution_checklist.md`、`repository_function_and_refactor_report.md`、`DEBUG_PLAN.md` | 日期较早，容易被误读为当前状态 | 增加历史状态说明，指向本审查文档和当前源码 |

## 仍需代码侧处理的缺口

以下是“文档已如实标注”但仍需要代码修改或现场验证的项目，不是本次审查通过修改文档就能解决的：

1. `sensing-costmap` 启动了 `/cloud_obstacle` 发布者，但 `sensing_costmap.yaml` 的 costmap 直接消费 `/cloud_registered_body`；若目标是把分割结果送入 costmap，需要统一 observation source。
2. `obstacle_segmentation` 的 `/robot/clear_state`、`base_frame`、`use_downsample` 当前没有实际作用；要么补实现，要么从文档和接口合同中移除“有效”表述。
3. `sls-real-check` 需要同时采样 `/mavros/mavros/pose`、`/mavros/mavros/local` 与旧端点，才能覆盖不同启动路径。
4. `run_echo_drone.sh` 尊重外部 `ECHO_DRONE_WS`。如果 shell 中残留其他项目的工作区路径，`check` 等命令会指向错误目录；操作前可用 `ECHO_DRONE_WS=$PWD ./run_echo_drone.sh check`，或先确认 `echo "$ECHO_DRONE_WS"`。
5. `pcd2pgm` 仍存在历史报告提到的参数名/读参变量问题：YAML 使用 `thre_point_count`，代码声明读取 `thres_point_count`；`point_num_for_statistical` 和 `standard_deviation_multiplier` 被读入错误变量。
6. MAVROS adapter 的位置 setpoint 路径只按目标年龄门控，并不依据 `/robot/current_pose` 新鲜度；速度路径才检查位姿新鲜度。实机安全验证前不要把它当成完整的失联保护。
7. 所有硬件、MAVLink、PX4、Livox、串口和飞行结论仍需在目标机上重新验证；本次只验证了静态接口和部分软件测试。

## 本次软件检查结果

已执行：

```bash
ECHO_DRONE_WS=$PWD ./run_echo_drone.sh help
ECHO_DRONE_WS=$PWD ./run_echo_drone.sh check
ROS_LOG_DIR=/tmp/ros_logs ros2 launch robot_bring_up sensing.launch.py --show-args
ROS_LOG_DIR=/tmp/ros_logs ros2 launch robot_bring_up hardware.launch.py --show-args
ROS_LOG_DIR=/tmp/ros_logs ros2 launch sls_circle_controller sls_circle.launch.py --show-args
ROS_LOG_DIR=/tmp/ros_logs colcon test --packages-select behavior_control flight_control
```

结果：

- `help` 和 `check`：成功；`check` 列出 19 个包，当前无匹配串口设备。
- 三个 launch 的 `--show-args`：均成功解析。
- `behavior_control`：`test_mission_runtime`、`test_mission_graph` 等功能测试通过；`uncrustify` 代码风格检查失败。
- `flight_control`：本次目标测试全部通过。
- 未执行 `colcon build`、完整启动、硬件测试或飞行测试。

## 文档维护建议

- 当前状态只写入本审查文档、`system_contract.md` 和实际专题操作文档；设计稿、计划和交接记录保留日期标记。
- 修改 launch/参数时，同时检查 `run_echo_drone.sh help`、对应 YAML 和专题文档中的端点名。
- 修改 `drone.yaml` 的 `map_frame`/`odom_frame` 时，同步检查 `sensing.launch.py` 的静态 TF、`behavior_control` 的 `+0.39`、`obstacle_segmentation` 的 `+0.27` 和 adapter 的高度参数。
- 所有“已验证”表述都应附日期、机器、启动模式和验证范围；没有现场记录的结论继续标为“待验证”。
