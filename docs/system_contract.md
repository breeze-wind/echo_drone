# 现有系统接口清单

本文档记录当前 `echo_drone` 主线系统的可运行接口合同。它依据 2026-10-27 的源码、launch、YAML 和静态检查整理；后续重构必须先兼容这些接口，再逐步替换内部实现。硬件链路是否真正连通仍需在目标机上复核。

## 1. 当前环境与验证边界

本次审查直接确认：

| 项目 | 当前值 |
|---|---|
| 系统 | Ubuntu 20.04.6 LTS |
| 架构 | `aarch64`（Tegra 内核） |
| Python | 3.8.10 |
| ROS 2 | Foxy |
| 工作区可见 ROS 包 | 19 个 |
| 本次硬件测试 | 未执行；`check` 未发现匹配串口 |

目标仍以 ROS 2 Foxy 为主。旧文档中的 WSL2/amd64、14 个包等结论属于历史环境快照，不能直接沿用。

当前 `colcon list --names-only`：

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

## 2. 当前核心功能链路

### 2.1 Nav2 导航链路

```text
Livox MID-360 + IMU
  -> livox_ros_driver2
  -> Point-LIO
  -> /Odometry, /robot/current_pose, /cloud_registered_body, TF odom->livox
  -> obstacle_segmentation
  -> /cloud_obstacle
  -> Nav2 + TEB
  -> /cmd_vel
  -> flight_control/mavros_adapter
  -> MAVROS -> PX4
```

注意：只有 `sensing-nav`/`lidar_nav_stack` 路径的 costmap 当前订阅 `/cloud_obstacle`。`sensing-costmap` 路径使用的 `sensing_costmap.yaml` 直接订阅 `/cloud_registered_body`。

### 2.2 直接位置目标链路

```text
behavior_control
  -> /robot/target_pose
  -> flight_control/mavros_adapter
  -> /mavros/mavros/local
  -> MAVROS -> PX4
```

`/robot/nav_state=false` 时 adapter 选择位置目标；目标超时后停止重发陈旧位置目标。位置路径当前不检查 `/robot/current_pose` 新鲜度，实机前需要补安全验证。

### 2.3 SLS/QSF 控制链路

```text
MAVROS state/pose
  -> sls_circle_controller_cpp
  -> /mavros/setpoint_position/local 或 /mavros/setpoint_raw/attitude
  -> MAVROS -> PX4
```

SLS 的 `nav_goal_velocity` 模式则：

```text
behavior_control -> /sls_circle/nav_goal
  -> sls_circle_controller_cpp -> /sls_circle/nav_velocity_setpoint
  -> flight_control/mavros_adapter -> /mavros/mavros/cmd_vel -> PX4
```

SLS 对真实 MAVROS 输出有 `dry_run=false`、`enable_real_setpoint=true`、状态、模式和解锁门控。`run_echo_drone.sh` 的 `real-*` 入口会覆盖部分 MAVROS topic 名称，详见 SLS 专题文档。

## 3. 当前包职责

| 包/目录 | ROS 包名 | 当前职责 | 边界 |
|---|---|---|---|
| `Point-LIO/` | `point_lio` | Livox LiDAR/IMU 里程计，发布位姿、里程计、点云和 TF | `map_frame=odom`、`odom_frame=livox` 由 YAML 配置 |
| `obstacle_segmentation_tc/` | `obstacle_segmentation` | 机体系点云滤波、高度筛选，发布 `/cloud_obstacle` | `clear_state`、`base_frame`、`use_downsample` 尚未实际生效 |
| `teb_local_planner/` | `teb_local_planner`、`costmap_converter*`、`teb_msgs` | Foxy 兼容 TEB 规划插件和依赖 | 源码 fork 仍在工作区构建 |
| `behavior_control/` | `behavior_control` | 任务步骤、目标/模式发布、任务调试图和服务 | 正常转移仍由旧数字 step 分支执行 |
| `flight_control/` | `flight_control` | 旧 `/robot/*` 接口到 MAVROS 的适配层 | 当前默认飞控主线；不打开串口 |
| `mavlink_control/` | `mavlink_control` | `pymavlink` 直连 PX4 | 旧 fallback，默认不启动 |
| `servo_node/` | `servo_node` | STM32 串口舵机、dry-run、状态和投放服务 | 单字节命令；旧参数接口兼容 |
| `robot_serial_manager/` | `robot_serial_manager` | 枚举串口并发布状态，不打开业务串口 | 只做发现和报告 |
| `robot_bring_up/` | `robot_bring_up` | launch、全局和硬件 YAML 编排 | 不承担核心运行时算法 |
| `robot_interfaces/` | `robot_interfaces` | 自定义消息和服务 | 当前已有 4 个 msg、2 个 srv |
| `robot_mapping/` | `merge_pcd`、`pcd2pgm` | 离线 PCD 合并与栅格地图转换 | 当前 `pcd2pgm` 仍有历史参数读参 bug |
| `sls_circle_controller_cpp/` | `sls_circle_controller_cpp` | 当前主 SLS/QSF C++ 控制节点 | 实机脚本覆盖端点 |
| `sls_qsf_core/` | `sls_qsf_core` | QSF 生成算法的 C ABI 和共享库 | 非独立 ROS 节点 |
| `sls_circle_controller/` | `sls_circle_controller` | SLS launch、参数、Python 仿真/桥接资源和 SITL 资源 | Python 控制器不是当前实机主路径 |
| `third_party/livox_ros_driver2/` | `livox_ros_driver2` | Livox ROS 2 驱动 | 依赖 Livox-SDK2 |
| `robot_behavior_tree/` | `robot_behaviors`、`robot_bt_navigator`、`robot_msgs` | 历史自定义行为树 | 三个子包由 `COLCON_IGNORE` 排除 |
| `detect/` | 无 ROS 包 | YOLO/相机实验脚本 | 不发布 `ImageLocation`，不在主控制链 |

## 4. 当前话题、服务和 Action

### 4.1 定位与点云

| 接口 | 类型 | 发布者 | 订阅者/用途 |
|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Livox driver | Point-LIO |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Livox driver | Point-LIO |
| `/Odometry` | `nav_msgs/msg/Odometry` | Point-LIO | Nav2/TEB、诊断和记录 |
| `/robot/current_pose` | `geometry_msgs/msg/TransformStamped` | Point-LIO | behavior_control、obstacle_segmentation、flight_control |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Point-LIO | RViz/调试 |
| `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | Point-LIO | obstacle_segmentation；`sensing-costmap` 的 costmap |
| `/cloud_obstacle_new` | `sensor_msgs/msg/PointCloud2` | Point-LIO | 当前主链路未使用 |
| `/Laser_map` | `sensor_msgs/msg/PointCloud2` | Point-LIO | RViz/调试 |
| `/path` | `nav_msgs/msg/Path` | Point-LIO | RViz/调试 |
| `planner_normal` | `visualization_msgs/msg/Marker` | Point-LIO | 调试 |

### 4.2 障碍物和 Nav2

| 接口 | 类型 | 发布者 | 订阅者/用途 |
|---|---|---|---|
| `/cloud_obstacle` | `sensor_msgs/msg/PointCloud2` | obstacle_segmentation | `sensing-nav` 路径的 local/global costmap |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 controller/TEB | flight_control adapter |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | behavior_control / RViz | 调试或 SLS 目标输入 |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | behavior_control 作为 client | Nav2 BT Navigator 作为 server |
| `/robot/obstacle_height` | `std_msgs/msg/Float64` | behavior_control | obstacle_segmentation 高度上限 |
| `/robot/clear_state` | `std_msgs/msg/Bool` | behavior_control | obstacle_segmentation 当前只缓存，未参与清图 |
| `/controller_server/set_parameters` | `rcl_interfaces/srv/SetParameters` | behavior_control 调用 | 穿门时调整 TEB 参数 |
| `/local_costmap/local_costmap/set_parameters` | `rcl_interfaces/srv/SetParameters` | behavior_control 调用 | 穿门时调整局部 costmap 参数 |

### 4.3 任务决策和飞控

| 接口 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/robot/arm_state` | `std_msgs/msg/Bool` | flight_control -> behavior_control | 飞控解锁状态 |
| `/robot/target_pose` | `geometry_msgs/msg/TransformStamped` | behavior_control -> flight_control | 非 Nav2 模式位置目标 |
| `/robot/nav_state` | `std_msgs/msg/Bool` | behavior_control -> flight_control | `true` 优先 `/cmd_vel`；`false` 使用位置目标 |
| `/robot/passing_door_state` | `std_msgs/msg/Bool` | behavior_control -> flight_control | 穿门上下文 |
| `/robot/turning_state` | `std_msgs/msg/Bool` | behavior_control -> flight_control | 转向上下文 |
| `/robot/sls_goal_control_active` | `std_msgs/msg/Bool` | behavior_control 或 SLS -> adapter | SLS 速度输出租约 |
| `/flight_control/status` | `std_msgs/msg/String` JSON | flight_control -> 调试 | 连接、模式、解锁、输入年龄和输出端点 |
| `/servo/command` | `std_msgs/msg/Int32` | 调试/任务 -> servo_node | 舵机命令，范围由 YAML 限制 |
| `/servo/drop` | `std_srvs/srv/Trigger` | 调试/任务 -> servo_node | 发送 `drop_command` |
| `/servo/status` | `std_msgs/msg/String` JSON | servo_node -> 调试 | 串口、dry-run、最近命令和错误 |
| `/servo_node/set_parameters` | `rcl_interfaces/srv/SetParameters` | behavior_control -> servo_node | 旧 `/servo/servo` 参数兼容桥 |
| `/hardware/serial_status` | `std_msgs/msg/String` JSON | robot_serial_manager -> 调试 | 串口枚举和 owner 提示 |
| `/hardware/rescan_serials` | `std_srvs/srv/Trigger` | 调试 -> robot_serial_manager | 手动重扫串口 |

### 4.4 任务调试服务

| 服务 | 类型 | 说明 |
|---|---|---|
| `/mission/start` | `std_srvs/srv/Trigger` | 启动任务 |
| `/mission/pause` | `std_srvs/srv/Trigger` | 暂停 |
| `/mission/resume` | `std_srvs/srv/Trigger` | 继续 |
| `/mission/step_once` | `std_srvs/srv/Trigger` | 单步 |
| `/mission/abort` | `std_srvs/srv/Trigger` | 中止 |
| `/mission/dump_context` | `std_srvs/srv/Trigger` | 输出上下文 JSON 日志 |
| `/mission/jump_to_state` | `robot_interfaces/srv/JumpMissionState` | dry-run 且 idle/paused 时按 `state_id` 硬跳 |
| `/mission/shift_state` | `robot_interfaces/srv/ShiftMissionState` | 按调试顺序做 `delta` 跳转 |

行为节点还发布：

- `/mission/status`：`robot_interfaces/msg/MissionStatus`
- `/mission/event`：`robot_interfaces/msg/MissionEvent`
- `/mission/graph_dot`：`std_msgs/msg/String`，1 Hz 且 transient-local
- `/robot/mission_status`：兼容旧接口的 `MissionStatus`

### 4.5 SLS/QSF 接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | SLS launch 默认位姿输入 |
| `/mavros/local_position/velocity_local` | `geometry_msgs/msg/TwistStamped` | SLS launch 默认速度输入 |
| `/mavros/state` | `mavros_msgs/msg/State` | 飞控状态输入 |
| `/sls_circle/load_pose` | `geometry_msgs/msg/PoseStamped` | 可选负载位姿 |
| `/sls_circle/nav_goal` | `geometry_msgs/msg/PoseStamped` | SLS 最终目标输入 |
| `/sls_circle/nav_velocity_setpoint` | `geometry_msgs/msg/TwistStamped` | SLS 速度输出，由 adapter 转发 |
| `/sls_circle/reference_pose` | `geometry_msgs/msg/PoseStamped` | 参考轨迹调试 |
| `/sls_circle/debug/attitude_target` | `mavros_msgs/msg/AttitudeTarget` | 调试姿态目标 |
| `/sls_circle/status` | `std_msgs/msg/String` JSON | 控制状态和索引 |
| `/sls_circle/wind_estimate` | `geometry_msgs/msg/Vector3Stamped` | 抗风估计，单位 N，z 固定为 0 |
| `/sls_circle/wind_actual` | `geometry_msgs/msg/Vector3Stamped` | Gazebo 仿真实际风扰 |
| `/mavros/setpoint_position/local` | `geometry_msgs/msg/PoseStamped` | 单独 launch 默认起飞位置输出 |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/msg/AttitudeTarget` | 绕圈真实姿态输出 |
| `/mavros/mavros/pose` | `geometry_msgs/msg/PoseStamped` | `real-*` 脚本默认位姿输入 |
| `/mavros/mavros/local` | `geometry_msgs/msg/PoseStamped` | `real-*` 脚本默认位置 setpoint 输出 |

## 5. 自定义消息和服务

当前 `robot_interfaces` 生成：

```text
msg/Receive/ImageLocation.msg: uint8 id, float32 image_x, float32 image_y
msg/Transmit/ServoPos.msg:     uint8 servo_pos
msg/MissionStatus.msg:         stamp, legacy_step, state_id, state, phase, target,
                               execution_state, elapsed_sec, dry_run, armed,
                               navigation_active, flags, outgoing_state_ids,
                               debug_previous_state_id, debug_next_state_id
msg/MissionEvent.msg:          stamp, sequence, from/to legacy_step, from/to state_id,
                               from/to state, reason, execution_state
srv/JumpMissionState.srv:      state_id, reset_context -> success, message,
                               resulting_state_id, resulting_state
srv/ShiftMissionState.srv:     delta, reset_context -> success, message,
                               resulting_state_id, resulting_state
```

兼容要求：不要直接修改 `ImageLocation.id/image_x/image_y` 字段，也不要随意修改 `MissionStatus`/`MissionEvent` 的现有字段顺序和类型。

## 6. TF / Frame 合同

当前实际 TF 关系：

```text
map --static z=0.39--> odom --Point-LIO dynamic--> livox
livox --static--> mavlink_body
livox --static--> camera_link
```

| Frame | 当前来源 | 说明 |
|---|---|---|
| `map` | 静态 TF/世界系 | 任务和 Nav2 目标使用 |
| `odom` | `sensing.launch.py` 的 `map -> odom` 静态 TF | 当前 Point-LIO 的父 frame |
| `livox` | Point-LIO 动态 TF 的子 frame | 项目约定雷达系，也是 Nav2 `robot_base_frame` |
| `mavlink_body` | `livox -> mavlink_body` 静态 TF | 飞控机体系 |
| `camera_link` | `livox -> camera_link` 静态 TF | D435 相机系 |

当前 `sensing.launch.py` 没有 `livox_raw -> livox`。Point-LIO YAML 当前为：

```yaml
common:
  map_frame: "odom"
  odom_frame: "livox"
```

### 6.1 当前硬编码偏置

| 位置 | 数值 | 含义/风险 |
|---|---:|---|
| `robot_bring_up/launch/sensing.launch.py` | `map -> odom z=0.39` | 雷达离地/装载偏置，未集中管理 |
| `robot_bring_up/launch/sensing.launch.py` | `livox -> mavlink_body z=-0.08`，含 180° roll + 45° yaw 修正 | 飞控机体安装外参 |
| `robot_bring_up/launch/sensing.launch.py` | `livox -> camera_link [0.065,-0.065,-0.26]` | 相机外参 |
| `behavior_control.cpp` | `current_z = z + 0.39` | 任务高度修正 |
| `obstacle_segmentation.cpp` | `current_z = z + 0.27` | 障碍物高度筛选修正 |
| `mavros_adapter.yaml` | `height_feedback_z_offset=0.39`、`fcu_reference_z_offset=0.08` | 旧飞控高度和 vision 输入换算 |

修改 frame 或偏置时必须同步检查 Point-LIO、静态 TF、behavior_control、obstacle_segmentation、adapter 和 Nav2 参数。

## 7. 当前 launch 入口

| 文件 | 作用 | 当前边界 |
|---|---|---|
| `robot_bring_up/launch/hardware.launch.py` | 串口管理、舵机、MAVROS、adapter、legacy fallback | 默认 `dry_run=true`，不启动真实 MAVROS |
| `flight_control/launch/mavros_state.launch.py` | 最小 MAVROS 节点 | 只做连接/状态检查 |
| `flight_control/launch/mavros_adapter.launch.py` | 单独启动 adapter | 默认 YAML `dry_run=true` |
| `robot_bring_up/launch/sensing.launch.py` | Livox、Point-LIO、静态 TF、可选 costmap | 默认不启动 Nav2 action 服务 |
| `robot_bring_up/launch/sensing_nav.launch.py` | 单雷达 sensing + rolling costmap + Nav2 action | 使用 `/cloud_obstacle` 的 costmap 路径 |
| `robot_bring_up/launch/lidar_nav_stack.launch.py` | costmap、planner、controller、recoveries、BT navigator | 需要已有 sensing |
| `robot_bring_up/launch/nav_services.launch.py` | 在已有 costmap 上追加 NavigateToPose 服务 | 不重启感知 |
| `robot_bring_up/launch/bringup_launch.py` | Nav2 bringup 包装 | 基础模板，实际无人机常用 `sensing-nav` |
| `robot_bring_up/launch/navigation_launch.py` | Foxy Nav2 navigation 节点集合 | 模板入口 |
| `robot_bring_up/launch/localization_launch.py` | map_server/AMCL 等模板逻辑 | 与 LIO 主链路部分冗余 |
| `robot_bring_up/launch/drone.launch.py` | 旧整机入口 | 一次性拉起多模块，不宜作为首选 |
| `behavior_control/launch/behavior_control.launch.py` | 任务决策节点 | `dry_run` 默认 false，`mission-dry` 会覆盖为 true |
| `obstacle_segmentation_tc/launch/obstacle_segmentation.launch.py` | 障碍物分割 | 默认加载 `drone.yaml` |
| `Point-LIO/launch/pointlio.launch.py` | Point-LIO | 默认加载 `drone.yaml` |
| `sls_circle_controller/launch/sls_circle.launch.py` | SLS 单节点 | 默认 dry-run |
| `sls_circle_controller/launch/sls_circle_sim.launch.py` | 点质量仿真 | 无 PX4/Gazebo |
| `sls_circle_controller/launch/sls_circle_gazebo.launch.py` | Gazebo 力输入烟测 | 非 PX4 动力学 |
| `sls_circle_controller/launch/px4_sitl_sls.launch.py` | PX4 SITL 闭环 | 依赖本机 PX4/Gazebo Classic |

启动模式的权威列表仍是：

```bash
./run_echo_drone.sh help
```

## 8. 当前配置入口

| 配置 | 当前内容 |
|---|---|
| `robot_bring_up/config/drone.yaml` | behavior_control、mavlink_control、obstacle_segmentation、Point-LIO、Nav2/TEB、costmap、map_server 等 |
| `robot_bring_up/config/sensing_costmap.yaml` | `sensing-costmap` 的 rolling costmap；当前直接订阅 `/cloud_registered_body` |
| `robot_bring_up/config/hardware/ports.yaml` | 逻辑串口设备和 owner |
| `robot_bring_up/config/hardware/mavros.yaml` | 硬件层 MAVROS 旧参数位（实际 launch 参数也由命令行传入） |
| `robot_bring_up/config/hardware/servo.yaml` | 舵机串口、范围、dry-run 和状态周期 |
| `robot_bring_up/config/hardware/openmv.yaml` | OpenMV 配置占位，驱动未实现 |
| `flight_control/config/mavros_adapter.yaml` | adapter 端点、坐标模式、安全门和 SLS 租约 |
| `flight_control/config/mavros_vision_px4.yaml` | MAVROS PX4 vision/setpoint 插件和 frame 配置 |
| `sls_circle_controller/config/sls_circle.yaml` | SLS/QSF、轻量仿真和 Gazebo 桥接参数 |
| `sls_circle_controller/config/mavros_px4_sitl.yaml` | PX4 SITL 的 MAVROS 插件 allowlist |

## 9. 必须保留的外部行为

- Point-LIO 继续输出 `/Odometry`、`/robot/current_pose` 和 `/cloud_registered_body`，并保持 `odom -> livox` 动态 TF。
- `sensing.launch.py` 继续发布 `map -> odom`、`livox -> mavlink_body`、`livox -> camera_link`。
- Nav2/TEB 继续消费 `/Odometry`、TF 和障碍物输入；使用 `/cloud_obstacle` 的路径必须明确区分于 `sensing-costmap` 的 `/cloud_registered_body`。
- 决策层仍能通过 `navigate_to_pose` action 发送目标，并能切换 `/robot/nav_state`。
- flight_control 继续提供 `/robot/arm_state`、`/robot/target_pose`、`/cmd_vel` 兼容入口和 `/flight_control/status`。
- 舵机继续支持旧 `/servo/servo` 参数接口、`/servo/command`、`/servo/drop` 和 `/servo/status`。
- `ImageLocation.id/image_x/image_y` 字段先保持兼容。
- 任务调试接口 `/mission/status`、`/mission/event`、`/mission/graph_dot` 和硬跳服务保持不变。
- SLS 的真实输出必须同时满足 `dry_run=false`、`enable_real_setpoint=true` 和飞行阶段/状态门控。

## 10. 待确认和已知缺口

| 项 | 当前状态 |
|---|---|
| `sensing-costmap` 的 costmap 输入 | 与 `/cloud_obstacle` 不一致，需决定修改 YAML 还是文档 |
| `obstacle_segmentation` 的 `clear_state`/`base_frame`/`use_downsample` | 参数和回调存在，但未参与处理逻辑 |
| `sls-real-check` | 仍采样旧通用 MAVROS 端点，不能完整覆盖 `real-*` 路径 |
| 外部视觉输入 | 仓库内未发现发布 `/robot/image_location`、`/robot/usb_camera` 的 ROS 节点 |
| MAVROS 实机连接 | 仍需在目标机上验证 topic/service 名称、vision pose、setpoint 和 RC 接管 |
| PX4/ARM64 | 仍需验证 MAVROS、Livox-SDK2、Point-LIO、Nav2、TEB 和 Gazebo 依赖 |
| TF/高度偏置 | 0.39/0.27/0.08 仍需现场实测确认 |
| 安全 | 无桨 OFFBOARD、失联保护、RC 接管和急停流程仍需独立验证 |
| `pcd2pgm` | 仍有参数名和读参变量 bug |
| 测试 | 功能单测通过一部分，`behavior_control` 的 `uncrustify` 当前失败 |

## 11. 源码索引

- Point-LIO 接口与 TF：`Point-LIO/src/laserMapping.cpp`、`Point-LIO/src/parameters.cpp`
- 感知 launch/TF：`robot_bring_up/launch/sensing.launch.py`
- 导航组合：`robot_bring_up/launch/sensing_nav.launch.py`、`lidar_nav_stack.launch.py`
- 障碍物处理：`obstacle_segmentation_tc/src/obstacle_segmentation.cpp`
- 任务决策：`behavior_control/src/behavior_control.cpp`、`behavior_control/src/mission_graph.cpp`
- MAVROS 适配：`flight_control/flight_control/mavros_adapter_node.py`
- MAVROS 配置：`flight_control/config/mavros_adapter.yaml`、`mavros_vision_px4.yaml`
- SLS 控制：`sls_circle_controller_cpp/src/sls_circle_controller_node.cpp`
- 硬件层：`robot_bring_up/launch/hardware.launch.py`
- 串口/舵机：`robot_serial_manager/robot_serial_manager/robot_serial_manager_node.py`、`servo_node/servo_node/servo_node.py`
