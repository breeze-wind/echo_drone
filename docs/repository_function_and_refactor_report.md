# echo_drone 仓库功能与重构计划分析报告

## 1. 结论摘要

`echo_drone` 是一个面向 2025 国机赛无人机任务的 ROS2 工作空间。主线代码以实机任务可跑为目标，核心链路是：

Livox MID-360 / IMU → Point-LIO 定位建图 → 点云障碍物分割 → Nav2 + TEB 导航 → `behavior_control` 任务状态机 → `mavlink_control` 控制 PX4 → `servo_node` 控制投放机构。

仓库当前最大问题不是“功能缺失”，而是“赛前集成型代码”：任务逻辑高度集中、接口靠话题和硬编码约定维持、参数都堆在 `drone.yaml`、测试覆盖薄弱。上个 AI 的重构方向总体正确，尤其是拆状态机、拆配置、隔离第三方代码、增加测试。但它的实施计划过于激进，直接另起 `echo_drone_v2/` 会破坏现有接口，并且已经写出的 V2 代码还只是原型，不能替代现有系统。

建议后续采用“保留主线可运行系统 + 分模块渐进替换”的路线：先冻结现有接口和启动流程，再从 `mavlink_control`、`pcd2pgm`、`behavior_control` 的纯逻辑拆分开始，每一步都用兼容话题和回归测试保护。

### 2026-07-20 执行更新

本轮已经从分析进入第一阶段执行：

- `echo_drone_v2/` 已按用户确认删除，主工作区不再有重复包名。
- ROS2 Foxy、MAVROS、Nav2、PCL、g2o 已安装并用于构建验证。
- 官方 `livox_ros_driver2` 已放入 `third_party/`，Livox-SDK2 已安装到 `/usr/local`，Point-LIO 已能依赖官方 Livox `CustomMsg` 构建。
- 旧 `robot_bt_navigator`、`robot_behaviors`、`robot_msgs` 已通过 `COLCON_IGNORE` 排除；主 launch 改回 Foxy 支持的官方 Nav2 BT navigator 和 recoveries。
- `teb_local_planner` 已做 Foxy API 兼容补丁，保留为当前必须的仓库 fork。
- 干净删除 `build/install/log` 后执行 `colcon build`，14 个包全部通过，耗时约 7min 11s。

这说明当前仓库已经有了一个可重复的 Foxy 构建基线。下一步应在这个基线上继续 MAVROS adapter、TF 独立管理、决策层可调试重构，而不是继续扩展旧 `pymavlink` 或旧 BT fork。

## 2. 仓库现状

- Git 根目录：`/home/sfx/echo_drone`
- 当前分支：`master`
- 主线最近提交：`ad6626e 赛前111`
- 未提交变更：
  - `robot_bring_up/launch/drone.launch.py` 新增 `livox_raw -> livox` 静态 TF。
  - `robot_bring_up/config/drone.yaml` 将 Point-LIO `odom_frame` 从 `livox` 改为 `livox_raw`，并把 Nav2 behavior 配置切回 Foxy `recoveries_server`。
  - `REFACTORING_PLAN.md`、`REASONIX.md`、`docs/` 等为未跟踪内容，明显是上个 AI 产生或延续的重构材料。
  - `echo_drone_v2/` 已删除，不再参与工作区。

主线工作空间包含这些主要包：

| 模块 | 作用 |
|---|---|
| `Point-LIO/` | Livox LiDAR-惯性里程计，发布 `/Odometry`、`/robot/current_pose`、`/cloud_registered_body` |
| `obstacle_segmentation_tc/` | 目录名带 `_tc`，但 ROS 包名是 `obstacle_segmentation`；从点云中提取障碍物，发布 `/cloud_obstacle` |
| `teb_local_planner/` | 内嵌 TEB 和 costmap_converter 上游代码 |
| `robot_behavior_tree/` | 旧 Nav2 BT navigator、自定义 keep-away 行为和 action；当前已排除默认构建 |
| `behavior_control/` | 任务决策核心，负责目标顺序、识别、投掷、穿门、降落 |
| `mavlink_control/` | Python MAVLink 节点，连接 PX4，发送位姿估计、速度/位置 setpoint |
| `servo_node/` | Python 串口舵机节点，通过参数 `/servo/servo` 发投放指令 |
| `robot_mapping/` | PCD 合并与 PCD 转 PGM/YAML 地图工具 |
| `robot_bring_up/` | 主启动文件和全局参数 `drone.yaml` |
| `robot_interfaces/` | 自定义视觉和舵机消息 |

## 3. 系统功能分析

### 3.1 传感与定位

`robot_bring_up/launch/drone.launch.py` 启动 Livox 驱动、Point-LIO、Nav2、障碍物分割和 RViz。Point-LIO 订阅 `/livox/lidar` 与 `/livox/imu`，输出：

- `/Odometry`：Nav2 和 TEB 使用的里程计。
- `/robot/current_pose`：`behavior_control`、`mavlink_control`、`obstacle_segmentation` 共同依赖的当前位姿。
- `/cloud_registered_body`：给障碍物分割使用的机体系点云。

当前未提交改动把 Point-LIO 的 `odom_frame` 改成 `livox_raw`，再由 launch 增加 `livox_raw -> livox` 旋转 TF，这是为修正雷达原始坐标和项目约定坐标之间的差异。

### 3.2 障碍物感知与导航

`obstacle_segmentation` 订阅 `/cloud_registered_body`、`/robot/current_pose`、`/robot/clear_state`、`/robot/obstacle_height`。它把机体系点云变到 map 系，根据高度和当前机体高度筛出障碍物，再变回 livox 系发布 `/cloud_obstacle`。

Nav2 的 local/global costmap 都使用 `/cloud_obstacle` 作为 PointCloud2 障碍物输入。控制器为 TEB，配置集中在 `robot_bring_up/config/drone.yaml` 的 `controller_server.FollowPath` 下。当前 Foxy 启动路径使用官方 `nav2_bt_navigator` 和 `nav2_recoveries`，不再启动旧 `robot_bt_navigator` / `robot_behaviors`。

### 3.3 任务决策

`behavior_control/src/behavior_control.cpp` 是当前系统的任务中枢，约 1888 行。它用两个 timer 驱动：

- `step_timer_callback()`：判断当前 step 是否达成，并切换下一个 step。
- `mission_timer_callback()`：根据当前 step 发布目标位置、发送 Nav2 action、设置舵机参数、发布模式标志。

主要 step 流程：

1. 起飞到巡航高度。
2. 在起飞点两侧搜索两个随机靶。
3. 依次处理 4 个静态靶：导航到目标点、拉高识别、下降投掷。
4. 搜索和投放随机 tank / random target。
5. 根据配置选择穿门或直接返航。
6. 穿门后或起点处降落。

任务控制同时发布：

- `/robot/target_pose`：非 Nav2 模式下给 MAVLink 的位置 setpoint。
- `/goal_pose`：调试/可视化导航目标。
- `/robot/nav_state`：选择 MAVLink 用 Nav2 速度还是直接位置控制。
- `/robot/passing_door_state`、`/robot/turning_state`：控制穿门和转向姿态。
- `/robot/obstacle_height`、`/robot/clear_state`：影响障碍物分割和代价地图。
- `/camera/choose`：切换相机。

它也订阅：

- `/robot/arm_state`
- `/robot/current_pose`
- `/robot/image_location`
- `/robot/usb_camera`

### 3.4 飞控与执行机构

`mavlink_control` 连接 manufacturer 为 `CUAV` 的串口，读取遥控通道触发解锁，发布 `/robot/arm_state`。它把 `/robot/current_pose` 转为 MAVLink `vision_position_estimate` 发给 PX4，这是定位闭环的关键路径。

控制输出有两种模式：

- `if_nav == true`：把 `/cmd_vel` 转换成 MAVLink local NED 速度。
- `if_nav == false`：把 `/robot/target_pose` 转换成 MAVLink local NED 位置 setpoint。

`servo_node` 连接 manufacturer 为 `STMicroelectronics` 的串口，每 0.1 秒读取参数 `/servo/servo`，发送单字节舵机命令。`behavior_control` 通过 `/servo_node/set_parameters` 改这个参数完成投放。

### 3.5 地图工具

`pcd2pgm` 将 PCD 地图转为 PGM/YAML 栅格地图，支持 point / normal / height 三种转换方法。这里有明确现存问题：

- YAML 中参数是 `thre_point_count`，代码声明和读取的是 `thres_point_count`，导致 YAML 值不会生效。
- `point_num_for_statistical` 被读进了 `point_num_for_normal`。
- `standard_deviation_multiplier` 被读进了 `angle_threshold`。

这说明配置项缺少自动测试，重构计划中“拆 IO、参数、转换逻辑并加测试”的方向是有必要的。

## 4. 上个 AI 重构计划评价

### 4.1 方向正确的部分

`REFACTORING_PLAN.md` 对现状的判断基本准确：

- 第三方包、自研包、比赛代码混在同一层，构建边界不清。
- `behavior_control.cpp` 是最大维护风险。
- `drone.yaml` 太大，且跨多个节点共享，修改风险高。
- 单元测试不足，改错只能靠实机。
- 需要把 TEB、costmap_converter、Point-LIO 这类外部/大模块隔离。

计划中的 HSM 状态机、`GoalManager`、`TfManager`、`ParamController`、配置分区、CI/Docker 和测试策略，都符合这个仓库的真实痛点。

### 4.2 主要问题

计划最大的问题是过于“重开新仓库”。无人机系统是强硬件耦合系统，直接从零复制代码会同时改变目录、话题、消息、参数、启动方式和飞控接口，很难判断问题来自重构还是硬件环境。

更具体的问题：

- ROS 版本目标不一致：`REASONIX.md` 和 Docker/CI 写 Humble，`docs/DEBUG_PLAN.md` 写 Foxy + Jetson Nano NX。
- 时间估算偏乐观：R3LIVE ROS1→ROS2 移植、Point-LIO 适配、实机等价验证不可能按几十分钟量级稳定完成。
- 接口迁移没有兼容层：原系统使用 `/robot/*` 话题和 `id/image_x/image_y` 消息，V2 改为 `/mavlink/*`、`/target_search/*`、`target_type/u/v`。
- 安全关键行为没有完整保留：遥控器解锁触发、PX4 vision pose estimate、穿门 yaw 控制、TEB 动态参数切换、代价地图清除、相机切换、舵机参数服务等都需要一一映射。
- 测试计划写得像目标状态，但当前 V2 代码的测试没有真正接入 `colcon test`。

## 5. V2 已落地代码审查（删除前历史记录）

`echo_drone_v2/` 已经创建了 `behavior_control`、`target_search`、`mavlink_control`、`robot_interfaces` 和 Docker/CI 原型。它有一些可复用的好点：

- `TargetDatabase` 是纯数据类，职责清楚。
- `MavlinkInterface` 把 MAVLink 协议和 ROS 节点分离，利于 mock 测试。
- `CoordinateConverter`、`VehicleStateTracker` 这类小类适合保留。
- 纯 C++ 的 `MissionStateMachine` 和 `TargetDatabase` 在手工编译为 C++17 后测试通过。

但当前 V2 不能作为替换版本：

- `MissionStateMachine` 没有从 `IDLE` 到 `ARMING` 的有效状态转移；`on_start()` 只调用 arm/offboard 服务，没有改变状态机状态。
- `MissionStateMachine::is_at_target()` 永远返回 false，导航到达判断没有实现。
- `BehaviorControlNode` 在初始化列表里先用默认 `params_` 构造 `sm_`，随后才 `load_params()`，所以 YAML 参数不会进入状态机。
- V2 使用 `std::clamp`，但 CMake 没有设置 C++17；本机默认 g++ 是 C++14，默认标准下编译失败。
- `target_search` 改了 `ImageLocation` 字段和话题，和原系统视觉节点不兼容。
- `TargetSearchNode::publish_status()` 没被调用，`behavior_control` 无法知道所有目标是否命中。
- `mavlink_control` V2 没有订阅 LIO 位姿并调用 `send_vision_pose_estimate()`，会丢掉原系统给 PX4 的视觉定位输入。
- `.github/workflows/ci.yml` 假设包在 `src/` 下，但 V2 当前包直接放在仓库根目录，CI 路径不匹配。

本次轻量验证结果：

- `g++ -std=c++17` 手工编译并运行 `test_mission_state_machine`：5 passed。
- `g++ -std=c++17` 手工编译并运行 `test_target_database`：9 passed。
- 默认 g++ C++14 编译 V2 `mission_state_machine`：失败，原因是 `std::clamp`。
- Python 测试未运行：本机缺少 `pytest`。
- 未运行 `colcon build`：当前环境没有 `colcon` 和 `ros2` 命令。

## 6. 推荐重构路线

### Phase 0：冻结现状接口

先写一份“现有接口清单”，包括话题、消息字段、参数名、launch 启动顺序、TF 树、飞控/舵机串口筛选规则。后续所有重构都必须先兼容这份清单。

### Phase 1：修小而确定的问题

- 修 `pcd2pgm` 的参数名和读参变量错误。
- 给 `mavlink_control` 和 `servo_node` 增加无硬件 fallback，避免找不到串口时直接异常。
- 把 `behavior_control` 中雷达/相机/机体高度偏置集中成参数，先不改状态机结构。

### Phase 2：抽纯逻辑，不改外部接口

从原 `behavior_control.cpp` 中先抽出：

- 静态靶配置与顺序管理。
- step 编号到语义状态的映射。
- 投掷偏置和舵机位选择。
- TEB 参数 profile。

外部话题仍保持 `/robot/*` 不变，这样可以逐步替换内部实现。

### Phase 3：HSM 替代 step 逻辑

在原包内实现 HSM，而不是马上切到 V2 新话题体系。每迁移一组 step，就保留同样的发布/订阅行为，并用测试覆盖：

- 起飞/巡航高度。
- 4 个静态靶通用流程。
- 随机靶搜索和 fallback。
- 穿门参数切换、转向、清图。
- 降落。

### Phase 4：再考虑目录清理和第三方隔离

等任务逻辑有测试保护后，再移动第三方目录到 `third_party/`，拆配置文件和集中 launch。这个阶段只做路径和构建层面的整理，不再同时改飞控/任务逻辑。

### Phase 5：V2 代码择优回收

不要直接切换到 `echo_drone_v2/`。可回收的部分是：

- `mavlink_control/coordinate_converter.py`
- `mavlink_control/vehicle_state.py`
- `mavlink_control/serial_discovery.py`
- `target_search/target_database.*`

需要重写或大改的部分是：

- V2 `behavior_control` 状态机启动和参数加载。
- V2 `target_search` 与原视觉消息的兼容。
- V2 `mavlink_control` 对 `/robot/current_pose` → `vision_position_estimate` 的支持。
- CI 的 workspace 布局。

## 7. 优先级建议

高优先级：

1. 修复 `pcd2pgm` 参数读取错误。
2. 写出现有话题/参数/TF 清单。
3. 给 `mavlink_control` 增加串口未找到时的明确报错和 mock 模式。
4. 给 `behavior_control` 做 step 流程表和最小纯逻辑测试。

中优先级：

1. 抽 `GoalManager`、`ParamController`、`ServoController`。
2. 拆 `drone.yaml`，但先保持加载结果等价。
3. 把 `echo_drone_v2` 中可复用的小模块迁回主线。

已完成的基线项：

1. 删除 `echo_drone_v2/` 并消除重复包名。
2. 接入官方 Livox ROS2 driver 和 Livox-SDK2。
3. 清理旧 BT/behavior 包出默认构建。
4. 修复 Foxy 构建兼容问题并完成干净 `colcon build`。

低优先级：

1. R3LIVE LIO 移植。
2. 大规模目录重排。
3. Docker/CI 全量完善。

这些不是不重要，而是不应该排在飞控闭环和任务等价验证之前。

## 8. 最终判断

上个 AI 的重构计划可以作为“方向参考”，不能作为“直接执行方案”。真正可执行的路线应该是：先保护原系统接口和实机行为，再从风险最低的纯逻辑和参数一致性开始重构。`echo_drone_v2/` 当前更像概念验证仓库，价值在于提供了一些可测试小模块，不适合作为替换主线。
