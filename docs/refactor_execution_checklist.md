# 全面重构执行清单

本清单按用户确认的硬约束组织：MAVROS 包装、决策层可调试、清理 BT/遗留版本、TF 单独管理、launch/config 齐全、面向 Foxy + ARM64 + 试验机验证。

## 当前执行状态（2026-07-20）

- [x] 已删除 `echo_drone_v2/`，主工作区不再有重复包名。
- [x] 已安装并验证 ROS2 Foxy、MAVROS、Nav2、PCL、g2o。
- [x] 已接入官方 `third_party/livox_ros_driver2`，并安装 Livox-SDK2 到 `/usr/local`。
- [x] 已给 `third_party/Livox-SDK2` 添加 `COLCON_IGNORE`，SDK2 不再作为 ROS 包构建。
- [x] 已给旧 `robot_bt_navigator`、`robot_behaviors`、`robot_msgs` 添加 `COLCON_IGNORE`。
- [x] 已将主 Nav2 launch 调整为 Foxy 支持的 `controller_server`、`planner_server`、`recoveries_server`、`bt_navigator`、`waypoint_follower`。
- [x] 已修复 TEB、Point-LIO、PCL/TF include、`merge_pcd` C++17 等 Foxy 构建问题。
- [x] 干净删除 `build/install/log` 后执行 `colcon build`：14 packages finished，耗时约 7min 11s。
- [x] `source install/setup.bash && ros2 pkg prefix point_lio` 成功。
- [x] `source install/setup.bash && ros2 pkg prefix livox_ros_driver2` 成功。
- [x] `source install/setup.bash && ros2 pkg prefix robot_bt_navigator` 返回 `Package not found`，旧 BT navigator 已从主索引移除。
- [x] `ros2 launch robot_bring_up drone.launch.py --show-args` 成功解析。
- [ ] 尚未在当前阶段迁移 MAVLink 到 MAVROS adapter；`mavlink_control` 仍是旧 fallback。
- [ ] 尚未拆出 `robot_tf_manager`；static TF 仍在 `drone.launch.py`。
- [ ] 尚未重构 `behavior_control` 状态机；本轮只完成构建基线和遗留包清理。

## 0. 总体边界

- [x] 不在当前 WSL2/amd64/Foxy 开发环境上假设试验机行为。
- [x] 不直接采用 `echo_drone_v2/` 替换主线。
- [x] 不再新增 `pymavlink` 直连飞控逻辑。
- [ ] 不先改 `robot_interfaces/msg/Receive/ImageLocation.msg` 字段。
- [ ] 不同时修改飞控、决策、TF、Nav2 参数和目录结构。
- [ ] 每一阶段必须保留可回退入口。

## 1. Phase A：现状冻结

目标：把当前能跑系统的接口固定下来，后续以此判断重构是否破坏功能。

- [x] 记录当前开发环境：WSL2 / Ubuntu 20.04 / amd64 / ROS2 Foxy / MAVROS / Nav2。
- [x] 记录当前包职责。
- [x] 记录当前关键 topic/service/action。
- [x] 记录当前 TF 和硬编码偏置。
- [x] 记录当前 launch/config 入口。
- [x] 确认 `colcon list` / `colcon graph` 可运行。
- [x] 确认 `echo_drone_v2/` 与主线存在重复包名；用户确认后已删除。
- [ ] 在试验机上补充 `ros2 topic list`、`ros2 node list`、`ros2 service list` 实测输出。
- [ ] 在试验机上补充 `tf2_tools view_frames` 或等价 TF 树截图/文本。
- [ ] 录制一段基准 rosbag：`/robot/current_pose`、`/Odometry`、`/cloud_obstacle`、`/cmd_vel`、视觉识别话题、任务状态话题。

产物：

- [x] `docs/system_contract.md`
- [ ] `docs/trial_machine_baseline.md`

## 2. Phase B：MAVROS 飞控适配

目标：用 MAVROS 替代 `mavlink_control` 的 `pymavlink` 直连，同时保持旧 `/robot/*` 接口兼容。

### 2.1 试验机依赖确认

- [ ] 确认试验机 ROS2 版本：Foxy。
- [ ] 确认试验机 CPU 架构：ARM64。
- [ ] 确认 MAVROS Foxy 可用版本和安装方式。
- [ ] 确认 `mavros_msgs` 的 topic/service 名称。
- [ ] 确认 PX4 与 MAVROS 的连接方式：串口、波特率、设备名。
- [ ] 确认 MAVROS 是否能发布/订阅 vision pose、local setpoint、state、RC。

### 2.2 新包结构

新建 `flight_control`：

```text
flight_control/
├── package.xml
├── CMakeLists.txt 或 setup.py
├── launch/flight_control.launch.py
├── config/mavros_bridge.yaml
└── src 或 flight_control/
    ├── mavros_adapter
    ├── setpoint_mux
    ├── state_monitor
    └── safety_gate
```

### 2.3 兼容接口

- [ ] `/robot/current_pose` -> MAVROS vision pose 输入。
- [ ] `/robot/target_pose` -> MAVROS position setpoint。
- [ ] `/cmd_vel` -> MAVROS velocity setpoint。
- [ ] MAVROS state -> `/robot/arm_state`。
- [ ] `/robot/nav_state` 控制 setpoint 来源。
- [ ] `/robot/passing_door_state` 保留穿门坐标/姿态逻辑。
- [ ] `/robot/turning_state` 保留转向 yaw 逻辑。
- [ ] 原 `mavlink_control` launch 改为启动 MAVROS + adapter。
- [ ] 保留旧 `mavlink_control` 为 fallback，但默认不启动。

### 2.4 安全与测试

- [ ] Adapter 提供 `dry_run` 参数，dry-run 下不调用 MAVROS armed/mode 服务。
- [ ] Adapter 发布 `/flight_control/status`。
- [ ] 丢失 `/robot/current_pose` 时停止 setpoint 或进入安全状态。
- [ ] MAVROS disconnected 时拒绝任务启动。
- [ ] 无桨测试：vision pose 输入正常。
- [ ] 无桨测试：position setpoint 正常。
- [ ] 无桨测试：velocity setpoint 正常。
- [ ] 无桨测试：RC 接管/急停策略正常。

## 3. Phase C：TF 单独管理

目标：所有静态外参、frame 语义、高度偏置统一由 `robot_tf_manager` 管理。

### 3.1 新包结构

```text
robot_tf_manager/
├── package.xml
├── CMakeLists.txt 或 setup.py
├── launch/tf.launch.py
├── config/static_transforms.yaml
└── src 或 robot_tf_manager/
    ├── static_tf_loader
    ├── pose_normalizer
    └── tf_health_checker
```

### 3.2 必须迁移的 TF

- [ ] `map -> odom`
- [ ] `livox_raw -> livox`
- [ ] `livox -> mavlink_body`
- [ ] `livox -> camera_link`

### 3.3 必须消除的业务节点硬编码

- [ ] `behavior_control.cpp` 中 `current_z = z + 0.39`。
- [ ] `mavlink_control_node.py` 中 `z + 0.39` 与 `z - 0.08`。
- [ ] `obstacle_segmentation.cpp` 中 `z + 0.27`。
- [ ] `drone.launch.py` 中 static TF 数字参数。

### 3.4 健康检查

- [ ] 启动时检查 `map/livox/camera_link/mavlink_body` 均存在。
- [ ] 启动时打印当前外参来源文件。
- [ ] 提供 `/tf_manager/status`。
- [ ] TF 缺失时 mission 不允许进入 armed/offboard。

## 4. Phase D：决策层可调试重构

目标：把 `behavior_control.cpp` 的数字 step 迁移为可观察、可暂停、可单步、可回放的任务状态机。

### 4.1 状态机基本要求

- [ ] 状态命名替代数字 step。
- [ ] 保留旧 step 到新状态的映射表，便于比对。
- [ ] 状态转移必须产生日志事件。
- [ ] `tick()` 不允许阻塞等待 service/action。
- [ ] service/action 均异步处理。
- [ ] 所有目标点、投掷偏置、舵机序号从配置读取。
- [ ] 决策层不再直接维护 TF 矩阵，改用 `robot_tf_manager` 或标准 TF 查询封装。

### 4.2 调试接口

- [ ] 发布 `/mission/status`：state、target、phase、elapsed、flags。
- [ ] 发布 `/mission/event`：from_state、to_state、reason、timestamp。
- [ ] 服务 `/mission/start`。
- [ ] 服务 `/mission/pause`。
- [ ] 服务 `/mission/resume`。
- [ ] 服务 `/mission/step_once`。
- [ ] 服务 `/mission/abort`。
- [ ] 服务 `/mission/dump_context`。
- [ ] 参数 `dry_run`：不发真实飞控/舵机，只发布调试事件。
- [ ] 参数 `start_state`：允许从指定阶段开始调试。
- [ ] 参数 `target_override`：允许只调某一个目标。

### 4.3 模块拆分

```text
mission_control/
├── mission_state_machine
├── mission_context
├── target_manager
├── static_target_task
├── random_target_task
├── door_passing_task
├── nav_goal_client
├── flight_command_client
├── servo_client
├── teb_profile_client
└── mission_debug_publisher
```

### 4.4 兼容行为

- [ ] 保留 `/robot/target_pose`。
- [ ] 保留 `/robot/nav_state`。
- [ ] 保留 `/robot/passing_door_state`。
- [ ] 保留 `/robot/turning_state`。
- [ ] 保留 `/robot/obstacle_height`。
- [ ] 保留 `/robot/clear_state`。
- [ ] 保留 `/camera/choose`。
- [ ] 保留 `/robot/image_location`。
- [ ] 保留 `/robot/usb_camera`。
- [ ] 保留 Nav2 `navigate_to_pose` action 行为。
- [ ] 保留穿门时 TEB profile 切换行为。
- [ ] 保留舵机投放时序。

## 5. Phase E：BT/遗留清理

目标：减少旧版本包、重复 BT navigator、自定义行为插件带来的构建和调试负担。

### 5.1 待审查

- [x] `robot_behavior_tree/robot_bt_navigator` 是否有当前任务必须保留的修改：当前主 launch 已切回官方 `nav2_bt_navigator`，该包已忽略。
- [x] `robot_behavior_tree/robot_behaviors/KeepAwayFromObstacles` 是否被当前主任务调用：Foxy 主 launch 改用 `nav2_recoveries`，该包已忽略。
- [x] `robot_behavior_tree/robot_msgs/KeepAwayFromObstacles.action` 是否仍被引用：仅旧 `robot_behaviors` 使用，已忽略。
- [x] `teb_local_planner/` 是否必须使用仓库 fork：Foxy apt 源无 ROS2 版 TEB 二进制包，当前 fork 仍保留并已做 Foxy 兼容补丁。
- [ ] `costmap_converter/` 是否必须使用仓库 fork。
- [ ] `teb_msgs/` 是否可用系统包替代。

### 5.2 清理原则

- [ ] 能用 Foxy 系统包替代的，不保留源码 fork。
- [ ] 必须保留的第三方 fork 移到 `third_party/`。
- [ ] 主 launch 不启动自定义 BT/behavior，除非有明确任务依赖。
- [ ] 删除前先 `rg` 确认无引用。
- [ ] 删除前保留 git tag 或迁移分支。

## 6. Phase F：launch/config 重建

目标：建立可维护、可切换、适合试验机的启动体系。

### 6.1 目标目录

```text
launch/
├── bringup.launch.py
├── sensing.launch.py
├── localization.launch.py
├── tf.launch.py
├── perception.launch.py
├── navigation.launch.py
├── flight_control.launch.py
├── mission.launch.py
└── debug.launch.py

config/
├── hardware/ports.yaml
├── hardware/mavros.yaml
├── hardware/servo.yaml
├── hardware/openmv.yaml
├── sensing/livox.yaml
├── localization/point_lio.yaml
├── tf/static_transforms.yaml
├── perception/obstacle_segmentation.yaml
├── navigation/nav2.yaml
├── navigation/teb.yaml
├── mission/targets.yaml
├── mission/mission.yaml
└── mapping/pcd2pgm.yaml
```

### 6.2 launch 参数

- [x] `dry_run:=true|false`
- [ ] `use_rviz:=true|false`
- [x] `use_mavros:=true|false`
- [x] `use_legacy_mavlink:=true|false`
- [x] `use_servo:=true|false`
- [x] `use_serial_manager:=true|false`
- [x] `use_openmv:=true|false`
- [ ] `use_navigation:=true|false`
- [ ] `use_perception:=true|false`
- [ ] `localization:=point_lio`
- [ ] `params_dir:=...`
- [ ] `log_level:=info|debug`

### 6.3 配置规则

- [x] 硬件参数已拆出 `config/hardware/ports.yaml`、`mavros.yaml`、`servo.yaml`、`openmv.yaml`。
- [ ] 非硬件参数继续从 `drone.yaml` 拆分。
- [x] `servo_node` 只加载舵机参数。
- [ ] 试验机路径不写死在代码里。
- [ ] PCD/map 路径统一放在 mapping config。
- [x] 飞控、舵机、OpenMV 设备名支持参数覆盖。

## 7. Phase G：Foxy + ARM64 约束

- [x] C++ 默认使用 C++14；如使用 C++17，必须显式设置并在 ARM64 验证。当前 `merge_pcd` 已显式 C++17，本机 amd64 构建通过；ARM64 待验证。
- [x] 不使用 Humble-only API。当前已清理 Nav2 smoother/behavior/costmap filter 等非 Foxy API。
- [x] 不使用 `std::filesystem`，除非确认编译器/标准支持。当前 `merge_pcd` 使用 `std::filesystem`，已显式 C++17。
- [x] Python 按 3.8 兼容；新增串口管理和舵机驱动代码已通过 `py_compile`。
- [ ] 避免过度 component composition，优先独立进程方便调试。
- [ ] 所有依赖写入 `package.xml`。
- [ ] 第三方源码锁定版本。
- [ ] 记录 ARM64 构建时间和 CPU 占用。

## 8. Phase H：试验机上机前检查

### 8.1 构建检查

- [ ] `rosdep install --from-paths src --ignore-src -r -y`
- [x] `colcon build`
- [ ] `colcon build --symlink-install`
- [ ] `colcon test`
- [ ] `colcon test-result --verbose`

### 8.2 静态检查

- [x] `ros2 launch robot_bring_up drone.launch.py --show-args`
- [x] `ros2 launch robot_bring_up hardware.launch.py --show-args`
- [x] `timeout 6 ros2 launch robot_bring_up hardware.launch.py dry_run:=true use_mavros:=true use_servo:=true use_serial_manager:=true`
- [ ] `ros2 node list`
- [ ] `ros2 topic list`
- [ ] `ros2 service list`
- [ ] TF 树检查。
- [ ] 参数 dump 检查。

### 8.3 无桨测试

- [ ] MAVROS 连接飞控。
- [ ] MAVROS state 正常。
- [ ] vision pose 输入正常。
- [ ] position setpoint 输出正常。
- [ ] velocity setpoint 输出正常。
- [ ] arming/mode 服务受安全开关保护。
- [ ] RC 接管/急停可用。

### 8.4 rosbag 回放

- [ ] 回放定位/视觉/障碍物数据。
- [ ] mission 状态机按预期转移。
- [ ] dry-run 下投放事件时机正确。
- [ ] dry-run 下穿门 profile 切换事件正确。

### 8.5 实机飞行前

- [ ] 确认螺旋桨安装前完成所有无桨测试。
- [ ] 确认急停/接管人员和流程。
- [ ] 确认电池、电压、定位稳定。
- [ ] 确认 TF 外参和高度偏置。
- [ ] 确认舵机投放机构单独测试通过。

## 9. 第一批建议实施顺序

1. [ ] 在试验机确认 MAVROS Foxy/ARM64 可用版本。
2. [ ] 新建 `robot_tf_manager`，集中 static TF 和高度偏置。
3. [ ] 新建 `flight_control` MAVROS adapter，先兼容旧 `/robot/*`。
4. [x] 给 `servo_node` 增加无硬件保护、dry-run、状态 topic 和 `/servo/drop` service。
5. [ ] 给 `behavior_control` 增加 `/mission/status` 和 `/mission/event`，先不改状态机。
6. [ ] 抽 `target_manager`，合并 4 个静态靶重复流程。
7. [x] 清理 BT/third_party，减少构建负担。
8. [x] 第一批硬件 launch/config 已拆分。
