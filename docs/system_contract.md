# 现有系统接口清单

本文档记录当前 `echo_drone` 主线系统的可运行接口合同。后续重构必须先兼容这些接口，再逐步替换内部实现。

## 1. 当前环境确认

当前开发环境不是试验机：

| 项目 | 当前值 |
|---|---|
| 系统 | Ubuntu 20.04.6 LTS on WSL2 |
| 内核 | `microsoft-standard-WSL2` |
| 架构 | `amd64` |
| Python | 3.8.10 |
| ROS2 | Foxy 已安装：`/opt/ros/foxy` |
| ROS1 | Noetic 已保留：`/opt/ros/noetic` |
| colcon | 已安装：`/usr/bin/colcon` |
| MAVROS | Foxy 包已安装：`ros-foxy-mavros`, `ros-foxy-mavros-extras` |
| Nav2 | Foxy 包已安装：`ros-foxy-navigation2`, `ros-foxy-nav2-bringup` |
| GeographicLib | `egm96-5` geoid 数据已安装 |

Shell 环境已改为可切换：

- 默认新 shell：ROS2 Foxy，`ROS_ACTIVE_ENV=ros2_foxy`。
- ROS1 切换：`ROS_DEFAULT=noetic bash` 或在当前 shell 执行 `use_ros1`。
- ROS2 切换：`use_ros2`。
- 原 ROS2 apt 源从 USTC 切到 Aliyun，因为 USTC 源索引可用但 Foxy `.deb` 文件返回 404；备份文件为 `/etc/apt/sources.list.d/ros-fish.list.bak-codex-20260715`。

当前仓库构建入口确认：

- `colcon list` / `colcon graph` 可运行。
- `echo_drone_v2/` 曾与主线重复 `behavior_control`、`mavlink_control`、`robot_interfaces`；用户确认后已删除。
- 当前 `colcon list` 为 14 个 ROS2 包，旧 BT navigator/behavior/action 包已通过 `COLCON_IGNORE` 排除。
- 当前 clean `colcon build` 已通过：14 packages finished，约 7min 11s。

目标试验环境按以下约束设计：

| 项目 | 目标 |
|---|---|
| OS | Ubuntu 20.04 |
| ROS2 | Foxy |
| 架构 | ARM64 |
| 飞控接口 | MAVROS，不再用 `pymavlink` 直连 |
| 验证方式 | 先 dry-run / rosbag / 无桨测试，再上试验机 |

## 2. 当前核心功能链路

```text
Livox MID-360 + IMU
  -> livox_ros_driver2（third_party 源码包，依赖 /usr/local Livox-SDK2）
  -> Point-LIO
  -> /Odometry, /robot/current_pose, /cloud_registered_body
  -> obstacle_segmentation
  -> /cloud_obstacle
  -> Nav2 + TEB
  -> /cmd_vel
  -> mavlink_control
  -> PX4

behavior_control
  -> Nav2 NavigateToPose action
  -> /robot/target_pose
  -> /robot/nav_state
  -> /robot/obstacle_height
  -> /servo_node/set_parameters
  -> servo_node
```

## 3. 当前包职责

| 包/目录 | ROS 包名 | 当前职责 | 重构策略 |
|---|---|---|---|
| `Point-LIO/` | `point_lio` | Livox LIO、发布位姿和点云 | 保留，后续仅清理 frame/配置 |
| `obstacle_segmentation_tc/` | `obstacle_segmentation` | 从点云中提取障碍物 | 保留算法，重构 TF/参数 |
| `teb_local_planner/` | 多包 | TEB、costmap_converter、teb_msgs | Foxy 无 ROS2 TEB apt 包，当前 fork 保留并已补 Foxy API |
| `robot_behavior_tree/` | 多包 | 自定义 BT navigator/behavior/action | 已排除默认构建；主流程改用官方 Nav2 BT/recoveries |
| `behavior_control/` | `behavior_control` | 任务决策、目标流程、投掷、穿门 | 必须重构为可调试状态机 |
| `flight_control/` | `flight_control` | MAVROS adapter，兼容旧 `/robot/*` 飞控接口 | 默认飞控适配层，后续补安全门和试验机验证 |
| `mavlink_control/` | `mavlink_control` | `pymavlink` 直连 PX4 | 仅保留为 fallback，默认不启动 |
| `servo_node/` | `servo_node` | STM32 串口舵机 | 已参数化，支持 dry-run、状态 topic、命令 topic、drop service，并兼容旧参数接口 |
| `robot_serial_manager/` | `robot_serial_manager` | 串口设备枚举与状态发布 | 只做检查，不打开业务串口，避免和 MAVROS/驱动抢设备 |
| `robot_bring_up/` | `robot_bring_up` | 全局 launch/config | 必须拆分 launch/config |
| `robot_interfaces/` | `robot_interfaces` | 自定义视觉/舵机消息 | 短期保持兼容，不先改消息字段 |
| `robot_mapping/` | `pcd2pgm`, `merge_pcd` | 地图工具 | 修 bug 后拆纯逻辑测试 |

## 4. 当前话题接口

### 4.1 定位与点云

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|---|---|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Livox driver | Point-LIO | 原始 Livox 点云 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Livox driver | Point-LIO | IMU |
| `/Odometry` | `nav_msgs/msg/Odometry` | Point-LIO | Nav2/TEB | 当前里程计 |
| `/robot/current_pose` | `geometry_msgs/msg/TransformStamped` | Point-LIO | behavior_control, flight_control, obstacle_segmentation | 当前位姿主接口 |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Point-LIO | RViz/调试 | map 系点云 |
| `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | Point-LIO | obstacle_segmentation | 机体系点云 |
| `/cloud_obstacle_new` | `sensor_msgs/msg/PointCloud2` | Point-LIO | 暂未作为主链路 | Point-LIO 内部障碍物输出 |
| `/Laser_map` | `sensor_msgs/msg/PointCloud2` | Point-LIO | RViz/调试 | 地图点云 |
| `/path` | `nav_msgs/msg/Path` | Point-LIO | RViz/调试 | 位姿路径 |

### 4.2 障碍物和 Nav2

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|---|---|---|---|---|
| `/cloud_obstacle` | `sensor_msgs/msg/PointCloud2` | obstacle_segmentation | local_costmap, global_costmap | 主避障点云 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 controller/TEB | flight_control | 导航速度指令 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | behavior_control | Nav2/RViz | 调试或目标点输入 |

### 4.3 任务决策接口

| 话题/服务 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/robot/arm_state` | `std_msgs/msg/Bool` | flight_control -> behavior_control | 飞控解锁状态 |
| `/robot/target_pose` | `geometry_msgs/msg/TransformStamped` | behavior_control -> flight_control | 非 Nav2 模式下的位置设定点 |
| `/robot/nav_state` | `std_msgs/msg/Bool` | behavior_control -> flight_control | `true` 使用 `/cmd_vel`；`false` 使用 `/robot/target_pose` |
| `/robot/passing_door_state` | `std_msgs/msg/Bool` | behavior_control -> flight_control | 穿门姿态/坐标逻辑标志 |
| `/robot/turning_state` | `std_msgs/msg/Bool` | behavior_control -> flight_control | 穿门前转向标志 |
| `/robot/obstacle_height` | `std_msgs/msg/Float64` | behavior_control -> obstacle_segmentation | 障碍物高度筛选上限 |
| `/robot/clear_state` | `std_msgs/msg/Bool` | behavior_control -> obstacle_segmentation/Nav2 相关逻辑 | 清图/清障碍状态 |
| `/camera/choose` | `std_msgs/msg/Bool` | behavior_control -> 相机选择节点 | `true` 选择 D435，当前只看到发布端 |
| `/robot/image_location` | `robot_interfaces/msg/ImageLocation` | 视觉 -> behavior_control | D435 目标识别结果 |
| `/robot/usb_camera` | `robot_interfaces/msg/ImageLocation` | USB 相机 -> behavior_control | 起飞点随机靶识别结果 |
| `/servo_node/set_parameters` | `rcl_interfaces/srv/SetParameters` | behavior_control -> servo_node | 旧兼容接口，改 `/servo/servo` 触发投放 |
| `/servo/command` | `std_msgs/msg/Int32` | behavior_control/debug -> servo_node | 新舵机命令 topic，取值范围由 `servo.yaml` 约束 |
| `/servo/drop` | `std_srvs/srv/Trigger` | behavior_control/debug -> servo_node | 新投放服务，发送 `drop_command` |
| `/servo/status` | `std_msgs/msg/String` JSON | servo_node -> debug/monitor | 舵机串口状态、dry-run 状态、最近命令和错误 |
| `/flight_control/status` | `std_msgs/msg/String` JSON | flight_control -> debug/monitor | MAVROS 连接、解锁、模式、setpoint 与位姿新鲜度 |
| `/hardware/serial_status` | `std_msgs/msg/String` JSON | robot_serial_manager -> debug/monitor | 串口设备存在性、逻辑 owner、fallback 匹配状态 |
| `/hardware/rescan_serials` | `std_srvs/srv/Trigger` | debug/monitor -> robot_serial_manager | 手动重新扫描串口设备 |
| `/controller_server/set_parameters` | `rcl_interfaces/srv/SetParameters` | behavior_control -> Nav2 controller_server | 穿门时动态修改 TEB 参数 |
| `/local_costmap/local_costmap/set_parameters` | `rcl_interfaces/srv/SetParameters` | behavior_control -> local_costmap | 穿门时动态修改 `robot_radius` |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | behavior_control -> Nav2 | 静态靶/随机靶/穿门导航 |

### 4.4 自定义消息

当前 `robot_interfaces/msg/Receive/ImageLocation.msg`：

```text
uint8 id
float32 image_x
float32 image_y
```

当前 `robot_interfaces/msg/Transmit/ServoPos.msg`：

```text
uint8 servo_pos
```

短期重构必须保持 `ImageLocation.id/image_x/image_y` 字段兼容，不能直接改成 V2 中的 `target_type/u/v`。

## 5. 当前 TF / Frame 合同

### 5.1 当前 frame

| Frame | 当前来源 | 说明 |
|---|---|---|
| `map` | Point-LIO / Nav2 | 世界系 |
| `odom` | static TF: `map -> odom` | 当前 launch 中偏移 z=0.39 |
| `livox_raw` | Point-LIO 参数变更后使用 | 原始雷达坐标，当前为未提交改动 |
| `livox` | static TF: `livox_raw -> livox` | 项目约定雷达系，当前为未提交改动 |
| `camera_link` | static TF: `livox -> camera_link` | D435 相机系 |
| `mavlink_body` | static TF: `livox -> mavlink_body` | 飞控机体系 |

### 5.2 当前硬编码偏置

| 位置 | 数值 | 当前含义 | 重构要求 |
|---|---:|---|---|
| `robot_bring_up/launch/drone.launch.py` | `map -> odom z=0.39` | 雷达离地高度/装载偏置 | 移入 `robot_tf_manager` 配置 |
| `robot_bring_up/launch/drone.launch.py` | `livox -> mavlink_body z=-0.08` | 雷达到飞控机体偏置 | 移入 `robot_tf_manager` 配置 |
| `robot_bring_up/launch/drone.launch.py` | `livox -> camera_link [0.065,-0.065,-0.26]` | 雷达到相机外参 | 移入 `robot_tf_manager` 配置 |
| `behavior_control.cpp` | `current_z = z + 0.39` | 决策高度修正 | 禁止业务节点直接加偏置 |
| `mavlink_control_node.py` | `current_z = z + 0.39`, MAVLink z 用 `0.08` | PX4 NED/传感器高度换算 | 迁移到 MAVROS adapter/TF |
| `obstacle_segmentation.cpp` | `current_z = z + 0.27` | 障碍物高度筛选修正 | 参数化并由 TF 管理 |

## 6. 当前 launch 入口

| 文件 | 作用 | 当前问题 |
|---|---|---|
| `robot_bring_up/launch/drone.launch.py` | 主启动：Livox、Point-LIO、Nav2、障碍物、RViz、静态 TF | static TF 写死；未启动 mavlink/servo/behavior_control |
| `robot_bring_up/launch/bringup_launch.py` | Nav2 bringup 包装 | 基本来自 Nav2 模板 |
| `robot_bring_up/launch/navigation_launch.py` | Foxy Nav2 navigation nodes | 已改为 `controller_server`、`planner_server`、`recoveries_server`、`bt_navigator`、`waypoint_follower` |
| `robot_bring_up/launch/localization_launch.py` | map_server / AMCL 等模板逻辑 | 对当前无人机 LIO 体系可能部分冗余 |
| `behavior_control/launch/behavior_control.launch.py` | 启动任务决策 | 单独入口 |
| `robot_bring_up/launch/hardware.launch.py` | 统一硬件入口：serial manager、servo、MAVROS 或 legacy MAVLink fallback | 默认 dry-run，不在非试验机环境误开飞控串口 |
| `robot_bring_up/launch/mavlink_control.launch.py` | 启动 `pymavlink` 节点 | 保留为 fallback，默认不由硬件入口启动 |
| `robot_bring_up/launch/serial.launch.py` | 旧 `robot_serial` 入口 | 已改为 deprecated wrapper，只启动 dry-run serial manager |
| `robot_bring_up/launch/servo.launch.py` | 启动舵机节点 | 已加载 `config/hardware/servo.yaml` 并支持 `dry_run` 参数 |
| `obstacle_segmentation_tc/launch/obstacle_segmentation.launch.py` | 启动障碍物分割 | 默认加载 `drone.yaml` |
| `Point-LIO/launch/pointlio.launch.py` | 启动 Point-LIO | 默认加载 `drone.yaml` |

串口统一管理的使用说明见 `docs/serial_hardware_management.md`。

目标 launch 需要提供：

```text
launch/bringup.launch.py
launch/sensing.launch.py
launch/localization.launch.py
launch/tf.launch.py
launch/perception.launch.py
launch/navigation.launch.py
launch/flight_control.launch.py
launch/mission.launch.py
launch/debug.launch.py
```

## 7. 当前配置入口

当前主要配置集中在 `robot_bring_up/config/drone.yaml`，包含：

| 配置段 | 内容 |
|---|---|
| `behavior_control_node` | 目标点、任务顺序、高度、是否投掷、穿门、投掷偏置、穿门 TEB 参数 |
| `mavlink_control_node` | 巡航高度、穿门高度 |
| `pcd2pgm_node` | PCD 转地图参数 |
| `map_server` | map yaml 路径 |
| `obstacle_segmentation_node` | 点云输入输出、滤波范围、体素参数 |
| `laser_mapping` | Point-LIO 所有参数 |
| `controller_server` | TEB 参数 |
| `local_costmap` | 局部代价地图 |
| `global_costmap` | 全局代价地图 |
| `planner_server` | 全局规划器 |
| `recoveries_server` | Foxy Nav2 recovery 插件 |
| `waypoint_follower` | waypoint 配置 |

目标配置拆分：

```text
config/hardware/mavros.yaml
config/hardware/servo.yaml
config/hardware/ports.yaml
config/hardware/openmv.yaml
config/sensing/livox.yaml
config/localization/point_lio.yaml
config/tf/static_transforms.yaml
config/perception/obstacle_segmentation.yaml
config/navigation/nav2.yaml
config/navigation/teb.yaml
config/mission/targets.yaml
config/mission/mission.yaml
config/mapping/pcd2pgm.yaml
```

## 8. 必须保留的外部行为

重构后必须保持这些行为等价，除非试验机确认替换：

- Point-LIO 仍输出 `/Odometry` 和 `/robot/current_pose`。
- Nav2/TEB 仍消费 `/Odometry` 与 `/cloud_obstacle`。
- 决策层仍能通过导航 action 发送目标点。
- 决策层仍能切换导航模式和直接位置控制模式。
- 飞控仍收到视觉定位输入；MAVROS 迁移后由 `/mavros/vision_pose/pose` 等接口完成。
- 穿门时仍能降低/调整 TEB 参数和机体半径。
- 舵机仍能按原投放序号触发。
- 视觉识别消息字段 `id/image_x/image_y` 先保持兼容。

## 9. 待试验机确认项

| 项 | 需要确认 |
|---|---|
| MAVROS | Foxy/ARM64 可用版本、安装方式、topic/service 名称 |
| PX4 | 是否接收 MAVROS vision pose；需要的 frame 和 ENU/NED 语义 |
| 串口 | 飞控、舵机、Livox 在 ARM64 上的设备名/udev 规则 |
| 性能 | Point-LIO + Nav2 + MAVROS + 障碍物分割的 CPU 占用 |
| TF | 真实外参和高度偏置，尤其 `0.39/0.27/0.08` 的来源 |
| Nav2 | Foxy 下 TEB 参数名是否与当前 Humble/模板一致 |
| 安全 | 无桨 offboard 测试、RC 接管、急停/降落策略 |
