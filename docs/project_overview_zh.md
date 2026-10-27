# echo_drone 项目总览

本文根据仓库目录、ROS 包、主要 launch、配置和现有专题文档整理，目的是说明系统由哪些部分组成以及从哪里继续阅读。它不是硬件验收报告；串口、网络、飞行和仿真结论均需按对应指南在目标环境复核。

## 项目是什么

`echo_drone` 是 ROS 2 无人机任务与吊载控制工作区。系统将 LiDAR/IMU 里程计、点云处理、任务决策、MAVROS 飞控接口、执行机构和 SLS/QSF 控制算法组合在一起，并提供 Gazebo 与 PX4 SITL 仿真入口。当前开发主线面向 ROS 2 Foxy；仓库同时保留历史代码、上游依赖和实验材料。

## 系统数据流

```text
Livox MID360 + IMU
        │
        ▼
 livox_ros_driver2 ──点云/IMU──> Point-LIO
                                  │
                     ┌────────────┴────────────┐
                     ▼                         ▼
       /robot/current_pose、/Odometry    /cloud_registered_body
                     │                         │
                     ▼                         ▼
          behavior_control          obstacle_segmentation
                     │                         │
      目标/任务状态/导航接口    /cloud_obstacle（sensing-nav）
                     │                         │
                     ▼                         ▼
          flight_control          Nav2 / TEB costmap
              │         ▲                 │
              │ MAVROS  │目标/速度        └──导航结果/速度──┐
              ▼         │                                  │
             PX4 <──── MAVROS adapter <───────────────────┘
               ▲
               │
 behavior_control ───舵机命令──> servo_node
```

这张图概括仓库中可见的模块关系，不代表每种启动模式都会同时运行全部节点。`sensing-costmap` 的 costmap 当前直接订阅 `/cloud_registered_body`，而 `sensing-nav` 的 costmap 才订阅 `/cloud_obstacle`。SLS/QSF 控制器可独立接收 MAVROS 状态并发布控制 setpoint；轻量仿真、简化 Gazebo 和 PX4 SITL 是不同验证层级，入口见 `run_echo_drone.sh`。

## 模块职责

| 模块 | 目录 | 作用与边界 |
|---|---|---|
| 系统启动与配置 | `robot_bring_up/` | 分层 launch、硬件入口、导航启动包装及 `drone.yaml` 等配置 |
| 雷达驱动 | `third_party/livox_ros_driver2/` | Livox ROS 2 驱动；依赖的 SDK 也在 `third_party/` |
| 里程计 | `Point-LIO/` | LiDAR/IMU 融合，向下游提供里程计、位姿和点云 |
| 障碍物处理 | `obstacle_segmentation_tc/` | 点云过滤/分割，ROS 包名为 `obstacle_segmentation` |
| 导航与规划 | `robot_bring_up/`、`teb_local_planner/` | Nav2 启动配置及 TEB/costmap_converter 相关代码 |
| 任务决策 | `behavior_control/` | 任务流程、目标发布和状态调试；具体任务图及运行接口参见任务调试指南 |
| 飞控适配 | `flight_control/` | 把项目接口桥接到 MAVROS 话题/服务；优先级高于旧直连实现 |
| 旧飞控实现 | `mavlink_control/` | `pymavlink` 直连代码，保留作兼容或 fallback |
| 串口与执行机构 | `robot_serial_manager/`、`servo_node/` | 设备状态扫描、舵机串口命令；串口管理器不应替业务节点占用设备 |
| ROS 接口 | `robot_interfaces/` | 自定义消息和服务定义 |
| SLS/QSF 控制 | `sls_circle_controller_cpp/`、`sls_qsf_core/` | C++ 控制器和算法核心；`sls_circle_controller/` 提供 launch、参数、桥接与仿真资源 |
| 地图工具 | `robot_mapping/` | PCD 合并及 PCD 转栅格地图工具 |
| 检测实验 | `detect/` | 相机/YOLO 相关实验脚本和模型材料，不属于主飞控链路 |
| 历史行为树 | `robot_behavior_tree/` | 保留的旧 navigator/行为树代码；是否参与构建取决于目录忽略配置 |

## 主要操作入口

### 统一脚本

根目录 `run_echo_drone.sh` 是人工联调入口。它负责 ROS 环境加载、默认 ROS 环境变量和常见模式分发；支持的模式持续演进，因此以以下命令输出为准：

```bash
./run_echo_drone.sh help
```

通常先做只读环境检查和 dry-run，再分层启动硬件或传感器：

```bash
./run_echo_drone.sh check
./run_echo_drone.sh hardware-dry
./run_echo_drone.sh sensing-check
```

`hardware-real`、`mavros-real`、`servo-real`、`sls-*-real` 及自动 OFFBOARD/解锁模式可能对真实硬件产生控制输出。具体前置条件与现场流程见[实机文档](README.md#实机与硬件)。

### Launch 分层

- `robot_bring_up/launch/hardware.launch.py`：串口管理、舵机、MAVROS 和 adapter 的硬件编排入口。
- `robot_bring_up/launch/sensing.launch.py`：感知/里程计入口；其下由 Livox 驱动、Point-LIO 和相关 TF 组成。
- `Point-LIO/launch/pointlio.launch.py`、`obstacle_segmentation_tc/launch/obstacle_segmentation.launch.py`：单模块入口。
- `robot_bring_up/launch/bringup_launch.py`、`navigation_launch.py`、`localization_launch.py`：Nav2 相关入口。
- `sls_circle_controller/launch/`：SLS 控制器单独运行、轻量仿真、简化 Gazebo 和 PX4 SITL 入口。
- `robot_bring_up/launch/drone.launch.py`：较早的整机组合入口；排查问题时宜先按子系统启动。

## 配置和接口入口

- 全局无人机/导航参数：`robot_bring_up/config/drone.yaml`。
- 硬件设备：`robot_bring_up/config/hardware/ports.yaml`、`mavros.yaml`、`servo.yaml`、`openmv.yaml`。
- SLS/QSF 算法与仿真：`sls_circle_controller/config/sls_circle.yaml`、`mavros_px4_sitl.yaml`。
- 对外 ROS 合同：优先查看[系统接口清单](system_contract.md)，再核对实际 package、launch 和节点实现。
- 个人设备覆盖可通过脚本读取的本地环境文件配置；具体变量以 `run_echo_drone.sh help` 和脚本实现为准，勿将个人串口/网络配置写入共享默认值。

## 当前理解与限制

- MAVROS adapter 是仓库明确推进的主线，旧 `pymavlink` 包仍存在，不能据此推断它已经从所有路径彻底移除。
- `behavior_control` 和 SLS/QSF 控制器是不同职责：前者组织任务和目标，后者负责特定轨迹/姿态控制实验。实际由哪条链路驱动，取决于启动模式和参数。
- `docs/ARCHITECTURE.md`、`REFACTORING_PLAN.md`、`docs/DEBUG_PLAN.md` 中有较多目标设计；应与本页的代码目录和当前 launch 区分。
- `build/`、`install/`、`log/`、数据包和模型文件等属于生成物或实验材料，不应与 ROS 源包混为一谈。
- 2026-10-27 的文档一致性审查做了静态核对和部分 launch/单测检查，没有做完整构建、硬件或飞行验证；详见[文档一致性审查](documentation_consistency_audit.md)。

## 推荐阅读路径

1. 本文了解模块和链路。
2. [文档索引](README.md)按工作场景找到启动或仿真说明。
3. [系统接口合同](system_contract.md)核实话题、frame、配置和兼容要求。
4. 根据目标阅读对应 launch、package.xml 和节点源码。
