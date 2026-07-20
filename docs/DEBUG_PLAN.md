# 重构与调试工作顺序规划表

> 本文档规划 behavior_control 包重构和调试的详细步骤与依赖关系。
> 目标环境：ROS2 Foxy + Jetson Nano NX (ARM64) | 开发环境：WSL Ubuntu 20.04 + ROS1 Noetic
>
> **里程计后端**: 同时重构 Point-LIO（ROS2 原生）和 R3LIVE LIO（从 ROS1 移植），
> 两者输出标准化话题接口，通过 `drone.yaml` 的 `odometry.backend` 参数选择。

---

## 总路线图

```
第 0 步 ── 环境准备：WSL 安装 ROS2 Foxy
        │
        ▼
第 1 步 ── Phase A：mavlink_control 重构（热身，Python 简单）
        │
        ▼
第 2 步 ── Phase B：R3LIVE LIO 移植到 ROS2（取代 Point-LIO）
        │   + 同步重构 Point-LIO（两种里程计选其一）
        │
        ▼
第 3 步 ── Phase C：obstacle_segmentation + TfManager
        │
        ▼
第 4 步 ── Phase D：behavior_control HSM 重构（核心大工程）
        │
        ▼
第 5 步 ── Phase E：话题标准化 + drone.yaml 重构 + 集成测试
        │
        ▼
第 6 步 ── Phase F：Jetson 部署 + 实机验证
```

---

## Phase A — mavlink_control 重构

> 目标：提取硬编码常数、统一日志格式、可 mock 测试。Python 包，快速出成果。

| 序号 | 任务 | 预计耗时 | 验证方法 |
|:---:|------|:-------:|----------|
| A.1 | 提取硬编码高度偏移（0.39, 0.08 等）为 YAML 参数 | 15min | `ros2 param list` 可见 |
| A.2 | 统一日志：printf 风格 → `self.get_logger().info`（含颜色常量改用结构化日志） | 10min | 日志输出整洁 |
| A.3 | 串口检测逻辑加固（非 CUAV 时的 fallback） | 10min | 模拟端口不崩溃 |
| A.4 | 编写 `test_mavlink_control.py`（mock serial + mock mavutil） | 25min | `colcon test` 通过 |
| A.5 | git commit "refactor: mavlink_control 参数化 + 测试" | 2min | 干净工作区 |

**产出**：mavlink_control 参数化 ✅ | Python mock 测试框架 ✅

---

## Phase B — 里程计后端：Point-LIO 重构 + R3LIVE LIO 移植

> 目标：两种 LiDAR-惯性里程计发布一致的话题接口，通过 `odometry.backend` 参数切换。

### B1：Point-LIO Foxy 兼容重构

| 序号 | 任务 | 预计耗时 | 验证方法 |
|:---:|------|:-------:|----------|
| B1.1 | CMakeLists.txt 适配 Foxy（C++14, `SingleThreadedExecutor`, 依赖版本） | 15min | `colcon build` 通过 |
| B1.2 | 统一输出话题：确保 `/robot/current_pose` 命名和类型 | 10min | `ros2 topic info` 确认 |
| B1.3 | 提取 TF 偏移常数到 YAML（0.39 高度偏移等） | 10min | 参数可配 |
| B1.4 | 编译验证 + 对比重构前后 `/Odometry` 输出一致性 | 20min | 录制 ros2 bag 对比 |
| B1.5 | git commit "refactor: Point-LIO Foxy 兼容" | 2min | |

### B2：R3LIVE LIO 提取 + 移植到 ROS2

| 序号 | 任务 | 预计耗时 | 验证方法 |
|:---:|------|:-------:|----------|
| B2.1 | 从 R3LIVE 中提取 LIO 核心文件（排除 VIO / meshing / rgb_map / optical_flow / Sophus） | 20min | 文件列表确认 |
| B2.2 | 创建 ROS2 包 `r3live_lio_ros2`（ament_cmake, package.xml） | 10min | `colcon build` 通过 |
| B2.3 | ROS1→ROS2 API 替换（`ros::Publisher`→`rclcpp::Publisher`, `ros::NodeHandle`→`Node` 等） | 60min | 编译通过 |
| B2.4 | 替换 `tf::*` → `tf2_ros::*`（`tf::transform_broadcaster` → `tf2_ros::TransformBroadcaster`） | 20min | 编译通过 |
| B2.5 | 标准化输出话题：/Odometry, /robot/current_pose, /cloud_registered_body | 15min | `ros2 topic list` 一致 |
| B2.6 | 配置 YAML 参数（对应原 r3live_adjusted_config.yaml） | 15min | 参数可加载 |
| B2.7 | 编译 + 与 Point-LIO 输出话题对比验证 | 30min | 录制 bag 对比 |
| B2.8 | git commit "feat: R3LIVE LIO 移植 ROS2" | 2min | |

### B3：里程计选择机制

| 序号 | 任务 | 预计耗时 | 验证方法 |
|:---:|------|:-------:|----------|
| B3.1 | 在 `drone.yaml` 中添加 `odometry.backend` 参数 | 5min | `ros2 param get` |
| B3.2 | `drone.launch.py` 添加条件启动逻辑（if point_lio / r3live_lio） | 15min | 分别启动验证 |
| B3.3 | 验证两种里程计下 behavior_control 的 `/robot/current_pose` 订阅一致 | 10min | topic echo 对比 |
| B3.4 | git commit "feat: 里程计后端可切换" | 2min | |

**产出**：两种里程计均可独立工作 ✅ | 切换参数生效 ✅ | 输出话题一致 ✅

---

## Phase C — obstacle_segmentation 重构 + TfManager 抽取

| 序号 | 任务 | 预计耗时 | 验证方法 |
|:---:|------|:-------:|----------|
| C.1 | 创建 `TfManager` 类，封装所有里程计→相机→map 变换 | 30min | `test_tf_manager.cpp` 通过 |
| C.2 | obstacle_segmentation 接入 TfManager，消除手动 `odom_array` 维护 | 20min | 编译通过 |
| C.3 | 提取 obstacle_segmentation 中的高度常数为 YAML 参数 | 10min | 参数可配 |
| C.4 | 重构 CMakeLists.txt 适配 Foxy | 10min | `colcon build` 通过 |
| C.5 | 编写 `test_tf_manager.cpp`（坐标变换正确性） | 15min | `colcon test` 通过 |
| C.6 | git commit "refactor: obstacle_segmentation + TfManager" | 2min | |

**产出**：TfManager 统一 TF 管理 ✅ | obstacle_segmentation 参数化 ✅ | 测试通过 ✅

---

## Phase D — behavior_control HSM 重构（核心大工程）

> 此时底层依赖（里程计 + mavlink + 障碍物感知）全部就绪。

### D1：基础设施（HSM 引擎 + 服务层）

| 序号 | 任务 | 预计耗时 | 验证方法 |
|:---:|------|:-------:|----------|
| D1.1 | 创建新目录结构（`include/fsm/`, `src/fsm/`, `test/`） | 5min | `ls -R` 完整 |
| D1.2 | 实现 `State` 抽象基类（`state.hpp`） | 10min | 编译通过 |
| D1.3 | 实现 `StateMachine` 引擎（`state_machine.hpp/.cpp`） | 20min | 编译通过 |
| D1.4 | 实现 `GoalManager` | 15min | 测试通过 |
| D1.5 | 实现 `NavClient`（NAV2 action 封装） | 20min | 编译通过 |
| D1.6 | 实现 `TargetDetect`（相机检测结果处理） | 15min | 编译通过 |
| D1.7 | 实现 `ServoController`（舵机参数设置） | 10min | 编译通过 |
| D1.8 | 实现 `ParamController`（TEB 参数档切换） | 20min | 测试通过 |
| D1.9 | 重构 `CMakeLists.txt`（C++14, ament_cmake_gtest） | 15min | `colcon build` 通过 |
| D1.10 | 编写 `test_state_machine.cpp` | 20min | `colcon test` 通过 |
| D1.11 | 编写 `test_goal_manager.cpp` / `test_param_controller.cpp` | 30min | `colcon test` 通过 |
| D1.12 | git commit "feat: HSM 引擎 + 服务层骨架" | 2min | |

### D2：状态迁移

| 序号 | 任务 | 预计耗时 | 验证方法 |
|:---:|------|:-------:|----------|
| D2.1 | `WaitArmState`（原 step 0） | 15min | 编译 + 测试 |
| D2.2 | `TakeoffState`（原 step 1） | 15min | 编译 + 测试 |
| D2.3 | `InitSearchState`（原 step 111-114） | 25min | 编译 + 测试 |
| D2.4 | `StaticTargetState`（**复用核心**，原 step 21-54） | 40min | 4 种参数实例化验证 |
| D2.5 | `RandomTargetState`（原 step 91-104） | 30min | 编译 + 测试 |
| D2.6 | `DoorPassingState`（原 step 71-82） | 35min | 编译 + 测试 |
| D2.7 | `LandingState`（原 step 81-82） | 15min | 编译 + 测试 |
| D2.8 | 重构 `behavior_control.hpp`：服务层 + HSM 替代旧成员 | 20min | 编译通过 |
| D2.9 | 重构 `behavior_control_node.cpp`（`SingleThreadedExecutor`） | 5min | 编译通过 |
| D2.10 | 编写 `test_static_target_state.cpp`（mock 导航） | 20min | `colcon test` 通过 |
| D2.11 | git commit "feat: 状态迁移完成" | 2min | |

**产出**：全部旧逻辑迁入 HSM ✅ | 复用状态消除重复 ✅ | 测试覆盖 ✅

---

## Phase E — 话题标准化 + drone.yaml 重构 + 集成测试

| 序号 | 任务 | 预计耗时 | 验证方法 |
|:---:|------|:-------:|----------|
| E.1 | 新增 `MissionStatus.msg`，HSM update 循环中发布 | 15min | `ros2 topic echo` 可见 |
| E.2 | 重构 `drone.yaml` 为分区结构（targets/heights/teb_profiles/odometry 等） | 20min | `ros2 param list` 验证 |
| E.3 | 为 behavior_control 添加参数动态重载回调 | 15min | 运行时改参数不崩溃 |
| E.4 | 为 servo_node 添加参数验证 | 10min | 异常输入不崩溃 |
| E.5 | `colcon build` 全工作空间编译 | 30min | 0 error 0 warning |
| E.6 | `colcon test` 全工作空间 | 10min | 全部 pass |
| E.7 | git commit "refactor: 接口标准化 + 全 workspace 验证" | 2min | |

**产出**：标准化接口 ✅ | 分区 YAML ✅ | 全 workspace 零警告测试全绿 ✅

---

## Phase F — Jetson 部署 + 实机验证

| 序号 | 任务 | 预计耗时 | 验证方法 |
|:---:|------|:-------:|----------|
| F.1 | Jetson 安装 ROS2 Foxy（如无） | 30min | `ros2 topic list` 可工作 |
| F.2 | SCP 工作空间到 Jetson + `colcon build` | 30min | ARM64 编译通过 |
| F.3 | Jetson 上 `colcon test` | 10min | 全部 pass |
| F.4 | 实机运行完整任务流程，录制 ros2 bag | 一天 | 任务按预期执行 |
| F.5 | 根据实机测试微调参数 | 按需 | 行为符合预期 |
| F.6 | 切换 odometry.backend 测试另一种里程计 | 半天 | 两种均可完成任务 |

---

## 依赖图

```
Phase 0 (环境准备)
  ├─ 0.1 装 ROS2 Foxy
  ├─ 0.2 编译依赖
  ├─ 0.3 旧代码编译验证（记录错误清单）
  ├─ 0.4 git 快照
  └─ 0.5 docs/ 目录

       ▼
Phase A (mavlink_control)
  A.1-A.5 ─── 参数化 + 测试

       ▼
Phase B (里程计后端)
  B1.1-B1.5 ─── Point-LIO Foxy 适配
  B2.1-B2.8 ─── R3LIVE LIO 提取 + ROS2 移植
  B3.1-B3.4 ─── 里程计选择机制

       ▼
Phase C (障碍物分割 + TfManager)
  C.1-C.6 ─── TfManager + obstacle_seg 重构

       ▼
Phase D (behavior_control HSM)
  D1.1-D1.12 ─── HSM 引擎 + 服务层
  D2.1-D2.11 ─── 状态迁移

       ▼
Phase E (标准化 + 集成)
  E.1-E.7 ─── 话题/YAML/全 workspace 编译测试

       ▼
Phase F (Jetson 部署)
  F.1-F.6 ─── 实机验证
```

---

## 每步验证检查清单

| 阶段 | 命令 / 方法 | 通过标准 |
|:----:|------------|----------|
| A | `colcon build --packages-select mavlink_control` | 0 error |
| A | `colcon test --packages-select mavlink_control` | 全部 pass |
| B | `colcon build --packages-select point_lio` / `r3live_lio_ros2` | 0 error |
| B | `ros2 topic echo /Odometry`（对比两种后端） | 话题一致 |
| C | `colcon build --packages-select obstacle_segmentation` | 0 error |
| C | `colcon test --packages-select obstacle_segmentation` | 全部 pass |
| D | `colcon build --packages-select behavior_control` | 0 error, 0 warning |
| D | `colcon test --packages-select behavior_control` | 全部 pass |
| D | `ros2 topic echo /robot/target_pose` 对比新旧 | 行为一致 |
| E | `colcon build` (全 workspace) | 0 error, 0 warning |
| E | `colcon test` (全 workspace) | 全部 pass |
| F | ssh Jetson → `colcon build` | ARM64 编译通过 |
| F | 实机运行完整流程 | 任务完成 |

```
Phase 0 ───────────────────── Phase 1 ─── Phase 2 ─── Phase 3 ─── Phase 4
  │                               │           │           │           │
  ├─ 0.1 装 ROS2 Foxy             │           │           │           │
  ├─ 0.2 编译依赖                 │           │           │           │
  ├─ 0.3 旧代码编译验证            │           │           │           │
  ├─ 0.4 git 快照                 │           │           │           │
  └─ 0.5 docs/ 目录               │           │           │           │
                                  │           │           │           │
                    ┌─────────────┘           │           │           │
                    ▼                         │           │           │
               Phase 1                       │           │           │
               ├─ 1.1 目录结构                │           │           │
               ├─ 1.2 State 基类       ───────┤           │           │
               ├─ 1.3 StateMachine     ───────┤           │           │
               ├─ 1.4 TfManager               │           │           │
               ├─ 1.5 CMakeLists.txt          │           │           │
               ├─ 1.6-1.8 测试                │           │           │
               └─ 1.9 git commit              │           │           │
                                              │           │           │
                    ┌─────────────────────────┘           │           │
                    ▼                                     │           │
               Phase 2a   ──► Phase 2b   ──► Phase 2c    │           │
               ├─ 2.1 WaitArm     ├─ 2.5 StaticTarget     │           │
               ├─ 2.2 Takeoff     ├─ 2.6 GoalManager      │           │
               ├─ 2.3 InitSearch  ├─ 2.7 NavClient        │           │
               ├─ 2.4 测试        ├─ 2.8 TargetDetect     │           │
                                  ├─ 2.9 ServoController  │           │
                                  ├─ 2.10 测试            │           │
                                             │             │           │
                                             ▼             │           │
                                        Phase 2c           │           │
                                        ├─2.11 RandomTgt   │           │
                                        ├─2.12 DoorPassing │           │
                                        ├─2.13 Landing     │           │
                                        ├─2.14 主节点组装   │           │
                                        ├─2.15 node.cpp    │           │
                                        └─2.16 git commit  │           │
                                                           │           │
                    ┌──────────────────────────────────────┘           │
                    ▼                                                  │
               Phase 3                                                │
               ├─ 3.1 MissionStatus.msg                               │
               ├─ 3.2 状态发布                                         │
               ├─ 3.3 YAML 重构                                        │
               ├─ 3.4 动态参数重载                                      │
               ├─ 3.5 obstacle_segmentation 适配                       │
               ├─ 3.6 servo_node 参数验证                               │
               └─ 3.7 git commit                                       │
                                                                       │
                    ┌───────────────────────────────────────────────────┘
                    ▼
               Phase 4
               ├─ 4.1 mavlink 常量提取
               ├─ 4.2 日志统一
               ├─ 4.3 mavlink 测试
               ├─ 4.4 全 workspace 编译
               ├─ 4.5 全 colcon test
               └─ 4.6 git commit

                    ▼
               Phase 5（Jetson 实机）
```

---

## 每步验证检查清单

| 阶段 | 命令 / 方法 | 通过标准 |
|:----:|------------|----------|
| Phase 1 | `colcon build --packages-select behavior_control` | 0 error, 0 warning |
| Phase 1 | `colcon test --packages-select behavior_control` | 全部 pass |
| Phase 2 | `colcon build` 每改完一个文件立刻编译 | 无编译错误 |
| Phase 2 | `colcon test --packages-select behavior_control --ctest-args -R test_xxx` | 关联测试 pass |
| Phase 2 | `ros2 topic echo /robot/target_pose` 对比新旧输出 | 行为一致 |
| Phase 3 | `ros2 param dump /behavior_control_node` | YAML 结构符合预期 |
| Phase 3 | `ros2 param set /servo_node /servo/servo 1` | 不崩溃 |
| Phase 4 | `colcon build` (全 workspace) | 0 error, 0 warning |
| Phase 4 | `colcon test` (全 workspace) | 全部 pass |
| Phase 5 | ssh 到 Jetson → `colcon build` | ARM64 编译通过 |
| Phase 5 | 实机运行 `ros2 launch ...` | 任务流程完成 |
