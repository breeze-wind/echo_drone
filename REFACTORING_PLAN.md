# 重构方案：RoboPoster V2

> 基于 2025 国机赛无人机代码的完整重构计划。
> 旧仓库 → 打 tag 归档；新仓库 → 从零开始，只搬需要的代码。

---

## 一、现状诊断

| # | 问题 | 严重程度 | 说明 |
|---|------|---------|------|
| 1 | **16 个包混杂** | 🔴 | 上游 fork（teb_local_planner / costmap_converter / Point-LIO）与自研代码放在同一级，`colcon build` 全都编译 |
| 2 | **命名不统一** | 🔴 | `obstacle_segmentation` vs `obstacle_segmentation_tc`，前者是最终版后者是实验版，容易混淆 |
| 3 | **behavior_control.cpp 84KB 单体文件** | 🔴 | 状态机、目标搜索、穿门逻辑、高度控制、MAVLink 心跳、TF 工具函数全部揉在一个文件，无法单元测试 |
| 4 | **drone.yaml 480 行单文件** | 🟡 | 所有节点的参数混在一起，改一个参数要翻半天，没有按功能拆分 |
| 5 | **launch 分散在各包** | 🟡 | 每个包都有自己的 `.launch.py`，真实的启动顺序藏在 `drone.launch.py` 的 include 链里 |
| 6 | **零单元测试** | 🔴 | 只有 Python lint 检查，mission 逻辑没有任何自动化测试，改错只能实机排查 |
| 7 | **无容器化** | 🟡 | 没有 Dockerfile，环境搭建依赖 README 里的 13 步手动操作，新人上手成本高 |
| 8 | **自定义消息太少** | 🟡 | 只有 2 个 msg + 1 个 action，状态传递全靠裸类型，接口不清晰 |
| 9 | **pcd2pgm 20KB 单体** | 🟡 | PCD→PGM 转换器的 IO、参数解析、转换逻辑全写在一起 |

---

## 二、目标架构

### 目录结构

```
roboposter-v2/
├── .docker/
│   └── Dockerfile                   # 基于 humble-dev，一键可构建
├── .github/
│   └── workflows/
│       └── ci.yml                   # colcon build + test
├── config/                           # 按功能拆分的参数文件
│   ├── mission.yaml                 # 任务逻辑参数
│   ├── sensing.yaml                 # LiDAR 参数
│   ├── perception.yaml              # 障碍物分割参数
│   ├── hardware.yaml                # MAVLink + 舵机参数
│   └── navigation/                  # Nav2 参数（4 个文件）
│       ├── teb_params.yaml
│       ├── costmap_common.yaml
│       ├── local_costmap.yaml
│       └── global_costmap.yaml
├── launch/                           # 集中管理启动文件
│   ├── drone.launch.py              # 主入口：include 以下子 launch
│   ├── sensing.launch.py            # Livox driver + Point-LIO
│   ├── perception.launch.py         # 障碍物分割
│   ├── navigation.launch.py         # Nav2 栈（map_server + BT + controller）
│   ├── mission.launch.py            # 行为控制状态机
│   └── hardware.launch.py           # MAVLink + 舵机 + 串口
├── robot_interfaces/                # 自定义消息（原样保留）
│   ├── msg/ImageLocation.msg
│   ├── msg/ServoPos.msg
│   └── action/KeepAwayFromObstacles.action
├── robot_behaviors/                 # Nav2 BT 插件（原样保留）
├── robot_bt_navigator/              # BT 导航器 + 行为树 XML（原样保留）
├── behavior_control/                # [重构核心] 拆分为多文件
│   ├── include/behavior_control/
│   │   ├── mission_state_machine.hpp
│   │   ├── mission_types.hpp
│   │   ├── target_search.hpp
│   │   ├── door_passing.hpp
│   │   └── height_controller.hpp
│   ├── src/
│   │   ├── mission_state_machine.cpp
│   │   ├── target_search.cpp
│   │   ├── door_passing.cpp
│   │   └── height_controller.cpp
│   └── test/
├── mavlink_control/                 # [小改] Python 节点，参数抽取到 yaml
├── servo_node/                      # [小改] Python 节点，参数抽取到 yaml
├── obstacle_segmentation/           # [清理] 保留最终版，删除 _tc 实验包
├── point_lio/                       # [小改] Point-LIO，只保留自定义配置
├── pcd2pgm/                         # [重构] 拆分 IO/参数/转换逻辑
│   ├── include/pcd2pgm/
│   ├── src/
│   └── test/
├── merge_pcd/                       # [保留] 工具类不动
├── third_party/                     # 上游 fork 统一隔离
│   ├── teb_local_planner/
│   ├── costmap_converter/
│   └── costmap_converter_msgs/
├── CMakeLists.txt                   # 顶层仅声明 subdirs（或直接用 colcon）
└── README.md                        # 干净的项目文档
```

### 模块依赖关系

```
┌─────────────┐
│  mission    │  behavior_control（任务状态机）
│  decision   │  └─ 依赖 hardware（MAVLink 起飞/降落）
└──────┬──────┘     └─ 依赖 navigation（发送导航目标）
       │
       ▼
┌─────────────┐
│ navigation  │  Nav2 栈（BT + TEB + costmap）
│  planning   │  └─ 依赖 perception（避障输入）
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ perception  │  obstacle_segmentation（地面剔除）
│  sensing    │  └─ 依赖 point_lio（注册点云）
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ hardware    │  Livox driver → Point-LIO（里程计）
│  raw data   │  MAVLink（飞控） / 舵机 / 串口
└─────────────┘
```

---

## 三、分步实施计划

### Phase 1：基础设施（1-2 天）

| 步骤 | 内容 | 产出 |
|------|------|------|
| 1.1 | 新仓库 `git init`，建立目录骨架 | 干净的仓库 |
| 1.2 | 写 `Dockerfile`（基于 `ros:humble` 或 `humble-dev`，安装 nav2 / PCL / Livox SDK）| 可复现构建 |
| 1.3 | 配置 `.gitignore`（继承旧规则 + Docker 产物）| 版本控制干净 |
| 1.4 | 配置 GitHub Actions CI（`colcon build` + `colcon test`）| 自动验证 |

### Phase 2：复制 + 清理（2-3 天）

| 步骤 | 内容 | 说明 |
|------|------|------|
| 2.1 | 复制 `robot_interfaces` | 原样，不动 |
| 2.2 | 复制 `robot_behaviors`、`robot_bt_navigator` | 原样，不动 |
| 2.3 | 复制 `servo_node`、`mavlink_control` | 抽取硬编码参数到 `config/hardware.yaml` |
| 2.4 | 复制 `obstacle_segmentation`（最终版） | 改名去 `_tc`，删实验包 |
| 2.5 | 复制 `Point-LIO` 到 `point_lio/` | 只保留自定配置和改动，删 PCD 数据 |
| 2.6 | 复制 `pcd2pgm`、`merge_pcd` | 原样，后续再重构 |
| 2.7 | 复制上游 fork 到 `third_party/` | 用 `colcon.meta` 跳过 lint，加速编译 |

### Phase 3：拆分配置文件（1 天）

| 步骤 | 内容 |
|------|------|
| 3.1 | 将 `drone.yaml` 按功能拆成 6 个 yaml |
| 3.2 | 每个子 launch 只加载自己需要的 yaml |
| 3.3 | 主 `drone.launch.py` 只做 include 串联 |

### Phase 4：behavior_control 重构（核心，3-5 天）

**现状**：84KB 单文件包含以下逻辑：

```
behavior_control.cpp
├── 状态机主循环（while + switch on mission_state）
├── 目标搜索（图像坐标 → 导航目标 / 图像话题回调）
├── 穿门逻辑（水平门/垂直门识别 + 穿越策略）
├── 高度控制（根据目标高度调整无人机高度）
├── 起飞/降落流程（MAVLink 指令序列）
├── MAVLink 心跳监控（超时→紧急降落）
└── TF 坐标变换工具函数
```

**目标**：拆分为 6 个文件 + 测试

| 步骤 | 内容 | 可测试性 |
|------|------|---------|
| 4.1 | `mission_types.hpp` — 状态枚举、目标结构体、常量定义 | 不涉及逻辑 |
| 4.2 | `height_controller.hpp/.cpp` — 纯计算类，输入当前高度/目标高度 → 输出控制量 | **单元测试**（mock free）|
| 4.3 | `target_search.hpp/.cpp` — 策略类：图像坐标 → 目标点转换 | **单元测试**（mock ImageLocation）|
| 4.4 | `door_passing.hpp/.cpp` — 穿门状态机子单元 | **单元测试**（mock 传感器输入）|
| 4.5 | `mission_state_machine.hpp/.cpp` — 组合上述策略的主状态机 | **集成测试** |
| 4.6 | `main.cpp` — 只保留 ROS2 胶水代码（节点初始化、订阅、发布、回调转发）| 手动验证与原版等价 |

### Phase 5：pcd2pgm 重构（1 天）

| 步骤 | 内容 |
|------|------|
| 5.1 | 拆分 IO（读 PCD / 写 PGM + YAML）为独立函数 |
| 5.2 | 参数解析改用 yaml 文件而非命令行参数 |
| 5.3 | 转换核心逻辑写单元测试（输入短 PCD → 验证 PGM 像素值）|

### Phase 6：收尾（1 天）

| 步骤 | 内容 |
|------|------|
| 6.1 | 写干净的 README（快速开始 + 架构图 + 参数说明）|
| 6.2 | 全流程验证：`docker build` → `colcon build` → 功能等价于旧仓库 |
| 6.3 | 旧仓库打 tag `archive/2025-competition` 归档 |

---

## 四、时间估算

| Phase | 内容 | 预估 |
|-------|------|------|
| 1 | 基础设施（Docker + CI）| 1-2 天 |
| 2 | 复制 + 清理 | 2-3 天 |
| 3 | 拆分配置 | 1 天 |
| 4 | behavior_control 重构（核心）| 3-5 天 |
| 5 | pcd2pgm 重构 | 1 天 |
| 6 | 收尾（README + 验证）| 1 天 |
| **总计** | | **9-13 天** |

---

## 五、明确排除（不做）

- ❌ **不重写 TEB 规划器** — 上游 fork 稳定，不动
- ❌ **不改 Nav2 BT 树逻辑** — 12 个 XML 行为树保持原样
- ❌ **不引入新框架** — 保持 ROS2 Humble + Nav2 原生栈，不加 state machine library（避免学习成本）
- ❌ **不改 MAVLink 通信协议** — `mavlink_control` 只做配置抽取，不改心跳/消息逻辑
- ❌ **不做模拟器测试** — 没有 Gazebo 环境，保持实机测试
- ❌ **不重构 costmap_converter / teb_msgs** — 上游依赖，原样保留

---

## 六、风险与缓解

| 风险 | 概率 | 缓解措施 |
|------|------|---------|
| behavior_control 重构引入新 bug | 中 | 每拆一个文件就编译验证 + 与旧版行为 diff 对比 |
| Docker 环境与实机环境不一致 | 低 | Dockerfile 严格对照 README 的 13 步安装 |
| 参数拆分遗漏某个关键参数 | 低 | 拆完配置后 `diff` 检查总参数覆盖率 |
| 旧仓库 fork 的上游代码有隐藏改动 | 中 | `git diff` 对比 upstream tag，记录所有自定义修改 |
