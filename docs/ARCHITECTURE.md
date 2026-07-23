# 无人机任务系统架构设计稿

> 当前实装状态见本节，后面的 HSM/服务层内容是行为决策进一步重构的设计稿。
> 现阶段主线优先保证 Foxy、MAVROS、串口统一管理和最小实机联调可用。

## 0. 当前实装架构

当前代码还没有完全改成 HSM，但硬件层和飞控链路已经先做了分层。

```text
Livox MID360
    |
    v
Point-LIO ------------------> /robot/current_pose
    |                              |
    |                              v
    |                       behavior_control
    |                              |
    v                              v
/cloud_registered_body      /robot/target_pose or /cmd_vel
    |                              |
    v                              v
obstacle_segmentation       flight_control/mavros_adapter
    |                              |
    v                              v
/cloud_obstacle             MAVROS topics/services
    |                              |
    v                              v
Nav2 / TEB ----------------> PX4 flight controller
```

运行入口分为三层：

- `run_echo_drone.sh`：人工联调统一入口，封装环境变量、source 和常用 launch。
- `robot_bring_up/launch/hardware.launch.py`：硬件层入口，管理串口检查、舵机、MAVROS 和 MAVROS adapter。
- `robot_bring_up/launch/drone.launch.py`：旧整机入口，仍会一次性拉起 Livox、Point-LIO、Nav2、点云分割和 RViz，调试时不要优先使用。

### 0.1 启动入口职责

根目录 `README.md` 已列出完整脚本模式和 launch 表。架构上按下面边界理解：

| 层级 | 入口 | 职责 |
|---|---|---|
| 操作入口 | `run_echo_drone.sh` | 面向调试人员，统一 source 环境、设置默认 ROS 变量，并把常用启动方式收敛成固定模式 |
| 硬件层 | `robot_bring_up/launch/hardware.launch.py` | 管理串口检查、舵机、MAVROS、MAVROS adapter 和旧 pymavlink fallback |
| 飞控桥接 | `flight_control/launch/mavros_state.launch.py`、`flight_control/launch/mavros_adapter.launch.py` | 前者只检查 MAVROS 到飞控连接，后者只运行旧接口到 MAVROS 的适配器 |
| 感知/里程计 | `livox_ros_driver2`、`Point-LIO/launch/pointlio.launch.py`、`obstacle_segmentation_tc/launch/obstacle_segmentation.launch.py` | 按雷达驱动、LIO、点云障碍物分割的顺序启动 |
| 导航层 | `robot_bring_up/launch/bringup_launch.py`、`navigation_launch.py`、`localization_launch.py` | 封装 Nav2 的地图、规划、控制和生命周期节点 |
| 决策层 | `behavior_control/launch/behavior_control.launch.py` | 当前只用于起飞后圆周运动截断调试，后续再拆 HSM |
| 旧整机入口 | `robot_bring_up/launch/drone.launch.py` | 一次性拉起多模块，容易掩盖 CPU/TF 问题，仅在分层验证后使用 |

`run_echo_drone.sh` 的模式按风险分为四类：

| 类别 | 模式 |
|---|---|
| 只读检查 | `check`、`topics`、`mavros-state` |
| 低风险 dry-run | `hardware-dry`、`serial-dry`、`servo-dry`、`adapter-dry` |
| 子系统启动 | `livox`、`pointlio`、`obstacle`、`behavior`、`nav` |
| 真实硬件 | `mavros-real`、`servo-real`、`hardware-real`、`full` |

飞控链路当前为：

```text
behavior_control
    |
    | /robot/target_pose, /cmd_vel, /robot/nav_state
    v
flight_control/mavros_adapter_node
    |
    | /mavros/vision_pose/pose
    | /mavros/setpoint_position/local
    | /mavros/setpoint_velocity/cmd_vel
    | /mavros/cmd/arming
    | /mavros/set_mode
    v
MAVROS -> PX4
```

`mavlink_control` 仍保留在仓库里，但只作为 fallback，默认不启动。

需要继续处理的架构债：

- `drone.launch.py` 还把感知、里程计、导航、RViz 混在一个入口里，容易掩盖 CPU 或 TF 问题。
- MAVROS 插件白名单/黑名单还没有在 Foxy 下最终定型，当前只读测试会出现 ODOM TF 刷屏。
- TF 偏移分散在 launch、Point-LIO、MAVROS adapter 和行为层参数中，还需要单独收敛为一个 TF 管理包或统一配置。
- `behavior_control` 仍是大 step 状态机，当前只新增了起飞后圆周运动截断逻辑，后续应按下方 HSM 方案继续拆分。

---

> 本文档记录 behavior_control 包重构的架构设计和技术决策。
> 基于 2025 国机赛无人机代码，迁移目标 ROS2 Foxy (Jetson Nano NX ARM64)。
>
> **里程计后端**: 支持 Point-LIO 和 R3LIVE LIO 两种 LiDAR-惯性里程计可切换，
> 通过 `drone.yaml` 中的 `odometry.backend` 参数选择。输出话题接口一致，上层应用无关。

---

## 一、架构总览

```
┌─────────────────────────────────────────────────────────────────────────┐
│                       behavior_control (任务决策层)                      │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                     StateMachine (HSM 引擎)                      │   │
│  │  ┌────────────┐  ┌──────────────┐  ┌────────────────────┐      │   │
│  │  │ WaitArm    │→ │ Takeoff      │→ │ StaticTargetFSM    │      │   │
│  │  │            │  │              │  │ (复用4次,参数化)    │      │   │
│  │  └────────────┘  └──────┬───────┘  └────────────────────┘      │   │
│  │                         │           ┌────────────────────┐      │   │
│  │                  ┌──────▼───────┐   │ RandomTargetState  │      │   │
│  │                  │ InitSearch   │──▶│                    │      │   │
│  │                  │ (随机靶搜索)  │   └────────────────────┘      │   │
│  │                  └──────────────┘           │                    │   │
│  │                  ┌──────────────┐   ┌───────▼────────┐          │   │
│  │                  │ DoorPassing  │◀──│ LandingState    │          │   │
│  │                  │              │   │                 │          │   │
│  │                  └──────────────┘   └─────────────────┘          │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌────────────────────────────────────────────────────────────────┐    │
│  │                    服务层 (Service Layer)                        │    │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐     │    │
│  │  │ TfManager    │  │ GoalManager  │  │ ServoController  │     │    │
│  │  │ (统一TF查询)  │  │ (目标顺序/   │  │ (舵机参数设置)   │     │    │
│  │  │              │  │  偏置管理)   │  │                  │     │    │
│  │  └──────────────┘  └──────────────┘  └──────────────────┘     │    │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐     │    │
│  │  │ NavClient    │  │ TargetDetect │  │ ParamController  │     │    │
│  │  │ (navigate_   │  │ (相机检测    │  │ (TEB/costmap     │     │    │
│  │  │  to_pose封装) │  │  结果处理)   │  │  参数档切换)     │     │    │
│  │  └──────────────┘  └──────────────┘  └──────────────────┘     │    │
│  └────────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────────┘
            │                    │                     │
      话题/动作               话题                 参数服务
            ▼                    ▼                     ▼
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────────┐
│  mavlink_control  │  │ obstacle_        │  │ servo_node           │
│  (MAVLink → PX4)  │  │ segmentation     │  │ (串口→ STM32 舵机)   │
└──────┬───────────┘  └──────────────────┘  └──────────────────────┘
       │ MAVLink
       ▼
┌──────────────┐  ┌──────────────────────┐  ┌──────────────┐
│   PX4 飞控    │  │  里程计后端 (可切换)   │  │  NAV2 导航栈  │
│              │  │  ┌──────────────┐    │  │  (规划器 +    │
│              │  │  │ Point-LIO    │    │  │   TEB + 地图) │
│              │  │  │  或          │    │  └──────────────┘
│              │  │  │ R3LIVE LIO   │    │
│              │  │  └──────────────┘    │
└──────────────┘  └──────────────────────┘
```

---

## 二、HSM 引擎与状态模式

### 2.1 状态基类接口

```cpp
class State {
public:
    virtual ~State() = default;

    /// 进入状态时调用：发导航目标、切换参数、重置计数器
    virtual void onEnter(StateMachine& fsm) {}

    /// 每 tick 调用：检测到达条件、超时判断、转移
    virtual void onUpdate(StateMachine& fsm) = 0;

    /// 退出状态时调用：清理
    virtual void onExit(StateMachine& fsm) {}

    /// 状态名（日志/监控用）
    virtual std::string name() const = 0;
};
```

设计原则：
- **`onUpdate` 只做条件判断和状态转移，不直接发布话题/发 action**
- **实际动作在 `onEnter` 中触发**（发 goal、设目标点、切换参数）
- 每个 `onUpdate` 函数体 ≤ 20 行

### 2.2 StateMachine 引擎

```cpp
class StateMachine {
public:
    void registerState(std::string name, std::unique_ptr<State> state);
    void setInitialState(const std::string& name);
    void transitionTo(const std::string& name);
    void update();  // 每 tick 调用

    std::string currentStateName() const;

    // 用于定时/计数（状态类通过 fsm 访问）
    void resetTimer();
    double elapsedSeconds() const;

private:
    std::map<std::string, std::unique_ptr<State>> states_;
    State* current_ = nullptr;
    rclcpp::Time enter_time_;  // 记录进入当前状态的时间
};
```

转移逻辑：
```cpp
void StateMachine::transitionTo(const std::string& name) {
    if (current_) current_->onExit(*this);
    current_ = states_[name].get();
    enter_time_ = /* 当前时钟 */;
    current_->onEnter(*this);
}
```

### 2.3 可复用子状态机 —— StaticTargetFSM

用**参数化 + 组合**替代 4 套重复代码：

```cpp
class StaticTargetState : public State {
public:
    struct TargetConfig {
        std::vector<double> position;   // map 坐标
        std::string name;               // "car"/"pillbox"/...
        bool need_hit;
        double offset_x, offset_y;      // 投掷偏置
    };

    explicit StaticTargetState(TargetConfig config);

    void onEnter(StateMachine& fsm) override;
    void onUpdate(StateMachine& fsm) override;
    void onExit(StateMachine& fsm) override;
    std::string name() const override;

private:
    enum class Phase { APPROACH, RAISE, DETECT, DESCEND, EJECT };
    Phase phase_ = Phase::APPROACH;
    TargetConfig config_;
    int tick_count_ = 0;  // 超时计数（代替旧代码的 cnt / threshold）
};
```

**对比旧代码**：

| 旧代码 | 新架构 |
|---|---|
| step 21 / 31 / 41 / 51（4 段几乎相同的导航代码） | `StaticTargetState(target_car)` / `StaticTargetState(target_pillbox)` … |
| step 22 / 32 / 42 / 52（4 段相同的拉高代码） | `phase_ = RAISE` 中统一处理 |
| step 23 / 33 / 43 / 53（4 段相同的识别代码） | `phase_ = DETECT` 中统一处理 |
| step 24 / 34 / 44 / 54（4 段相同的投掷代码） | `phase_ = EJECT` 中统一处理 |
| 目标顺序硬编码在 if-else 中 | `GoalManager` 统一管理 |

---

## 三、服务层设计

### 3.1 TfManager — 集中 TF 管理

当前问题：TF 计算分散在 `behavior_control.cpp`（手动维护 `map_to_livox_affine`、`map_to_camera_affine`）和 `obstacle_segmentation.cpp`（手动维护 `odom_array`）。坐标偏移值（`+0.39`、`+0.27`、`-0.08` 等）硬编码在多个文件中。

```cpp
class TfManager {
public:
    explicit TfManager(rclcpp::Node& node);

    // 统一获取变换
    Eigen::Affine3d getMapToLivox() const;
    Eigen::Affine3d getMapToCamera() const;
    Eigen::Affine3d getLivoxToCamera() const;

    // 便捷点变换
    geometry_msgs::msg::PointStamped
    transformPoint(const geometry_msgs::msg::PointStamped& pt,
                   const std::string& target_frame);

    // 由 /robot/current_pose 回调统一更新
    void updateFromPose(const geometry_msgs::msg::TransformStamped& pose);

    // 硬编码偏移的单一来源
    static constexpr double LIVOX_TO_BODY_Z = -0.08;   // livox → mavlink_body
    static constexpr double CAMERA_OFFSET_X = 0.065;
    static constexpr double CAMERA_OFFSET_Y = -0.065;
    static constexpr double CAMERA_OFFSET_Z = -0.26;

private:
    rclcpp::Node& node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    Eigen::Affine3d map_to_livox_;
    Eigen::Affine3d livox_to_camera_;
    mutable std::mutex mutex_;
};
```

### 3.2 GoalManager — 目标点管理

```cpp
class GoalManager {
public:
    struct TargetInfo {
        std::string name;           // "car" / "pillbox" / ...
        std::vector<double> pos;    // [x, y]
        bool need_hit;
        double offset_x, offset_y;
    };

    explicit GoalManager(const std::vector<TargetInfo>& targets);

    // 顺序访问
    const TargetInfo* current() const;
    const TargetInfo* next();
    bool hasNext() const;
    void reset();
    int index() const;           // 当前第几个（0-based）
    int total() const;           // 总数

private:
    std::vector<TargetInfo> targets_;
    int current_index_ = 0;
};
```

静态靶和随机靶共用同一接口，只是 `TargetInfo` 内容不同。

### 3.3 ParamController — 参数档管理

```cpp
class ParamController {
public:
    struct Profile {
        std::string name;
        double max_vel_x, max_vel_y, max_vel_theta;
        double acc_lim_x, acc_lim_y, acc_lim_theta;
        double weight_inflation;
        double robot_radius;
        double max_global_plan_lookahead_dist;
    };

    explicit ParamController(rclcpp::Node& node);

    // 异步应用参数档
    void applyProfile(const Profile& profile);

    // 预定义的参数档
    static Profile normalProfile();
    static Profile doorPassingProfile();
    static Profile searchProfile();

private:
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr
        controller_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr
        costmap_client_;
};
```

### 3.4 NavClient — NAV2 Action Client 封装

```cpp
class NavClient {
public:
    explicit NavClient(rclcpp::Node& node);

    // 发送导航目标
    void sendGoal(double x, double y, double z);

    // 取消当前目标
    void cancelGoal();

    // 是否已到达（由 goal checker 反馈）
    bool isGoalReached() const;

    // 导航状态回调
    using GoalStateCallback = std::function<void(rclcpp_action::ResultCode)>;
    void setOnGoalFinished(GoalStateCallback cb);

private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
        goal_handle_;
};
```

---

## 四、话题接口规范

所有跨节点通信在 `robot_interfaces` 包中统一定义。

### 4.1 现有话题（保持兼容）

| 话题 | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/robot/current_pose` | `TransformStamped` | 里程计后端 (Point-LIO / R3LIVE LIO) | 无人机当前位置（map 系） |
| `/robot/arm_state` | `Bool` | mavlink_control | 飞控解锁状态 |
| `/robot/nav_state` | `Bool` | behavior_control | 是否使用 NAV2 导航 |
| `/robot/target_pose` | `TransformStamped` | behavior_control | 目标位置（定点模式） |
| `/robot/image_location` | `ImageLocation` | d435 相机 | (id, image_x, image_y) |
| `/robot/usb_camera` | `ImageLocation` | USB 相机 | 同上 |
| `/robot/passing_door_state` | `Bool` | behavior_control | 穿门模式标志 |
| `/robot/turning_state` | `Bool` | behavior_control | 转向标志 |
| `/robot/obstacle_height` | `Float64` | behavior_control | 障碍物高度阈值 |
| `/robot/clear_state` | `Bool` | behavior_control | 清除代价地图标志 |
| `/camera/choose` | `Bool` | behavior_control | 相机选择 |
| `/cmd_vel` | `Twist` | TEB 规划器 | 速度指令 |

### 4.2 新增话题

| 话题 | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/robot/mission_status` | `MissionStatus` | behavior_control | 任务状态上报（调试用） |

### 4.3 新增消息定义

```bash
# robot_interfaces/msg/MissionStatus.msg
uint8 state_id           # 对应状态机步骤 ID
string state_name        # 人类可读的状态名
string target_name       # 当前目标名称（如 "car" / "random" / "door"）
float32 progress         # 0.0 ~ 1.0 进度
uint32 elapsed_ms        # 当前状态已耗时
```

---

## 五、参数配置结构（drone.yaml 重构）

旧代码的问题：所有参数平铺在同一个命名空间下，无分区、无注释层次。

新结构：

```yaml
behavior_control_node:
  ros__parameters:
    targets:
      static:
        car:     [2.7, 1.0]
        bridge:  [1.2, -1.0]
        pillbox: [4.7, 2.5]
        tent:    [1.7, 3.2]
        tank:    [4.07, -1.5]
      random_search:
        init_points: [[0.2, 2.2], [0.2, -2.2]]
        search_points: [[5.33, 2.41], [4.68, -0.125], [5.41, -3.94]]
      sequence: ["car", "bridge", "pillbox", "tent"]
      hit_flags: { car: true, bridge: true, pillbox: true, tent: true, tank: true }

    heights:
      cruise: 1.0
      detection: 1.5
      detection_high: 1.4
      door_passing: 0.6
      eject: 0.5
      dynamic_eject: 0.86

    door_passing:
      src_1: [7.72, 3.57]
      src_2: [5.12, 2.33]
      des:   [8.05, -4.2]
      enabled: true
      need_passing_all: false

    eject_offsets:
      - [ -0.08, -0.08 ]
      - [ -0.08,  0.08 ]
      - [  0.08,  0.08 ]

    teb_profiles:
      normal:
        max_vel_x: 0.65
        max_vel_y: 0.65
        max_vel_x_backwards: 1.0
        max_vel_theta: 0.12
        acc_lim_x: 0.25
        acc_lim_y: 0.25
        acc_lim_theta: 0.15
        weight_inflation: 2.0
        robot_radius: 0.11
        max_global_plan_lookahead_dist: 1.45
      door_passing:
        max_vel_x: 0.35
        max_vel_y: 0.35
        max_vel_x_backwards: 0.4
        max_vel_theta: 0.11
        acc_lim_x: 0.25
        acc_lim_y: 0.25
        acc_lim_theta: 0.11
        weight_inflation: 8.0
        robot_radius: 0.08
        max_global_plan_lookahead_dist: 0.7
```

---

## 六、Foxy 兼容性技术要点

| 检查项 | 旧代码 (Humble) | 新代码 (Foxy) | 修改点 |
|---|---|---|---|
| C++ 标准 | C++17 | C++14 | CMakeLists.txt 改为 `-std=c++14` |
| Executor | `StaticSingleThreadedExecutor` | `SingleThreadedExecutor` | `behavior_control_node.cpp` |
| `std::filesystem` | ✅ | ❌ 改用 `boost::filesystem` | 如不涉及可跳过 |
| `tf2_geometry_msgs` | 新路径 | 旧路径 | 改用 `tf2_geometry_msgs/tf2_geometry_msgs.h` |
| Nav2 版本 | Humble 对应版 | Foxy 对应版 | `controller_server` 参数名检查 |
| `rclcpp::Duration` | `from_seconds(0.5)` | 相同 | 保持 |

---

## 七、单元测试策略

```
test/
├── test_state_machine.cpp       # HSM 引擎基础：注册/转移/onEnter/onUpdate/onExit
├── test_static_target_state.cpp # mock NavClient + 验证状态流转
├── test_tf_manager.cpp          # 坐标变换正确性
├── test_goal_manager.cpp        # 目标顺序/边界条件
├── test_param_controller.cpp    # 参数档切换验证
├── test_nav_client.cpp          # action client 封装逻辑
└── test_mavlink_control.py      # Python mock serial + mavlink 消息
```

每个测试文件遵循模式：
```cpp
TEST(StateMachineTest, BasicTransition) {
    // Arrange
    StateMachine fsm;
    fsm.registerState("a", std::make_unique<TestStateA>());
    fsm.registerState("b", std::make_unique<TestStateB>());
    fsm.setInitialState("a");

    // Act
    fsm.update();  // TestStateA::onUpdate 触发 → "b"

    // Assert
    EXPECT_EQ(fsm.currentStateName(), "b");
}
```

## 八、文件目录结构

```
behavior_control/
├── CMakeLists.txt                  # Foxy 兼容：C++14, SingleThreadedExecutor
├── package.xml
├── include/behavior_control/
│   ├── behavior_control.hpp        # 主节点：组装所有服务 + HSM
│   ├── fsm/
│   │   ├── state.hpp               # State 抽象基类
│   │   ├── state_machine.hpp       # HSM 引擎
│   │   ├── takeoff_state.hpp
│   │   ├── search_state.hpp
│   │   ├── static_target_state.hpp
│   │   ├── random_target_state.hpp
│   │   └── door_passing_state.hpp
│   ├── navigation/
│   │   ├── nav_client.hpp
│   │   └── goal_manager.hpp
│   ├── perception/
│   │   ├── target_detection.hpp
│   │   └── tf_manager.hpp
│   └── hardware/
│       ├── servo_controller.hpp
│       └── param_controller.hpp
├── src/
│   ├── behavior_control_node.cpp
│   ├── fsm/
│   │   ├── state_machine.cpp
│   │   ├── takeoff_state.cpp
│   │   ├── search_state.cpp
│   │   ├── static_target_state.cpp
│   │   ├── random_target_state.cpp
│   │   └── door_passing_state.cpp
│   ├── navigation/
│   │   ├── nav_client.cpp
│   │   └── goal_manager.cpp
│   ├── perception/
│   │   ├── target_detection.cpp
│   │   └── tf_manager.cpp
│   └── hardware/
│       ├── servo_controller.cpp
│       └── param_controller.cpp
├── test/
│   ├── test_state_machine.cpp
│   ├── test_static_target_state.cpp
│   ├── test_tf_manager.cpp
│   ├── test_goal_manager.cpp
│   ├── test_param_controller.cpp
│   └── test_nav_client.cpp
├── launch/
│   └── behavior_control.launch.py
└── config/
    └── behavior_control_params.yaml
```

---

## 九、里程计后端抽象（可切换设计）

Point-LIO 和 R3LIVE LIO 可以互换，对上层应用透明。

### 9.1 标准化输出话题接口

两个里程计后端必须发布相同的话题（名称 + 类型 + frame_id 语义一致）：

| 话题 | 类型 | frame_id | child_frame_id | 说明 |
|---|---|---|---|---|
| `/Odometry` | `nav_msgs/Odometry` | `odom` | `livox_raw` | TEB 控制器和 behavior_control 使用 |
| `/robot/current_pose` | `geometry_msgs/TransformStamped` | `map` | `livox` | 各节点订阅的当前位置（已含 0.39m 高度偏移） |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | `livox` | — | 注册到 body 系的点云，obstacle_segmentation 输入 |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | `livox_raw` | — | LiDAR 原始点云（后端订阅作为输入） |
| `/livox/imu` | `sensor_msgs/Imu` | `livox_raw` | — | IMU 数据（后端订阅作为输入） |

### 9.2 配置选择

```yaml
# drone.yaml
odometry:
  backend: "point_lio"        # "point_lio" | "r3live_lio"
  common:
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "livox"
    lidar_topic: "/livox/lidar"
    imu_topic: "/livox/imu"
    publish_pose: true
    pose_topic: "/robot/current_pose"

# 各后端专用参数保持在自己 namespace 下
point_lio:
  laser_mapping:
    # ... Point-LIO 原有参数 ...

r3live_lio:
  r3live_common:
    # ... R3LIVE LIO 参数 ...
```

### 9.3 launch 文件选择逻辑

```python
# drone.launch.py 中
odometry_backend = LaunchConfiguration("odometry_backend", default="point_lio")

if odometry_backend == "point_lio":
    include(pointlio_launch)
elif odometry_backend == "r3live_lio":
    include(r3live_lio_launch)
```

### 9.4 两后端的当前状态

| 后端 | 开发状态 | 需要的工作 |
|---|---|---|
| **Point-LIO** | ✅ 已有 ROS2 代码 | Foxy 兼容适配（CMakeLists 降 C++14, `SingleThreadedExecutor`） |
| **R3LIVE LIO** | ⏳ ROS1 (catkin) 待移植 | 提取 LIO 核心 → ament_cmake 化 → ROS1→ROS2 API 替换 → 测试 |
