# REASONIX.md

> 📖 项目索引文件。每次会话自动加载。
> 完整重构方案见 [REFACTORING_PLAN.md](./REFACTORING_PLAN.md)。

## 技术栈

- **语言** — C++17 (ament_cmake) + Python3 (ament_python)
- **框架** — ROS2 Humble (rclcpp / rclpy / nav2 / tf2)
- **关键依赖** — PCL（点云滤波/分割）、Livox ros_driver2、TEB 局部规划器、Point-LIO LiDAR-惯性里程计
- **自定义消息** — `robot_interfaces/msg/`（ImageLocation、ServoPos）、自定义 action `KeepAwayFromObstacles.action`

## 目录结构

| 目录 | 说明 |
|---|---|
| `behavior_control/` | C++ 节点——任务逻辑：目标搜索、穿门、高度控制。参数从 YAML 读取 |
| `mavlink_control/` | Python 节点——MAVLink 与飞控通信；发布高度/悬停指令 |
| `obstacle_segmentation_tc/` | C++ 节点——从 `/cloud_registered_body` 剔除地面，发布 `/cloud_obstacle` |
| `Point-LIO/` | C++ 节点——Livox LiDAR-惯性里程计与建图（修改版 LOAM）。发布 `/Odometry`，构建 ikd-tree 地图 |
| `robot_behavior_tree/` | 3 个子包：自定义 BT 插件（`robot_behaviors/`）、BT 导航器（`robot_bt_navigator/`）、action 定义（`robot_msgs/`） |
| `robot_bring_up/` | 启动文件 + `drone.yaml`（所有 ROS2 参数）。入口：`bringup_launch.py` |
| `robot_interfaces/` | 自定义消息包——`ImageLocation.msg`、`ServoPos.msg` |
| `robot_mapping/` | PCD 转二维栅格地图（PCD-to-PGM）和点云合并工具 |
| `servo_node/` | Python 节点——舵机位置控制 |
| `teb_local_planner/` | 内嵌的 TEB 规划器分支——`params/` 目录存放自定义调参 YAML |

## 常用命令

| 命令 | 作用 |
|---|---|
| `colcon build` | 编译所有 ROS2 包（在工作空间根目录执行） |
| `colcon build --packages-select <包名>` | 编译单个包 |
| `ros2 launch robot_bring_up bringup_launch.py` | 启动完整系统 |
| `ros2 launch robot_bring_up drone.launch.py` | 启动无人机相关节点 |

测试/代码检查：`colcon test --packages-select <包名>`（C++ 运行 `ament_lint_auto`，Python 运行 `flake8` + `pep257` + `pytest`）。

## 约定

- C++ 包：`ament_cmake` 构建，头文件放在 `include/<包名>/`。launch 文件在 `launch/`，配置在 `config/`。
- Python 包：`setup.py` 通过 `console_scripts` 注册入口。测试放在 `test/`，使用 `pytest` 标记（`@pytest.mark.linter`、`@pytest.mark.flake8`、`@pytest.mark.pep257`）。
- 所有 launch 文件使用 Python 格式（`*.launch.py`）。
- 自定义 ROS2 消息放在 `msg/` 子目录；自定义 action 放在 `action/` 子目录。
- C++ 编译选项：`-Wall -Wextra -Wpedantic`。
- 版权头文件从 ROS2 OSRF 模板复制（Apache-2.0）。

## 注意事项

- **没有顶层构建文件**——这是一个独立的 ROS2 工作空间。在工作空间根目录执行 `colcon build`，不要在某个包的子目录里执行。
- **`teb_local_planner/` 是内嵌的外部代码**——包含上游的 `.gitlab-ci.yml` / `.travis.yml`；若上游更新后合并，手工修改的部分可能产生冲突。
- **`Point-LIO/PCD/` 存放 LiDAR 地图数据**——大二进制文件（`*.pcd`）。`.gitignore` 已排除 `.pcd` 和 `PCD/` 目录。
- **`drone.yaml` 是所有 ROS2 参数的单一权威来源**——影响 behavior_control、mavlink_control、nav2、TEB、costmap、Point-LIO 等多个节点。修改此处参数会影响多个模块。
- **`colcon test` 默认跳过部分检查**——C++ 包显式屏蔽了 copyright 和 cpplint 检查器（`set(ament_cmake_copyright_FOUND TRUE)` / `ament_cmake_cpplint_FOUND TRUE`）。
