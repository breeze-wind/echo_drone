# 仿真代码导读

本文面向准备阅读和继续重构仿真链路的开发者，重点说明 `sls_circle_controller`、`sls_circle_controller_cpp` 和 `sls_qsf_core` 中各文件的意义。

当前仿真分三层：

1. 轻量闭环：`sls-circle-sim`，用 Python 点质量假 MAVROS 节点快速检查控制器输出。
2. Gazebo 力输入烟测：`sls-circle-gazebo`，用 Gazebo 简化刚体、吊载和 force 插件验证控制器/抗风估计趋势。
3. PX4 SITL 闭环：`sls-px4-sitl`，用 Gazebo Classic + PX4 SITL + MAVROS 验证更接近真实飞控的 offboard 链路。

## 1. 推荐阅读顺序

1. `run_echo_drone.sh`：先看 `sls-circle-sim`、`sls-circle-gazebo`、`sls-px4-sitl` 三个入口如何映射到 launch。
2. `sls_circle_controller/launch/sls_circle.launch.py`：理解单独启动控制器时有哪些参数。
3. `sls_circle_controller/config/sls_circle.yaml`：理解控制器、轻量仿真、Gazebo 桥接共用的默认参数。
4. `sls_circle_controller_cpp/src/sls_circle_controller_node.cpp`：当前主控制器，实机和 PX4 SITL 都应优先看它。
5. `sls_qsf_core/include/sls_qsf_core/qsf_c_api.h` 和 `sls_qsf_core/src/qsf_c_api.cpp`：理解 C++ 控制器如何调用 QSF/SLS 生成算法。
6. `sls_circle_controller/launch/sls_circle_gazebo.launch.py` 和 `sls_circle_controller/sls_circle_controller/gazebo_mavros_bridge_node.py`：理解简化 Gazebo 力输入烟测。
7. `sls_circle_controller/launch/px4_sitl_sls.launch.py` 和 `sls_circle_controller/px4_sitl/`：理解 PX4 SITL 的模型、world 和 MAVROS 接入。

## 2. 启动文件

| 文件 | 作用 | 当前关注点 |
|---|---|---|
| `sls_circle_controller/launch/sls_circle.launch.py` | 单独启动 C++ SLS 控制器，可接真实 MAVROS 或外部假数据 | 适合实机 dry-run、只看 setpoint、单独调控制器参数 |
| `sls_circle_controller/launch/sls_circle_sim.launch.py` | 启动 `fake_mavros_sim_node` 和 C++ 控制器 | 适合不打开 Gazebo 的快速算法烟测 |
| `sls_circle_controller/launch/sls_circle_gazebo.launch.py` | 启动 Gazebo 简化模型、桥接节点和 C++ 控制器 | 适合看吊载摆动、Gazebo force、风估计日志 |
| `sls_circle_controller/launch/px4_sitl_sls.launch.py` | 启动 Gazebo Classic、PX4 SITL、MAVROS 和 C++ 控制器 | 适合验证真正 MAVROS/PX4 offboard 链路 |

## 3. 参数文件

| 文件 | 作用 | 备注 |
|---|---|---|
| `sls_circle_controller/config/sls_circle.yaml` | SLS 控制器、轻量仿真、Gazebo 桥接的共同默认参数 | 修改控制律、任务、风估计、质量、绳长时先看这里 |
| `sls_circle_controller/config/mavros_px4_sitl.yaml` | PX4 SITL 用 MAVROS 插件 allowlist | 只保留状态、命令、local position、setpoint、IMU 等必要插件 |

## 4. ROS2 Python 包文件

| 文件 | 作用 | 备注 |
|---|---|---|
| `sls_circle_controller/setup.py` | 安装 Python 节点、launch、config、world 和 `px4_sitl/` 资源 | 新增仿真资源后要确认是否被安装 |
| `sls_circle_controller/package.xml` | 声明 Python 包依赖 | 这里依赖 `gazebo_ros`、`mavros_msgs`、`sls_qsf_core` 等 |
| `sls_circle_controller/sls_circle_controller/controller_math.py` | 早期 Python 控制器和桥接节点共用的数学函数 | 包括向量、姿态四元数和 LESO 单轴估计器 |
| `sls_circle_controller/sls_circle_controller/drown_qsf.py` | Python 通过 `ctypes` 调用 `libsls_qsf_core.so` 的封装 | 当前主控制器已 C++ 化，但它仍可用于对照和快速试验 |
| `sls_circle_controller/sls_circle_controller/fake_mavros_sim_node.py` | 点质量假 MAVROS 节点 | 订阅调试姿态 setpoint，积分出位姿并发布假 `/mavros/state` |
| `sls_circle_controller/sls_circle_controller/gazebo_mavros_bridge_node.py` | Gazebo force 烟测桥接 | 把 Gazebo odom 包装成 MAVROS 风格 pose/velocity/state，并把姿态目标换成 Gazebo force |
| `sls_circle_controller/sls_circle_controller/sls_circle_controller_node.py` | 早期 Python 版 SLS 控制器 | 保留作对照，不应作为后续实机主路径 |

## 5. C++ 控制器和算法库

| 文件 | 作用 | 备注 |
|---|---|---|
| `sls_circle_controller_cpp/src/sls_circle_controller_node.cpp` | 当前主 SLS/QSF 控制器 | 负责任务阶段、参考轨迹、PD/QSF 控制、风估计、MAVROS setpoint、状态 JSON |
| `sls_circle_controller_cpp/CMakeLists.txt` | 编译 C++ 控制器节点 | 依赖 `sls_qsf_core` 和 MAVROS 消息 |
| `sls_qsf_core/include/sls_qsf_core/qsf_c_api.h` | QSF 生成代码的稳定 C ABI | 避免 C++ 节点直接依赖生成代码内部符号 |
| `sls_qsf_core/src/qsf_c_api.cpp` | C ABI 到生成 QSF 函数的包装 | 做空指针检查并返回错误码 |
| `sls_qsf_core/src/generated/*` | 从原算法或工具生成的 QSF 控制源码 | 不建议手动加业务注释或重构，公式对照应另写文档 |
| `sls_qsf_core/CMakeLists.txt` | 编译共享库 `libsls_qsf_core.so` | 同时安装头文件和导出 ament 目标 |

## 6. Gazebo 简化烟测资源

| 文件 | 作用 | 备注 |
|---|---|---|
| `sls_circle_controller/worlds/sls_circle_force.world` | 简化刚体 + 吊绳 + 负载 + Gazebo force 插件 | 不含 PX4、电机、姿态内环，只验证外力闭环趋势 |

## 7. PX4 SITL 资源

| 文件或目录 | 作用 | 备注 |
|---|---|---|
| `sls_circle_controller/px4_sitl/worlds/empty.world` | PX4 SITL 默认空场景 | 供 `px4_sitl_sls.launch.py` 默认加载 |
| `sls_circle_controller/px4_sitl/worlds/windy.world` | 带 Gazebo world 风标签的场景 | 当前控制器主要使用代码内风估计和日志，world 风要单独验证 |
| `sls_circle_controller/px4_sitl/models/px4vision_sls/px4vision_sls.sdf` | PX4 vision 吊载模型 | 包含 PX4 Gazebo 插件、电机模型和吊载结构 |
| `sls_circle_controller/px4_sitl/models/iris*`、`px4vision*`、`gps`、`asphalt_plane` | PX4/Gazebo fallback 模型资源 | 主要解决本地 Gazebo 找不到 `model://...` 的问题 |
| `sls_circle_controller/px4_sitl/ros1_launch/*` | 原 ROS1 launch 归档 | 用于公式、参数和流程对照，不在 ROS2 colcon 中直接运行 |
| `sls_circle_controller/px4_sitl/legacy_src/*` | 原 Gazebo 插件和 QSF 代码归档 | 用于对照，不建议混入当前 ROS2 主路径 |
| `sls_circle_controller/px4_sitl/scripts/run_pure_gazebo_sim.sh` | 原纯 Gazebo 脚本归档 | 只作为参考脚本 |

## 8. 三个仿真入口的理论效果

### 8.1 `sls-circle-sim`

这个模式不打开 Gazebo，也不启动 PX4。

它用点质量动力学模拟 MAVROS 位姿和状态，适合快速检查控制器是否能发布姿态目标、参考轨迹和状态 JSON。

### 8.2 `sls-circle-gazebo`

这个模式打开 Gazebo 简化模型，桥接节点把控制器输出的姿态和推力换算成世界系 force。

它能看吊载摆动和风估计日志，但不能代表真实 PX4 内环、电机混控和飞控保护逻辑。

### 8.3 `sls-px4-sitl`

这个模式启动 PX4 SITL 和 MAVROS，控制器向 `/mavros/setpoint_raw/attitude` 发布真实 offboard setpoint。

它默认是 `takeoff_then_circle`，理论上会先起飞、保持，再进入绕圈。

后续应拆清 `point`、`swing`、`wind` 三个档位，避免默认参数难以判断是否实际施加抗风补偿。

## 9. 继续重构时的注意事项

- 不要把 `sls-circle-gazebo` 的稳定性等同于 PX4 SITL 稳定性。
- 不要把 `sls_qsf_core/src/generated/*` 当作普通手写业务代码重构。
- 修改坐标系时必须同时检查 ENU、NED、FRD、body z、MAVROS setpoint 和 Gazebo world force。
- 风扰补偿只允许作用于水平 x/y，z 轴风估计和补偿应保持禁用或清零。
- 实机前必须先确认 MAVROS heartbeat、外部里程计、OFFBOARD、arming 和 RC 接管。

