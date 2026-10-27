# 文档索引

此页按“先了解系统，再按工作场景查操作”的顺序组织仓库文档。项目总览、系统接口合同和源码描述当前代码；带有“设计稿”“计划”“交接”“历史”字样的文档只记录方向或某一时点的状态，不能直接当作当前实现。

> 当前文档与代码的静态核对结果见[文档一致性审查（2026-10-27）](documentation_consistency_audit.md)。该文同时列出已修正的文档错误和仍需代码侧处理的接口缺口。

## 项目与接口

| 文档 | 用途 |
|---|---|
| [项目总览](project_overview_zh.md) | 项目目标、模块职责、主要数据流、入口和维护边界；新成员首读 |
| [ROS 数据流审查](data_flow_audit.md) | 按源码追踪包内处理、包间 topic/action/service/TF 流向，并记录接口缺口和运行时核对项 |
| [系统接口合同](system_contract.md) | 当前 ROS 包、话题、消息、frame、配置和 launch 接口；重构兼容性参考 |
| [文档一致性审查](documentation_consistency_audit.md) | 2026-10-27 的文档对源码核对、已修正项、软件检查结果和待处理缺口 |
| [系统架构设计稿](ARCHITECTURE.md) | 前半部分梳理当前架构，后半部分包含 HSM/服务层等目标设计；不是全部已实现功能 |
| [仓库功能与重构分析](repository_function_and_refactor_report.md) | 较早期代码盘点与路线建议，含历史验证结论；不作为当前运行状态的唯一依据 |

## 人类操作

| 文档 | 用途 |
|---|---|
| [无人机系统人类操作手册](operator_manual_zh.md) | 从启动前检查、dry-run、仿真到实机分级操作和停止/排障；现场操作者首选 |

## 实机与硬件

| 文档 | 用途 |
|---|---|
| [现场硬件发布流程](field_hardware_flow.md) | 现场准备、分层启动和验收顺序 |
| [MAVROS 运行指南](mavros_runtime_guide.md) | MAVROS 接口、观测方法和实机链路排查 |
| [SLS 实机启动](sls_real_startup.md) | SLS/QSF 实机控制模式和启动参数 |
| [串口管理](serial_hardware_management.md) | 串口角色、映射配置、状态话题和 dry-run |
| [VMware 与 Livox 集成](vmware_livox_integration.md) | VMware 网络与 MID360 接入记录；环境专用，按需参考 |

## 仿真、算法与调试

| 文档 | 用途 |
|---|---|
| [PX4 SITL/Gazebo](px4_sitl_gazebo.md) | PX4 SITL 依赖、启动方式和本机记录 |
| [仿真代码导读](simulation_code_guide.md) | SLS/QSF 三层仿真结构及相关源码入口 |
| [SLS 圆周控制器](sls_circle_controller.md) | 控制器算法、接口、仿真边界与上机顺序 |
| [任务图调试](mission_debugging.md) | 决策图浏览、跳转、暂停/单步等调试方法 |

## 规划与历史记录

| 文档 | 定位 |
|---|---|
| [重构执行清单](refactor_execution_checklist.md) | 按阶段推进的计划；逐项状态需要对照代码更新 |
| [调试与重构计划](DEBUG_PLAN.md) | 较早的规划稿，包含尚未实施或已经变化的目标；不是当前状态 |
| [重构交接记录（2026-08-03）](refactor_handoff_2026-08-03.md) | 特定日期的工作上下文快照，不是当前状态保证 |
| 仓库根目录 `REFACTORING_PLAN.md`、`REASONIX.md` | 历史方案和旧助手上下文；作为背景材料阅读 |

## 文档维护规则

- 新增操作流程时，写入相应专题文档，并在本页登记；根目录 README 保持为入口页。
- ROS 启动模式以 `./run_echo_drone.sh help` 和对应 launch 实现为准，文档只解释用途和操作顺序。
- 写“当前状态”时注明验证日期、环境和验证范围；未在当前环境复核的旧结论应标为历史记录。
- 设计稿、计划和交接文档不要与运行指南混写，也不要把计划中的目标描述成已实现行为。
- 修改 launch、参数或节点接口后，同步检查 `run_echo_drone.sh help`、对应 YAML 和 `data_flow_audit.md`/`system_contract.md`。
