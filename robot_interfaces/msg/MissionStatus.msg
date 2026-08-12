# 状态采样时间（ROS 时钟）。
builtin_interfaces/Time stamp
# 与旧 behavior_control.cpp 对照用的数字 step；迁移完成后再考虑移除。
int32 legacy_step
# 新决策图使用的稳定唯一 ID；调试硬跳转以此字段为准。
int32 state_id
# 人类可读的具体任务状态，例如 static_target_1_approach。
string state
# 当前任务阶段，例如 startup、navigate、detect、eject。
string phase
# 当前目标名称，例如 car、random、door。
string target
# 执行器生命周期：idle、running、paused 或 aborted。
string execution_state
# 进入当前具体任务状态后的累计秒数。
float64 elapsed_sec
# true 表示禁止发布真实飞控、导航和舵机命令。
bool dry_run
# 最近一次收到的飞控解锁状态。
bool armed
# 当前是否选择 Nav2 速度控制链路。
bool navigation_active
# 附加健康/任务标志，例如 tf_missing、pose_missing、turning。
string[] flags
# 正常业务有向图中，当前状态所有可能的候选后继 ID。
int32[] outgoing_state_ids
# 调试顺序中的上一/下一节点；-1 表示已到边界。
int32 debug_previous_state_id
int32 debug_next_state_id
