# 事件发生时间（ROS 时钟）。
builtin_interfaces/Time stamp
# 节点生命周期内单调递增的事件序号，便于日志和 rosbag 对齐。
uint64 sequence
# 转移前后的旧数字 step，用于与历史代码逐项回归。
int32 from_legacy_step
int32 to_legacy_step
# 转移前后的稳定状态 ID。
int32 from_state_id
int32 to_state_id
# 转移前后的命名状态。
string from_state
string to_state
# 触发原因，例如 pause_requested、random_targets_found。
string reason
# 事件发生时的执行器生命周期。
string execution_state
