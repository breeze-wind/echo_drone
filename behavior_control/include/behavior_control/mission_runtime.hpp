#ifndef BEHAVIOR_CONTROL__MISSION_RUNTIME_HPP_
#define BEHAVIOR_CONTROL__MISSION_RUNTIME_HPP_

#include <string>

namespace behavior_control
{

/// 决策执行器的生命周期状态。
/// 该状态只描述“任务是否允许推进”，不等同于 PX4 飞行模式，也不替代具体任务状态。
enum class MissionExecutionState
{
  /// 尚未启动，等待 /mission/start。
  kIdle,
  /// 正常运行：允许状态判断 tick 和周期命令输出。
  kRunning,
  /// 已暂停：禁止周期命令输出，仅允许显式单步执行一次判断 tick。
  kPaused,
  /// 已中止：保持停止，必须显式 start 才能重新开始。
  kAborted,
};

/// 为旧 behavior_control 状态机提供启停、暂停和单步门控。
///
/// 这个类不包含 ROS 接口和具体任务逻辑，因此可以独立做单元测试；
/// 后续把数字 step 拆成 HSM 状态类时仍可复用这一层执行控制。
class MissionRuntime
{
public:
  /// autostart=true 时沿用旧节点“启动即运行”的行为，否则从 idle 开始。
  explicit MissionRuntime(bool autostart = true);

  /// 仅允许从 idle 或 aborted 启动。
  bool start();
  /// 仅允许 running -> paused。
  bool pause();
  /// 仅允许 paused -> running。
  bool resume();
  /// 在 paused 状态排队一个决策 tick；已有排队请求时拒绝重复请求。
  bool request_step_once();
  /// 进入 aborted，并清除尚未消费的单步请求。
  bool abort();

  /// 决策定时器每次回调时调用；返回 true 才执行一次状态判断。
  bool consume_decision_tick();
  /// 只有 running 状态允许周期性发布飞控、导航和舵机相关命令。
  bool commands_enabled() const;
  /// 返回当前生命周期枚举值。
  MissionExecutionState state() const;
  /// 返回用于状态话题和日志的人类可读名称。
  std::string state_name() const;

private:
  /// 当前执行器生命周期。
  MissionExecutionState state_;
  /// paused 状态下是否已排队一次单步判断。
  bool step_once_pending_{false};
};

}  // namespace behavior_control

#endif  // BEHAVIOR_CONTROL__MISSION_RUNTIME_HPP_
