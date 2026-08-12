#include "behavior_control/mission_runtime.hpp"

namespace behavior_control
{

MissionRuntime::MissionRuntime(bool autostart)
: state_(autostart ? MissionExecutionState::kRunning : MissionExecutionState::kIdle)
{
}

bool MissionRuntime::start()
{
  // paused 必须使用 resume，避免 start 意外重置一个暂停中的任务。
  if (state_ != MissionExecutionState::kIdle && state_ != MissionExecutionState::kAborted) {
    return false;
  }
  state_ = MissionExecutionState::kRunning;
  step_once_pending_ = false;
  return true;
}

bool MissionRuntime::pause()
{
  if (state_ != MissionExecutionState::kRunning) {
    return false;
  }
  state_ = MissionExecutionState::kPaused;
  return true;
}

bool MissionRuntime::resume()
{
  if (state_ != MissionExecutionState::kPaused) {
    return false;
  }
  state_ = MissionExecutionState::kRunning;
  step_once_pending_ = false;
  return true;
}

bool MissionRuntime::request_step_once()
{
  // 单步只服务于暂停调试，并且同一时刻只保留一个待执行请求。
  if (state_ != MissionExecutionState::kPaused || step_once_pending_) {
    return false;
  }
  step_once_pending_ = true;
  return true;
}

bool MissionRuntime::abort()
{
  if (state_ == MissionExecutionState::kAborted) {
    return false;
  }
  state_ = MissionExecutionState::kAborted;
  step_once_pending_ = false;
  return true;
}

bool MissionRuntime::consume_decision_tick()
{
  // 正常运行时，每个定时器周期都允许执行状态判断。
  if (state_ == MissionExecutionState::kRunning) {
    return true;
  }
  // 暂停状态仅放行一次显式请求；消费后立即恢复门控。
  if (state_ == MissionExecutionState::kPaused && step_once_pending_) {
    step_once_pending_ = false;
    return true;
  }
  return false;
}

bool MissionRuntime::commands_enabled() const
{
  // 单步只推进判断逻辑，不放行持续命令输出，防止调试时误动作。
  return state_ == MissionExecutionState::kRunning;
}

MissionExecutionState MissionRuntime::state() const
{
  return state_;
}

std::string MissionRuntime::state_name() const
{
  switch (state_) {
    case MissionExecutionState::kIdle:
      return "idle";
    case MissionExecutionState::kRunning:
      return "running";
    case MissionExecutionState::kPaused:
      return "paused";
    case MissionExecutionState::kAborted:
      return "aborted";
  }
  return "unknown";
}

}  // namespace behavior_control
