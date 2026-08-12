#include <gtest/gtest.h>

#include "behavior_control/mission_runtime.hpp"

using behavior_control::MissionExecutionState;
using behavior_control::MissionRuntime;

TEST(MissionRuntimeTest, AutostartRunsDecisionAndCommandTicks)
{
  // 兼容旧入口：autostart=true 时节点启动后立即允许判断和命令周期。
  MissionRuntime runtime(true);
  EXPECT_EQ(runtime.state(), MissionExecutionState::kRunning);
  EXPECT_TRUE(runtime.consume_decision_tick());
  EXPECT_TRUE(runtime.commands_enabled());
}

TEST(MissionRuntimeTest, PauseAndSingleStepGateExecution)
{
  // 暂停后禁止周期执行；单步只放行一次判断，不能放行命令输出。
  MissionRuntime runtime(true);
  ASSERT_TRUE(runtime.pause());
  EXPECT_FALSE(runtime.consume_decision_tick());
  EXPECT_FALSE(runtime.commands_enabled());

  ASSERT_TRUE(runtime.request_step_once());
  EXPECT_TRUE(runtime.consume_decision_tick());
  EXPECT_FALSE(runtime.consume_decision_tick());
  EXPECT_FALSE(runtime.commands_enabled());

  ASSERT_TRUE(runtime.resume());
  EXPECT_TRUE(runtime.consume_decision_tick());
  EXPECT_TRUE(runtime.commands_enabled());
}

TEST(MissionRuntimeTest, AbortRequiresExplicitRestart)
{
  // 中止是锁存状态，只有显式 start 才能重新进入 running。
  MissionRuntime runtime(true);
  ASSERT_TRUE(runtime.abort());
  EXPECT_EQ(runtime.state(), MissionExecutionState::kAborted);
  EXPECT_FALSE(runtime.consume_decision_tick());
  EXPECT_FALSE(runtime.commands_enabled());

  ASSERT_TRUE(runtime.start());
  EXPECT_EQ(runtime.state(), MissionExecutionState::kRunning);
}
