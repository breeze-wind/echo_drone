#include <gtest/gtest.h>

#include <algorithm>
#include <set>

#include "behavior_control/mission_graph.hpp"

using behavior_control::MissionGraph;

TEST(MissionGraphTest, EveryNodeHasUniqueStableIdentifiers)
{
  MissionGraph graph;
  std::set<int> state_ids;
  std::set<int> legacy_steps;
  std::set<std::string> names;
  for (const auto & node : graph.nodes()) {
    EXPECT_TRUE(state_ids.insert(node.state_id).second);
    EXPECT_TRUE(legacy_steps.insert(node.legacy_step).second);
    EXPECT_TRUE(names.insert(node.name).second);
  }
}

TEST(MissionGraphTest, RelativeShiftUsesDebugOrderAndChecksBounds)
{
  MissionGraph graph;
  const auto * wait_arm = graph.node_by_name("wait_arm");
  ASSERT_NE(wait_arm, nullptr);
  const auto * next = graph.shift_state(wait_arm->state_id, 1);
  ASSERT_NE(next, nullptr);
  EXPECT_EQ(next->name, "wait_takeoff");
  EXPECT_EQ(graph.shift_state(wait_arm->state_id, -1), nullptr);
}

TEST(MissionGraphTest, DoorApproachExposesBothBusinessBranches)
{
  MissionGraph graph;
  const auto * door = graph.node_by_name("door_approach");
  const auto * turn = graph.node_by_name("door_turn");
  const auto * home = graph.node_by_name("return_home");
  ASSERT_NE(door, nullptr);
  ASSERT_NE(turn, nullptr);
  ASSERT_NE(home, nullptr);
  const auto outgoing = graph.outgoing_state_ids(door->state_id);
  EXPECT_NE(std::find(outgoing.begin(), outgoing.end(), turn->state_id), outgoing.end());
  EXPECT_NE(std::find(outgoing.begin(), outgoing.end(), home->state_id), outgoing.end());
}

TEST(MissionGraphTest, DotExportContainsNodeIdsAndConditions)
{
  MissionGraph graph;
  const auto * door = graph.node_by_name("door_approach");
  ASSERT_NE(door, nullptr);
  const auto dot = graph.to_dot(door->state_id, "paused");
  EXPECT_NE(dot.find("digraph mission"), std::string::npos);
  EXPECT_NE(dot.find("passing_door_enabled"), std::string::npos);
  EXPECT_NE(dot.find("current_id=34"), std::string::npos);
  EXPECT_NE(dot.find("#ffe066"), std::string::npos);
  EXPECT_NE(dot.find("ID 34"), std::string::npos);
}
