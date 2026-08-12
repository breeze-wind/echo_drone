#include "behavior_control/mission_graph.hpp"

#include <algorithm>
#include <sstream>

namespace behavior_control
{

MissionGraph::MissionGraph()
{
  // ID 按调试时希望看到的规范流程连续排列。ID 一旦对外使用就不再复用；
  // 以后即使插入新节点，也通过 debug_order_ 指定前后关系，不依赖整数相邻。
  const struct NodeDefinition
  {
    int legacy_step;
    const char * name;
  } definitions[] = {
    {0, "wait_arm"},
    {1, "wait_takeoff"},
    {110, "takeoff_circle"},
    {111, "initial_random_search_1"},
    {112, "initial_random_detect_1"},
    {113, "initial_random_search_2"},
    {114, "initial_random_detect_2"},
    {21, "static_target_1_approach"},
    {22, "static_target_1_raise"},
    {23, "static_target_1_detect"},
    {24, "static_target_1_eject"},
    {31, "static_target_2_approach"},
    {32, "static_target_2_raise"},
    {33, "static_target_2_detect"},
    {34, "static_target_2_eject"},
    {41, "static_target_3_approach"},
    {42, "static_target_3_raise"},
    {43, "static_target_3_detect"},
    {44, "static_target_3_eject"},
    {51, "static_target_4_approach"},
    {52, "static_target_4_raise"},
    {53, "static_target_4_detect"},
    {54, "static_target_4_eject"},
    {91, "random_search_1"},
    {92, "random_search_2"},
    {93, "random_search_3"},
    {61, "random_tank_approach"},
    {62, "random_tank_raise"},
    {63, "random_tank_detect"},
    {64, "random_tank_eject"},
    {101, "random_target_approach"},
    {102, "random_target_raise"},
    {103, "random_target_detect"},
    {104, "random_target_eject"},
    {71, "door_approach"},
    {72, "door_turn"},
    {73, "door_midpoint"},
    {74, "door_exit"},
    {75, "return_home"},
    {81, "land_home"},
    {82, "land_after_door"},
    {300, "simulated_raise"},
  };

  for (std::size_t index = 0; index < sizeof(definitions) / sizeof(definitions[0]); ++index) {
    nodes_.push_back(
      MissionNode{static_cast<int>(index), definitions[index].legacy_step,
        definitions[index].name});
    debug_order_.push_back(static_cast<int>(index));
  }

  // 启动和起飞后的调试分支。
  add_edge_by_legacy(0, 1, "armed");
  add_edge_by_legacy(1, 110, "takeoff_circle_enabled");
  add_edge_by_legacy(1, 111, "!takeoff_circle_enabled");
  add_edge_by_legacy(110, 110, "continuous_circle");
  add_edge_by_legacy(111, 112, "arrived_search_point_1");
  add_edge_by_legacy(112, 21, "random_targets_found");
  add_edge_by_legacy(112, 113, "detect_timeout");
  add_edge_by_legacy(113, 114, "arrived_search_point_2");
  add_edge_by_legacy(114, 21, "targets_found_or_timeout");

  // 四个静态靶流程：是否投掷、是否继续遍历由运行参数决定。
  add_edge_by_legacy(21, 22, "target_1_need_hit");
  add_edge_by_legacy(21, 31, "target_1_skip");
  add_edge_by_legacy(22, 23, "raise_complete");
  add_edge_by_legacy(23, 24, "detect_complete");
  add_edge_by_legacy(24, 31, "pass_all_static_targets");
  add_edge_by_legacy(24, 91, "skip_remaining_static_targets");

  add_edge_by_legacy(31, 32, "target_2_need_hit");
  add_edge_by_legacy(31, 41, "target_2_skip");
  add_edge_by_legacy(32, 33, "raise_complete");
  add_edge_by_legacy(33, 34, "detect_complete");
  add_edge_by_legacy(34, 41, "eject_complete");

  add_edge_by_legacy(41, 42, "target_3_need_hit");
  add_edge_by_legacy(41, 51, "target_3_skip");
  add_edge_by_legacy(42, 43, "raise_complete");
  add_edge_by_legacy(43, 44, "detect_complete");
  add_edge_by_legacy(44, 51, "eject_complete");

  add_edge_by_legacy(51, 52, "target_4_need_hit");
  add_edge_by_legacy(51, 61, "target_4_skip");
  add_edge_by_legacy(52, 53, "raise_complete");
  add_edge_by_legacy(53, 54, "detect_complete");
  add_edge_by_legacy(54, 91, "eject_complete");

  // 随机靶、穿门和降落分支。
  add_edge_by_legacy(91, 61, "random_targets_found");
  add_edge_by_legacy(91, 92, "arrived_search_point_1");
  add_edge_by_legacy(92, 61, "random_targets_found");
  add_edge_by_legacy(92, 93, "arrived_search_point_2");
  add_edge_by_legacy(93, 61, "search_finished");
  add_edge_by_legacy(61, 62, "arrived_random_tank");
  add_edge_by_legacy(62, 63, "raise_complete");
  add_edge_by_legacy(63, 64, "detect_complete");
  add_edge_by_legacy(64, 101, "eject_complete");
  add_edge_by_legacy(101, 102, "arrived_random_target");
  add_edge_by_legacy(102, 103, "raise_complete");
  add_edge_by_legacy(103, 104, "detect_complete");
  add_edge_by_legacy(104, 71, "eject_complete");
  add_edge_by_legacy(71, 72, "passing_door_enabled");
  add_edge_by_legacy(71, 75, "skip_door");
  add_edge_by_legacy(72, 74, "turn_complete");
  add_edge_by_legacy(73, 74, "arrived_door_midpoint");
  add_edge_by_legacy(74, 82, "door_exit_reached");
  add_edge_by_legacy(75, 81, "home_reached");
}

const MissionNode * MissionGraph::node_by_id(int state_id) const
{
  const auto iterator = std::find_if(
    nodes_.begin(), nodes_.end(),
    [state_id](const MissionNode & node) {return node.state_id == state_id;});
  return iterator == nodes_.end() ? nullptr : &(*iterator);
}

const MissionNode * MissionGraph::node_by_legacy_step(int legacy_step) const
{
  const auto iterator = std::find_if(
    nodes_.begin(), nodes_.end(),
    [legacy_step](const MissionNode & node) {return node.legacy_step == legacy_step;});
  return iterator == nodes_.end() ? nullptr : &(*iterator);
}

const MissionNode * MissionGraph::node_by_name(const std::string & name) const
{
  const auto iterator = std::find_if(
    nodes_.begin(), nodes_.end(),
    [&name](const MissionNode & node) {return node.name == name;});
  return iterator == nodes_.end() ? nullptr : &(*iterator);
}

const MissionNode * MissionGraph::shift_state(int current_state_id, int delta) const
{
  const auto current = std::find(debug_order_.begin(), debug_order_.end(), current_state_id);
  if (current == debug_order_.end()) {
    return nullptr;
  }
  const auto current_index = std::distance(debug_order_.begin(), current);
  const auto target_index = current_index + delta;
  if (target_index < 0 || target_index >= static_cast<std::ptrdiff_t>(debug_order_.size())) {
    return nullptr;
  }
  return node_by_id(debug_order_[static_cast<std::size_t>(target_index)]);
}

std::vector<int> MissionGraph::outgoing_state_ids(int state_id) const
{
  std::vector<int> result;
  for (const auto & edge : edges_) {
    if (edge.from_id == state_id) {
      result.push_back(edge.to_id);
    }
  }
  return result;
}

std::string MissionGraph::to_dot(
  int current_state_id, const std::string & execution_state) const
{
  std::string current_color = "#d9e2ec";
  if (execution_state == "running") {
    current_color = "#8ce99a";
  } else if (execution_state == "paused") {
    current_color = "#ffe066";
  } else if (execution_state == "aborted") {
    current_color = "#ff8787";
  } else if (execution_state == "idle") {
    current_color = "#ced4da";
  }

  const auto * current = node_by_id(current_state_id);
  const auto outgoing = outgoing_state_ids(current_state_id);
  const auto is_outgoing = [&outgoing](int state_id) {
      return std::find(outgoing.begin(), outgoing.end(), state_id) != outgoing.end();
    };

  std::ostringstream dot;
  dot << "digraph mission {\n"
    // 自上而下排列，避免完整任务链被压成一条超宽横图。
    // newrank 让跨 cluster 的业务边仍能保持清晰的拓扑层次。
      << "  graph [rankdir=TB, newrank=true, splines=polyline, bgcolor=\"#ffffff\", " <<
    "pad=\"0.3\", nodesep=\"0.25\", ranksep=\"0.45\", " <<
    "fontname=\"DejaVu Sans\", fontsize=18, labelloc=t, label=\"" <<
    "Echo Drone 决策图 | execution=" << execution_state <<
    " | current_id=" << current_state_id;
  if (current != nullptr) {
    dot << " | " << current->name;
  }
  dot << "\"];\n" <<
    "  node [shape=box, style=\"rounded,filled\", fillcolor=\"#f8f9fa\", " <<
    "color=\"#868e96\", fontname=\"DejaVu Sans\", fontsize=10, margin=\"0.12,0.07\"];\n" <<
    "  edge [color=\"#adb5bd\", fontcolor=\"#495057\", " <<
    "fontname=\"DejaVu Sans\", fontsize=8, arrowsize=0.7];\n";

  const struct Cluster
  {
    int first_id;
    int last_id;
    const char * name;
    const char * color;
  } clusters[] = {
    {0, 6, "启动与初始搜索", "#e7f5ff"},
    {7, 22, "静态靶流程", "#ebfbee"},
    {23, 33, "随机靶流程", "#fff9db"},
    {34, 40, "穿门、返航与降落", "#fff0f6"},
    {41, 41, "兼容调试状态", "#f3f0ff"},
  };

  for (std::size_t cluster_index = 0;
    cluster_index < sizeof(clusters) / sizeof(clusters[0]); ++cluster_index)
  {
    const auto & cluster = clusters[cluster_index];
    dot << "  subgraph cluster_" << cluster_index << " {\n" <<
      "    label=\"" << cluster.name << "\"; color=\"" << cluster.color <<
      "\"; style=\"rounded,filled\"; fillcolor=\"" << cluster.color << "40\";\n";
    for (const auto & node : nodes_) {
      if (node.state_id < cluster.first_id || node.state_id > cluster.last_id) {
        continue;
      }
      dot << "    s" << node.state_id << " [label=\"ID " << node.state_id << "\\n" <<
        node.name << "\\nlegacy " << node.legacy_step << "\"";
      if (node.state_id == current_state_id) {
        dot << ", fillcolor=\"" << current_color <<
          "\", color=\"#212529\", penwidth=3";
      } else if (is_outgoing(node.state_id)) {
        dot << ", fillcolor=\"#d0ebff\", color=\"#1c7ed6\", penwidth=2";
      }
      dot << "];\n";
    }
    dot << "  }\n";
  }
  for (const auto & edge : edges_) {
    dot << "  s" << edge.from_id << " -> s" << edge.to_id << " [label=\"" <<
      edge.condition << "\"";
    if (edge.from_id == current_state_id) {
      dot << ", color=\"#1971c2\", fontcolor=\"#1864ab\", penwidth=2.5";
    }
    dot << "];\n";
  }
  dot << "}\n";
  return dot.str();
}

const std::vector<MissionNode> & MissionGraph::nodes() const
{
  return nodes_;
}

const std::vector<MissionEdge> & MissionGraph::edges() const
{
  return edges_;
}

const std::vector<int> & MissionGraph::debug_order() const
{
  return debug_order_;
}

void MissionGraph::add_edge_by_legacy(
  int from_step, int to_step, const std::string & condition)
{
  const auto * from = node_by_legacy_step(from_step);
  const auto * to = node_by_legacy_step(to_step);
  if (from != nullptr && to != nullptr) {
    edges_.push_back(MissionEdge{from->state_id, to->state_id, condition});
  }
}

}  // namespace behavior_control
