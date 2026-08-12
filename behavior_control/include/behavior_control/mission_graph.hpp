#ifndef BEHAVIOR_CONTROL__MISSION_GRAPH_HPP_
#define BEHAVIOR_CONTROL__MISSION_GRAPH_HPP_

#include <string>
#include <vector>

namespace behavior_control
{

/// 决策图中的一个可执行节点。
/// state_id 是新调试接口使用的稳定唯一 ID，legacy_step 仅用于对照旧代码。
struct MissionNode
{
  int state_id;
  int legacy_step;
  std::string name;
};

/// 正常任务转移使用的有向边；condition 是供调试器显示的条件说明。
struct MissionEdge
{
  int from_id;
  int to_id;
  std::string condition;
};

/// 当前任务流程的只读图注册表。
///
/// 正常业务根据 edges() 中的条件选择后继；调试时 shift_state() 沿
/// debug_order() 移动，因此 +1/-1 不会依赖旧 step 的稀疏数值。
class MissionGraph
{
public:
  MissionGraph();

  const MissionNode * node_by_id(int state_id) const;
  const MissionNode * node_by_legacy_step(int legacy_step) const;
  const MissionNode * node_by_name(const std::string & name) const;

  /// 沿调试顺序移动 delta 个节点；越界或当前节点未知时返回 nullptr。
  const MissionNode * shift_state(int current_state_id, int delta) const;
  /// 返回正常业务图中从指定节点出发的所有候选后继 ID。
  std::vector<int> outgoing_state_ids(int state_id) const;
  /// 返回 Graphviz DOT 文本；可传入当前节点和执行器状态生成动态高亮图。
  std::string to_dot(
    int current_state_id = -1,
    const std::string & execution_state = "unknown") const;

  const std::vector<MissionNode> & nodes() const;
  const std::vector<MissionEdge> & edges() const;
  const std::vector<int> & debug_order() const;

private:
  void add_edge_by_legacy(int from_step, int to_step, const std::string & condition);

  std::vector<MissionNode> nodes_;
  std::vector<MissionEdge> edges_;
  std::vector<int> debug_order_;
};

}  // namespace behavior_control

#endif  // BEHAVIOR_CONTROL__MISSION_GRAPH_HPP_
