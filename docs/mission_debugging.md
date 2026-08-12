# 决策图与硬跳调试

本文说明 `behavior_control` 当前的状态 ID、有向图和 dry-run 硬跳接口。

## 1. 三种编号不要混用

- `state_id`：新决策图的稳定唯一 ID，硬跳服务使用它。
- `legacy_step`：旧 `behavior_control.cpp` 的数字 step，仅用于新旧代码对照。
- `debug_previous_state_id/debug_next_state_id`：调试顺序中的前后节点。

正常任务根据有向边及条件选择后继；`delta=+1/-1` 沿独立调试顺序移动，
不会把旧 step 的稀疏数值当作流程关系。

当前 ID 分组：

| ID | 流程 |
|---:|---|
| 0-1 | 等待解锁、等待起飞高度 |
| 2 | 起飞后圆周调试 |
| 3-6 | 起飞点附近随机靶初始搜索 |
| 7-22 | 四个静态靶的接近、拉高、识别、投掷 |
| 23-25 | 随机靶搜索点 |
| 26-29 | 随机坦克靶流程 |
| 30-33 | 随机目标流程 |
| 34-40 | 穿门、返航和降落 |
| 41 | 模拟拉高兼容状态 |

完整节点和条件边由 `/mission/graph_dot` 以 Graphviz DOT 文本发布。该话题以
1 Hz 更新，并在状态变化时立即刷新：当前节点会按执行状态着色，当前候选边会高亮。

## 2. 浏览器查看实时拓扑

决策节点启动后，另开终端运行：

```bash
cd /home/sfx/echo_drone
./run_echo_drone.sh mission-graph
```

然后在 Windows 或 WSL 浏览器打开：

```text
http://127.0.0.1:8765
```

页面每秒刷新，显示：

- 每个节点的 `state_id`、状态名和 `legacy_step`；
- 正常业务有向边及转移条件；
- 当前执行器状态和当前节点；
- 当前节点所有候选后继边。

决策节点会为后加入的订阅者以 1 Hz 重发图，但查看器会比较 DOT 内容：拓扑和运行
状态未变化时不调用 Graphviz，也不会重载浏览器中的 SVG；仅实际变化时才更新图。

颜色含义：绿色为 running，黄色为 paused，灰色为 idle，红色为 aborted，
浅蓝色节点/蓝色边表示当前候选后继。

页面顶部还提供调试控制面板：`Start`、暂停、继续、单步、`ID -1/+1`、输入 ID
跳转和中止。也可以直接点击图中的任意节点，确认后跳转至对应 `state_id`。
`打印上下文` 会调用 `/mission/dump_context`，把当前 ID、执行状态、dry-run、TF/位姿
就绪情况及临时任务标志打印到页面的“调试日志”面板，不改变任务流程。
浏览器只连接本机 `127.0.0.1`，控制请求由查看器转交给已有 ROS 服务；不会新增
绕开安全门控的控制通道。绝对/相对跳转仍仅在 `dry_run=true` 且执行器为 `idle` 或
`paused` 时成功，运行中点击会被拒绝。

## 3. 安全启动

```bash
cd /home/sfx/echo_drone
./run_echo_drone.sh mission-dry
```

另开终端：

```bash
cd /home/sfx/echo_drone
source tools/echo_drone_env.bash

ros2 param get /behavior_control_node dry_run
ros2 param get /behavior_control_node autostart
ros2 topic echo /mission/status
```

必须确认 `dry_run=true`、`autostart=false`。硬跳只允许：

- `dry_run=true`；
- 执行器处于 `idle` 或 `paused`。

正常 `running` 和所有非 dry-run 模式都会拒绝硬跳。

## 4. 绝对 ID 跳转

在 idle 状态跳到 ID 7：

```bash
ros2 service call /mission/jump_to_state \
  robot_interfaces/srv/JumpMissionState \
  "{state_id: 7, reset_context: true}"
```

idle 下跳转会同时更新下一次 `/mission/start` 的起点：

```bash
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

运行中需要先暂停再跳：

```bash
ros2 service call /mission/pause std_srvs/srv/Trigger "{}"
ros2 service call /mission/jump_to_state \
  robot_interfaces/srv/JumpMissionState \
  "{state_id: 34, reset_context: true}"
```

## 5. 相对 +1/-1 跳转

```bash
# 调试顺序中的下一节点
ros2 service call /mission/shift_state \
  robot_interfaces/srv/ShiftMissionState \
  "{delta: 1, reset_context: true}"

# 调试顺序中的上一节点
ros2 service call /mission/shift_state \
  robot_interfaces/srv/ShiftMissionState \
  "{delta: -1, reset_context: true}"
```

`delta` 可以大于 1，但越过调试顺序边界时请求会失败，当前状态保持不变。

## 6. 观测和回退

```bash
ros2 topic echo /mission/status
ros2 topic echo /mission/event
ros2 topic echo /mission/graph_dot
ros2 service call /mission/dump_context std_srvs/srv/Trigger "{}"
ros2 service call /mission/abort std_srvs/srv/Trigger "{}"
```

每次硬跳都会发布事件：

- `operator_jump_absolute`：绝对 ID 跳转；
- `operator_jump_forward`：正 delta；
- `operator_jump_backward`：负 delta。

`/mission/graph_dot` 是动态话题，因此现在可直接使用：

```bash
ros2 topic hz /mission/graph_dot
```

预期频率约为 1 Hz。

## 7. 当前实现边界

有向图已经登记节点、正常候选边和调试顺序，但旧任务条件判断仍在
`behavior_control.cpp` 的数字 step 分支中。有向图目前用于观测、导出和安全硬跳；
后续再逐段把正常转移执行迁移到图/HSM 的 `onEnter/onUpdate/onExit`。
