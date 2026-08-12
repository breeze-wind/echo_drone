#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
WS_SETUP="${WS_DIR}/install/setup.bash"
DRONE_PARAMS="${DRONE_PARAMS:-${WS_DIR}/install/robot_bring_up/share/robot_bring_up/config/drone.yaml}"
ECHO_ENV="${WS_DIR}/tools/echo_drone_env.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS2_DISCOVERY_SPIN_TIME="${ROS2_DISCOVERY_SPIN_TIME:-3.0}"

MODE="${1:-full}"
if [[ $# -gt 0 ]]; then
  shift
fi

usage() {
  cat <<'EOF'
用法:
  tools/lidar_nav_test.sh full [额外 launch 参数...]
      一键启动 Livox + Point-LIO + rolling costmap + Nav2 action 服务 + RViz。

  tools/lidar_nav_test.sh append [额外 launch 参数...]
      sensing-costmap 已经在跑时，只追加 recoveries_server、bt_navigator、waypoint_follower。

  tools/lidar_nav_test.sh status
      查看关键 Nav2/Point-LIO topic、action 和 costmap publisher。

  tools/lidar_nav_test.sh goal <x> <y>
      向 /navigate_to_pose 发一个 map 坐标系目标，yaw 固定为 0。
      需要设置 CONFIRM_NAV_GOAL=I_UNDERSTAND。

常用环境变量:
  ROS_DOMAIN_ID=0
  ROS_LOCALHOST_ONLY=1
  ROS2_DISCOVERY_SPIN_TIME=3.0
  LAUNCH_RVIZ=true
  CLEAN_FASTDDS_SHM=1      # 仅在没有 ROS/Livox/Nav2 进程时清理 /dev/shm/fastrtps_*
  DRONE_PARAMS=/path/to/drone.yaml

推荐:
  # 已经跑着 sensing-costmap 时:
  tools/lidar_nav_test.sh append

  # 从零开始:
  tools/lidar_nav_test.sh full

  # 查看服务是否起来:
  tools/lidar_nav_test.sh status
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

source_env() {
  [[ -f "$ECHO_ENV" ]] || die "环境脚本不存在: $ECHO_ENV"
  local old_quiet="${ECHO_DRONE_ENV_QUIET:-}"
  export ECHO_DRONE_ENV_QUIET=1
  # shellcheck source=/dev/null
  source "$ECHO_ENV"
  if [[ -n "$old_quiet" ]]; then
    export ECHO_DRONE_ENV_QUIET="$old_quiet"
  else
    unset ECHO_DRONE_ENV_QUIET
  fi
}

stack_processes() {
  pgrep -af \
    "[l]ivox_ros_driver2_node|[p]ointlio_mapping|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[r]ecoveries_server|[w]aypoint_follower|[o]bstacle_segmentation_node|ros2 launch robot_bring_up sensing" \
    || true
}

require_no_running_stack() {
  local active
  active="$(stack_processes)"
  if [[ -n "$active" ]]; then
    cat >&2 <<EOF
已有感知/Nav2 进程在跑，full 模式不会重复启动，避免 Livox 端口和 ROS 节点名冲突。

当前进程:
$active

如果你现在的 sensing-costmap 已经正常，运行:
  tools/lidar_nav_test.sh append
EOF
    exit 2
  fi
}

require_costmap_stack() {
  pgrep -af "[p]ointlio_mapping" >/dev/null || die "没看到 pointlio_mapping；先跑 sensing-costmap 或用 full 模式。"
  pgrep -af "[c]ontroller_server" >/dev/null || die "没看到 controller_server；先跑 sensing-costmap 或用 full 模式。"
  pgrep -af "[p]lanner_server" >/dev/null || die "没看到 planner_server；先跑 sensing-costmap 或用 full 模式。"
}

clean_fastdds_shm_if_requested() {
  [[ "${CLEAN_FASTDDS_SHM:-0}" == "1" ]] || return 0
  if [[ -n "$(stack_processes)" ]]; then
    die "CLEAN_FASTDDS_SHM=1 只能在没有 ROS/Livox/Nav2 进程时使用。"
  fi
  ros2 daemon stop >/dev/null 2>&1 || true
  rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null || true
}

ros2_topic_list_no_daemon() {
  ros2 topic list --no-daemon --spin-time "$ROS2_DISCOVERY_SPIN_TIME" "$@"
}

ros2_node_list_no_daemon() {
  ros2 node list --no-daemon --spin-time "$ROS2_DISCOVERY_SPIN_TIME" "$@"
}

ros2_service_list_no_daemon() {
  ros2 service list --no-daemon --spin-time "$ROS2_DISCOVERY_SPIN_TIME" "$@"
}

stop_ros2_daemon_for_graph_queries() {
  ros2 daemon stop >/dev/null 2>&1 || true
}

print_nav_actions_best_effort() {
  local output
  if output="$(ros2 action list -t 2>&1)"; then
    printf "%s\n" "$output" | grep -E "navigate|follow|compute|spin|backup|wait" || true
    return
  fi

  echo "ros2 action list failed; Foxy action list has no --no-daemon flag."
  printf "%s\n" "$output" \
    | sed -n '/PermissionError/p;/Operation not permitted/p;/error:/p;/usage:/p' \
    | sed -n '1,4p'
}

print_topic_info_best_effort() {
  local topic="$1"
  local output
  if output="$(ros2 topic info "$topic" -v 2>&1)"; then
    printf "%s\n" "$output"
    return
  fi

  echo "${topic}: unavailable"
  printf "%s\n" "$output" \
    | sed -n '/Unknown topic/p;/PermissionError/p;/Operation not permitted/p;/error:/p' \
    | sed -n '1,3p'
}

print_status() {
  stop_ros2_daemon_for_graph_queries
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
  echo "ROS2_DISCOVERY_SPIN_TIME=${ROS2_DISCOVERY_SPIN_TIME}"
  echo
  echo "=== nodes ==="
  ros2_node_list_no_daemon | sort || true
  echo
  echo "=== key topics ==="
  ros2_topic_list_no_daemon -t \
    | grep -E "livox|Odometry|cloud_registered_body|local_costmap|global_costmap|cmd_vel|tf" \
    || true
  echo
  echo "=== nav services ==="
  ros2_service_list_no_daemon -t \
    | grep -E "costmap|lifecycle|navigate|clear|planner|controller" \
    || true
  echo
  echo "=== nav actions: Foxy has no --no-daemon flag here ==="
  print_nav_actions_best_effort
  echo
  echo "=== costmap publishers: topic info has no --no-daemon flag in Foxy ==="
  print_topic_info_best_effort /local_costmap/costmap
  print_topic_info_best_effort /global_costmap/costmap
}

send_goal() {
  [[ "${CONFIRM_NAV_GOAL:-}" == "I_UNDERSTAND" ]] || \
    die "发 Nav2 目标会让 controller_server 发布 /cmd_vel；确认后设置 CONFIRM_NAV_GOAL=I_UNDERSTAND。"

  local x="${1:-1.0}"
  local y="${2:-0.0}"
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: ${x}, y: ${y}, z: 0.0}, orientation: {w: 1.0}}}}" \
    --feedback
}

case "$MODE" in
  help|-h|--help)
    usage
    ;;

  full)
    source_env
    clean_fastdds_shm_if_requested
    require_no_running_stack
    echo "+ ros2 launch robot_bring_up sensing_nav.launch.py params_file:=${DRONE_PARAMS} launch_rviz:=${LAUNCH_RVIZ:-true} $*"
    exec ros2 launch robot_bring_up sensing_nav.launch.py \
      params_file:="$DRONE_PARAMS" \
      launch_rviz:="${LAUNCH_RVIZ:-true}" \
      "$@"
    ;;

  append)
    source_env
    require_costmap_stack
    echo "+ ros2 launch robot_bring_up nav_services.launch.py params_file:=${DRONE_PARAMS} $*"
    exec ros2 launch robot_bring_up nav_services.launch.py \
      params_file:="$DRONE_PARAMS" \
      "$@"
    ;;

  status)
    source_env
    print_status
    ;;

  goal)
    source_env
    send_goal "$@"
    ;;

  *)
    usage
    die "未知模式: $MODE"
    ;;
esac
