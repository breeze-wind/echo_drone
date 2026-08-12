#!/usr/bin/env bash
set -Eeuo pipefail

# 重构后的一线操作入口。台架 dry-run、硬件只读检查和旧整机启动都放在
# 同一个脚本下，方便机载单元按子系统逐层验证。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${ECHO_DRONE_WS:-$SCRIPT_DIR}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
WS_SETUP="${WS_DIR}/install/setup.bash"
DRONE_PARAMS="${DRONE_PARAMS:-${WS_DIR}/install/robot_bring_up/share/robot_bring_up/config/drone.yaml}"
ECHO_ENV="${WS_DIR}/tools/echo_drone_env.bash"
MODE="${1:-help}"

# 现场联调可以把固定串口、ROS_DOMAIN_ID 等放进工作区本地文件，避免每次
# 手写一长串环境变量；这个文件默认不提交。
LOCAL_ENV_FILE="${ECHO_DRONE_ENV:-${WS_DIR}/.echo_drone.env}"
if [[ -f "$LOCAL_ENV_FILE" ]]; then
  # shellcheck source=/dev/null
  set +u
  source "$LOCAL_ENV_FILE"
  set -u
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS2_DISCOVERY_SPIN_TIME="${ROS2_DISCOVERY_SPIN_TIME:-3.0}"
FCU_BAUD="${FCU_BAUD:-230400}"
MAVROS_GCS_URL="${MAVROS_GCS_URL:-${GCS_URL:-}}"
SLS_REAL_POSE_TOPIC="${SLS_REAL_POSE_TOPIC:-/mavros/mavros/pose}"
SLS_REAL_VELOCITY_TOPIC="${SLS_REAL_VELOCITY_TOPIC:-/mavros/mavros/velocity_local}"

# help 模式打印这一段；命令说明直接放在脚本内，避免联调时文档和实际命令脱节。
usage() {
  cat <<'EOF'
用法:
  ./run_echo_drone.sh <模式> [额外 ros2 launch 参数...]

安全软件检查:
  check             打印 ROS、工作区和串口设备状态。
  field-precheck    打印现场需要确认的设备、网络和配置文件位置。
  build             构建当前工作区。
  topics            打印当前 ROS 话题列表，绕过 ros2 daemon。
  nodes             打印当前 ROS 节点列表，绕过 ros2 daemon。
  graph             打印当前 ROS 节点、话题、服务和 action 摘要。
  daemon-reset      停止 ros2 CLI daemon；节点不会被停。
  shell             进入已 source echo_drone 环境的交互 shell。
  env               打印当前终端应 source 的环境脚本命令。

硬件层 dry-run:
  hardware-dry      dry-run 启动串口管理、舵机节点和 MAVROS adapter。
  serial-dry        只 dry-run 启动串口管理。
  servo-dry         只 dry-run 启动舵机节点。
  adapter-dry       只用 dry-run 配置启动 MAVROS adapter。

真实硬件层:
  mavros-state      只启动 MAVROS 连接飞控，不启动 adapter，不下发命令。
  mavros-check      限时启动 MAVROS 并采样 /mavros/state。
  mavros-baud-scan  只读扫描常见串口波特率，找到能收到 heartbeat 的配置。
  mavros-point-check 检查 Point-LIO -> adapter -> MAVROS 位姿链路。
  hardware-real     启动串口管理、舵机节点、MAVROS 和 MAVROS adapter。
  mavros-real       启动串口管理、MAVROS 和 MAVROS adapter，不启动舵机。
  mavros-vision     启动串口管理、MAVROS 和 dry-run adapter，不启动舵机。
  servo-real        只启动真实舵机节点，默认设备 /dev/stm32_servo。

实机 SLS 快捷启动:
  real-hold         起飞后只定点悬停，不进入绕圈，不自动解锁。
  real-point        只发位置点，不启用 QSF/抗风扰，不自动解锁。
  real-swing        起飞后 QSF 绕圈，只做绳摆抑制，不自动解锁。
  real-wind         起飞后 QSF 绕圈，同时做绳摆和抗风扰，不自动解锁。
  real-circle       real-wind 的兼容短入口，不自动解锁。
  real-point-auto   只发位置点，并自动请求 OFFBOARD/解锁。
  real-swing-auto   QSF 绳摆抑制，并自动请求 OFFBOARD/解锁。
  real-wind-auto    QSF 绳摆加抗风扰，并自动请求 OFFBOARD/解锁。
  real-circle-auto  real-wind-auto 的兼容短入口，会自动请求 OFFBOARD/解锁。

传感器、里程计、感知和决策:
  livox             启动 Livox MID360s 驱动。
  pointlio          只启动 Point-LIO。
  sensing           启动 Livox、Point-LIO 和感知静态 TF。
  sensing-costmap   启动 sensing，并额外启动障碍物分割和 Nav2 costmap。
  sensing-nav       启动单雷达 sensing、rolling costmap 和 Nav2 action 服务。
  sensing-nav-soft  同 sensing-nav，但强制 RViz 使用软件 OpenGL 渲染。
  sensing-check     限时启动 sensing 并采样 Livox、Point-LIO 和 TF 话题。
  rviz              只打开当前 Point-LIO/Nav2 RViz 配置。
  rviz-soft         同 rviz，但强制使用软件 OpenGL 渲染。
  costmap           不重启雷达/LIO，只追加启动障碍物分割和 Nav2 costmap。
  nav-services      不重启雷达/LIO/costmap，只追加 BT Navigator 等 Nav2 服务。
  nav-test          运行 tools/lidar_nav_test.sh，支持 full/append/status/goal。
  obstacle          只启动点云障碍物分割。
  behavior          只启动 behavior_control。
  mission-dry       安全启动可暂停/单步的决策层，不发布飞控、导航或舵机命令。
                    可加 CLEAN_FASTDDS_SHM=1，在确认没有 ROS 进程后清理 DDS 残留共享内存。
  mission-graph     实时渲染决策拓扑，在浏览器显示 ID、边和当前运行状态。
  nav               只启动 Nav2 bringup。
  sls-circle-dry    只启动 SLS 圆周控制器 dry-run，不写真实 MAVROS 控制话题。
  sls-circle-sim    启动 fake MAVROS 点质量仿真和 SLS 圆周控制器。
  sls-circle-gazebo 启动 Gazebo 力输入烟测和 SLS 圆周控制器。
  sls-px4-sitl      启动移植自原版的 PX4 SITL/Gazebo/MAVROS/SLS 闭环仿真。
  sls-circle-mission-dry dry-run 检查起飞后绕圈状态机。
  sls-takeoff-hold-dry dry-run 检查起飞后定点悬停状态机。
  sls-real-check   只读采样 MAVROS、位姿、SLS 状态和 setpoint 话题。
  sls-point-real   只发位置点到 MAVROS，不启用 QSF/抗风扰，不自动解锁。
  sls-point-real-auto 只发位置点到 MAVROS，并自动请求 OFFBOARD/解锁。
  sls-swing-real   起飞后 QSF 绕圈，只做绳摆抑制，不启用抗风扰。
  sls-swing-real-auto QSF 绳摆抑制，并自动请求 OFFBOARD/解锁。
  sls-swing-wind-real 起飞后 QSF 绕圈，同时启用绳摆抑制和抗风扰。
  sls-swing-wind-real-auto 绳摆加抗风扰，并自动请求 OFFBOARD/解锁。
  sls-takeoff-hold-real 起飞后只定点悬停，不进入绕圈，需要显式确认。
  sls-circle-real   启动起飞后绕圈真实 setpoint 输出，需要显式确认，不自动解锁。
  sls-circle-real-auto 自动请求 OFFBOARD 和解锁，需要额外显式确认。
  full              启动当前旧整机 drone.launch.py，默认关闭 RViz。

常用环境变量覆盖:
  ROS_DOMAIN_ID=0
  ROS_LOCALHOST_ONLY=1
  ROS2_DISCOVERY_SPIN_TIME=3.0
  FCU_URL=/dev/ttyACM0:230400
  FCU_BAUD=230400
  MAVROS_GCS_URL=udp://127.0.0.1:14553@127.0.0.1:14550
  SLS_REAL_POSE_TOPIC=/mavros/mavros/pose
  SLS_REAL_VELOCITY_TOPIC=/mavros/mavros/velocity_local
  DRONE_PARAMS=/path/to/drone.yaml
  ECHO_DRONE_WS=/home/sfx/echo_drone
  ECHO_DRONE_ENV=/path/to/local.env
  KEEP_GAZEBO_PROXY=false
  PX4_DIR=/home/sfx/PX4-Autopilot
  CLEAN_FASTDDS_SHM=1  # 仅 mission-dry 启动前：清理 /dev/shm 下 Fast DDS 残留文件
  ECHO_RVIZ_SOFTWARE=1 # RViz/WSLg OpenGL 异常时，强制软件渲染

本地变量文件:
  默认读取 ./.echo_drone.env；可在里面写 FCU_URL=/dev/ttyACM0:230400。
  需要 QGC 旁路时，写 MAVROS_GCS_URL=udp://127.0.0.1:14553@127.0.0.1:14550。
  如果不写 FCU_URL，脚本会按 /dev/px4_fcu、/dev/ttyACM*、/dev/ttyUSB* 自动选择。

示例:
  ./run_echo_drone.sh check
  ./run_echo_drone.sh field-precheck
  ./run_echo_drone.sh hardware-dry
  ./run_echo_drone.sh mavros-check
  ./run_echo_drone.sh mavros-point-check
  ./run_echo_drone.sh mavros-baud-scan
  ./run_echo_drone.sh sensing-check
  ./run_echo_drone.sh livox
  ./run_echo_drone.sh pointlio rviz:=false
  ./run_echo_drone.sh sensing-nav
  ./run_echo_drone.sh sensing-nav-soft
  ./run_echo_drone.sh shell
  ./run_echo_drone.sh rviz
  ./run_echo_drone.sh rviz-soft
  ./run_echo_drone.sh nav-test append
  ./run_echo_drone.sh sls-circle-sim
  ./run_echo_drone.sh sls-circle-gazebo
  ./run_echo_drone.sh sls-circle-gazebo use_load_pose:=true gui:=false
  PX4_DIR=/home/sfx/PX4-Autopilot ./run_echo_drone.sh sls-px4-sitl
  ./run_echo_drone.sh sls-circle-dry
  ./run_echo_drone.sh sls-circle-mission-dry
  ./run_echo_drone.sh sls-takeoff-hold-dry
  ./run_echo_drone.sh sls-real-check
  ./run_echo_drone.sh real-hold
  ./run_echo_drone.sh real-point
  ./run_echo_drone.sh real-swing
  ./run_echo_drone.sh real-wind
  ./run_echo_drone.sh mavros-vision
  ./run_echo_drone.sh mavros-real
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

# Fast DDS 在 WSL 异常退出后可能遗留共享内存段。症状是 launch 日志显示节点已
# 启动，但另一个终端的 ros2 node/topic list 看不到它。清理前必须确保没有 ROS
# 进程，避免删除当前运行节点仍在使用的通信资源。
ros_processes() {
  pgrep -af "[r]os2 (launch|run|bag)|[b]ehavior_control_node|[l]ivox_ros_driver2_node|[p]ointlio_mapping|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[r]viz2" || true
}

clean_fastdds_shm_if_requested() {
  [[ "${CLEAN_FASTDDS_SHM:-0}" == "1" ]] || return 0
  # CLI daemon 也属于 Fast DDS participant。它不是任务节点，清理前可安全
  # 停止；否则只剩 daemon 时会把用户卡在无法恢复的状态。
  ros2 daemon stop >/dev/null 2>&1 || true
  local active
  active="$(ros_processes)"
  if [[ -n "$active" ]]; then
    cat >&2 <<EOF
ERROR: CLEAN_FASTDDS_SHM=1 只能在没有 ROS 进程时使用；请先正常 Ctrl-C 关闭所有 launch。

当前进程:
$active
EOF
    exit 2
  fi
  rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null || true
  echo "已清理 Fast DDS 遗留共享内存。"
}

# source ROS 和工作区时临时关闭 nounset，因为 Foxy 的 setup 脚本会读取一些
# 可选但未设置的 shell 变量。
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

# 优先显示稳定 udev 别名，再显示原始 USB/ACM 设备；WSL/USBIP 台架环境里
# udev 规则可能尚未生效。
print_devices() {
  local devices=(/dev/px4_fcu /dev/stm32_servo /dev/openmv_serial /dev/ttyUSB* /dev/ttyACM*)
  local found=0

  shopt -s nullglob
  for dev in "${devices[@]}"; do
    if [[ -e "$dev" ]]; then
      ls -l "$dev"
      found=1
    fi
  done
  shopt -u nullglob

  if [[ "$found" -eq 0 ]]; then
    echo "未发现匹配的串口设备。"
  fi
}

detect_fcu_device() {
  if [[ -n "${FCU_DEVICE:-}" ]]; then
    echo "$FCU_DEVICE"
    return
  fi

  local candidates=(/dev/px4_fcu /dev/ttyACM0 /dev/ttyUSB0)
  local dev

  shopt -s nullglob
  candidates+=(/dev/ttyACM* /dev/ttyUSB*)
  shopt -u nullglob

  for dev in "${candidates[@]}"; do
    if [[ -e "$dev" ]]; then
      echo "$dev"
      return
    fi
  done

  echo "/dev/ttyACM0"
}

resolve_fcu_url() {
  if [[ -n "${FCU_URL:-}" ]]; then
    echo "$FCU_URL"
    return
  fi

  echo "$(detect_fcu_device):${FCU_BAUD}"
}

fcu_url_device_path() {
  local fcu_url="$1"

  case "$fcu_url" in
    udp://*|tcp://*|serial://*|file://*)
      return 1
      ;;
  esac

  if [[ "$fcu_url" == /dev/* ]]; then
    echo "${fcu_url%%:*}"
    return 0
  fi

  return 1
}

require_fcu_device() {
  local fcu_url="$1"
  local device_path

  if ! device_path="$(fcu_url_device_path "$fcu_url")"; then
    return
  fi

  [[ -e "$device_path" ]] || die "飞控串口不存在: ${device_path}；请先用 usbipd.exe attach 到 WSL，再重跑当前命令"
}

print_field_precheck() {
  local fcu_url
  fcu_url="$(resolve_fcu_url)"
  echo "Workspace: $WS_DIR"
  echo "ROS_DISTRO: ${ROS_DISTRO:-unknown}"
  echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
  echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY}"
  echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
  echo
  echo "Linux user:"
  id
  echo
  echo "Serial devices:"
  print_devices
  echo
  echo "Network:"
  ip -br addr || true
  ip route || true
  echo
  echo "Field config files:"
  echo "  Local env file: ${LOCAL_ENV_FILE}"
  echo "  FCU serial url: ${fcu_url}"
  echo "  MAVROS GCS url: ${MAVROS_GCS_URL:-<disabled>}"
  echo "  Serial inventory: ${WS_DIR}/robot_bring_up/config/hardware/ports.yaml"
  echo "  MAVROS launch: ${WS_DIR}/flight_control/launch/mavros_state.launch.py"
  echo "  MAVROS adapter: ${WS_DIR}/flight_control/config/mavros_adapter.yaml"
  echo "  Livox network: ${WS_DIR}/third_party/livox_ros_driver2/config/MID360s_config.json"
  echo "  Point-LIO params: ${WS_DIR}/robot_bring_up/config/drone.yaml"
  echo "  Sensing launch: ${WS_DIR}/robot_bring_up/launch/sensing.launch.py"
  echo
  echo "Field flow doc: ${WS_DIR}/docs/field_hardware_flow.md"
}

sample_topic() {
  local topic="$1"
  local type="$2"
  local outfile="$3"
  local seconds="${4:-${TOPIC_SAMPLE_TIMEOUT:-5}}"
  # Foxy 的 topic echo 没有 --no-daemon；它会创建真实订阅者采样。
  timeout -s INT "$seconds" ros2 topic echo "$topic" "$type" > "$outfile" 2>&1 || true
}

sample_topic_if_listed() {
  local topic="$1"
  local type="$2"
  local outfile="$3"
  local topics_file="$4"
  local seconds="${5:-${TOPIC_SAMPLE_TIMEOUT:-5}}"
  if awk -v topic="$topic" '$1 == topic { found = 1 } END { exit found ? 0 : 1 }' "$topics_file"; then
    sample_topic "$topic" "$type" "$outfile" "$seconds"
  else
    echo "topic absent in direct-discovery list: ${topic}" > "$outfile"
  fi
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

print_topic_info_best_effort() {
  local topic="$1"
  echo "--- ${topic}"
  ROS2CLI_NO_DAEMON=1 ros2 topic info "$topic" -v 2>&1 | sed -n '1,120p' || true
}

stop_ros2_daemon_for_graph_queries() {
  ros2 daemon stop >/dev/null 2>&1 || true
}

print_ros_actions_best_effort() {
  local output
  if output="$(ros2 action list -t 2>&1)"; then
    printf "%s\n" "$output" | sort
    return
  fi

  echo "ros2 action list failed; Foxy action list has no --no-daemon flag."
  printf "%s\n" "$output" \
    | sed -n '/PermissionError/p;/Operation not permitted/p;/error:/p;/usage:/p' \
    | sed -n '1,4p'
}

print_ros_graph_direct() {
  stop_ros2_daemon_for_graph_queries
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
  echo "ROS2_DISCOVERY_SPIN_TIME=${ROS2_DISCOVERY_SPIN_TIME}"
  echo
  echo "=== nodes: direct discovery ==="
  ros2_node_list_no_daemon | sort || true
  echo
  echo "=== topics: direct discovery ==="
  ros2_topic_list_no_daemon -t | sort || true
  echo
  echo "=== services: direct discovery ==="
  ros2_service_list_no_daemon -t | sort || true
  echo
  echo "=== actions: Foxy has no --no-daemon flag here ==="
  print_ros_actions_best_effort
}

match_lines() {
  local pattern="$1"
  local file="$2"
  if command -v rg >/dev/null 2>&1; then
    rg "$pattern" "$file"
  else
    grep -E "$pattern" "$file"
  fi
}

match_lines_numbered() {
  local pattern="$1"
  local file="$2"
  if command -v rg >/dev/null 2>&1; then
    rg -n "$pattern" "$file"
  else
    grep -En "$pattern" "$file"
  fi
}

match_lines_quiet() {
  local pattern="$1"
  local file="$2"
  if command -v rg >/dev/null 2>&1; then
    rg -q "$pattern" "$file"
  else
    grep -Eq "$pattern" "$file"
  fi
}

print_key_log_lines() {
  local logfile="$1"
  match_lines_numbered \
    "FCU URL|open failed|opened successfully|HEARTBEAT|connected|bind failed|Node init finished|No point|ERROR|Error|Traceback|process has died" \
    "$logfile" | sed -n '1,120p' || true
}

# 用 exec 让 Ctrl-C 和退出码直接归属到被启动的 ROS 命令。
run() {
  echo "+ $*"
  exec "$@"
}

run_launch() {
  echo "+ ros2 launch $*"
  exec ros2 launch "$@"
}

require_sls_real_control() {
  local mode_name="$1"
  local action_desc="$2"
  [[ "${CONFIRM_SLS_REAL_CONTROL:-}" == "I_UNDERSTAND" ]] || \
    die "${mode_name} 会${action_desc}；确认后设置 CONFIRM_SLS_REAL_CONTROL=I_UNDERSTAND"
}

require_sls_auto_takeoff() {
  local mode_name="$1"
  [[ "${CONFIRM_SLS_AUTO_TAKEOFF:-}" == "I_UNDERSTAND" ]] || \
    die "${mode_name} 会请求 OFFBOARD 和解锁；确认后设置 CONFIRM_SLS_AUTO_TAKEOFF=I_UNDERSTAND"
}

# Gazebo Classic 的 gzclient 会用 libcurl 访问在线模型库。WSL 里如果继承了
# 已失效的 Windows 本机代理，例如 127.0.0.1:7890，就会反复打印连接失败。
prepare_gazebo_env() {
  export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI:-}"

  if [[ "${KEEP_GAZEBO_PROXY:-false}" == "true" ]]; then
    return
  fi

  local proxy_vars=(http_proxy https_proxy all_proxy HTTP_PROXY HTTPS_PROXY ALL_PROXY)
  local name
  local value

  for name in "${proxy_vars[@]}"; do
    value="${!name:-}"
    if [[ "$value" == *"127.0.0.1:7890"* || "$value" == *"localhost:7890"* ]]; then
      unset "$name"
    fi
  done
}

normalize_mode() {
  case "$MODE" in
    real-point|sls-point-real-go)
      export CONFIRM_SLS_REAL_CONTROL=I_UNDERSTAND
      MODE="sls-point-real"
      ;;
    real-point-auto|sls-point-real-auto-go)
      export CONFIRM_SLS_REAL_CONTROL=I_UNDERSTAND
      export CONFIRM_SLS_AUTO_TAKEOFF=I_UNDERSTAND
      MODE="sls-point-real-auto"
      ;;
    real-swing|sls-swing-real-go)
      export CONFIRM_SLS_REAL_CONTROL=I_UNDERSTAND
      MODE="sls-swing-real"
      ;;
    real-swing-auto|sls-swing-real-auto-go)
      export CONFIRM_SLS_REAL_CONTROL=I_UNDERSTAND
      export CONFIRM_SLS_AUTO_TAKEOFF=I_UNDERSTAND
      MODE="sls-swing-real-auto"
      ;;
    real-wind|sls-swing-wind-real-go)
      export CONFIRM_SLS_REAL_CONTROL=I_UNDERSTAND
      MODE="sls-swing-wind-real"
      ;;
    real-wind-auto|sls-swing-wind-real-auto-go)
      export CONFIRM_SLS_REAL_CONTROL=I_UNDERSTAND
      export CONFIRM_SLS_AUTO_TAKEOFF=I_UNDERSTAND
      MODE="sls-swing-wind-real-auto"
      ;;
    real-hold|sls-takeoff-hold-real-go)
      export CONFIRM_SLS_REAL_CONTROL=I_UNDERSTAND
      MODE="sls-takeoff-hold-real"
      ;;
    real-circle|sls-circle-real-go)
      export CONFIRM_SLS_REAL_CONTROL=I_UNDERSTAND
      MODE="sls-circle-real"
      ;;
    real-circle-auto|sls-circle-real-auto-go)
      export CONFIRM_SLS_REAL_CONTROL=I_UNDERSTAND
      export CONFIRM_SLS_AUTO_TAKEOFF=I_UNDERSTAND
      MODE="sls-circle-real-auto"
      ;;
    rviz-soft)
      export ECHO_RVIZ_SOFTWARE=1
      MODE="rviz"
      ;;
    sensing-nav-soft)
      export ECHO_RVIZ_SOFTWARE=1
      MODE="sensing-nav"
      ;;
  esac
}

normalize_mode

if [[ "${ECHO_RVIZ_SOFTWARE:-0}" == "1" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
fi

case "$MODE" in
  help|-h|--help)
    usage
    ;;

  # 不会下发硬件命令的检查入口。
  check)
    source_env
    echo "Workspace: $WS_DIR"
    echo "ROS_DISTRO: ${ROS_DISTRO:-unknown}"
    echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
    echo "ROS_LOCALHOST_ONLY: ${ROS_LOCALHOST_ONLY}"
    echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
    uname -a
    echo
    echo "Packages:"
    colcon list --base-paths "$WS_DIR" --names-only
    echo
    echo "Serial devices:"
    print_devices
    ;;

  field-precheck)
    source_env
    print_field_precheck
    ;;

  build)
    [[ -f "$ROS_SETUP" ]] || die "ROS setup 不存在: $ROS_SETUP"
    # shellcheck source=/dev/null
    set +u
    source "$ROS_SETUP"
    set -u
    cd "$WS_DIR"
    run colcon build --symlink-install
    ;;

  topics)
    source_env
    stop_ros2_daemon_for_graph_queries
    echo "+ ros2 topic list --no-daemon --spin-time ${ROS2_DISCOVERY_SPIN_TIME} ${*:2}"
    exec ros2 topic list --no-daemon --spin-time "$ROS2_DISCOVERY_SPIN_TIME" "${@:2}"
    ;;

  nodes)
    source_env
    stop_ros2_daemon_for_graph_queries
    echo "+ ros2 node list --no-daemon --spin-time ${ROS2_DISCOVERY_SPIN_TIME} ${*:2}"
    exec ros2 node list --no-daemon --spin-time "$ROS2_DISCOVERY_SPIN_TIME" "${@:2}"
    ;;

  graph)
    source_env
    print_ros_graph_direct
    ;;

  daemon-reset)
    source_env
    echo "+ ros2 daemon stop"
    ros2 daemon stop || true
    ;;

  env)
    echo "source ${ECHO_ENV}"
    ;;

  shell)
    source_env
    echo "echo_drone 环境已加载。退出这个 shell 用 exit。"
    exec "${SHELL:-/bin/bash}" -i
    ;;

  # 硬件层 dry-run 保持 MAVROS adapter 运行，但避免飞控服务调用，适合无桨、
  # 无电机台架验证。
  hardware-dry)
    source_env
    run_launch robot_bring_up hardware.launch.py \
      dry_run:=true \
      use_serial_manager:=true \
      use_servo:=true \
      use_mavros:=true \
      use_legacy_mavlink:=false \
      "${@:2}"
    ;;

  serial-dry)
    source_env
    run_launch robot_bring_up hardware.launch.py \
      dry_run:=true \
      use_serial_manager:=true \
      use_servo:=false \
      use_mavros:=false \
      use_legacy_mavlink:=false \
      "${@:2}"
    ;;

  servo-dry)
    source_env
    run_launch robot_bring_up servo.launch.py dry_run:=true "${@:2}"
    ;;

  adapter-dry|mavros-adapter-dry)
    source_env
    run_launch flight_control mavros_adapter.launch.py "${@:2}"
    ;;

  # 真实硬件入口拆开，先确认飞控心跳，再启用 adapter 或舵机命令链路。
  mavros-state)
    source_env
    print_devices
    fcu_url="$(resolve_fcu_url)"
    echo "FCU URL: ${fcu_url}"
    require_fcu_device "$fcu_url"
    mavros_args=(
      "${WS_DIR}/flight_control/launch/mavros_state.launch.py"
      "fcu_url:=${fcu_url}"
      "tgt_system:=${TARGET_SYSTEM:-1}"
      "tgt_component:=${TARGET_COMPONENT:-1}"
      "fcu_protocol:=${FCU_PROTOCOL:-v2.0}"
      "gcs_url:=${MAVROS_GCS_URL}"
    )
    run_launch "${mavros_args[@]}" "${@:2}"
    ;;

  mavros-check)
    source_env
    print_devices
    tmp_dir="$(mktemp -d /tmp/echo_mavros_check.XXXXXX)"
    check_seconds="${CHECK_SECONDS:-12}"
    fcu_url="$(resolve_fcu_url)"
    echo "FCU URL: ${fcu_url}"
    require_fcu_device "$fcu_url"
    mavros_args=(
      "${WS_DIR}/flight_control/launch/mavros_state.launch.py"
      "fcu_url:=${fcu_url}"
      "tgt_system:=${TARGET_SYSTEM:-1}"
      "tgt_component:=${TARGET_COMPONENT:-1}"
      "fcu_protocol:=${FCU_PROTOCOL:-v2.0}"
      "gcs_url:=${MAVROS_GCS_URL}"
    )
    echo "+ ros2 launch ${mavros_args[*]} ${*:2}"
    ros2 launch "${mavros_args[@]}" "${@:2}" > "${tmp_dir}/mavros.log" 2>&1 &
    check_pid=$!
    sleep "$check_seconds"
    stop_ros2_daemon_for_graph_queries
    ros2_topic_list_no_daemon -t > "${tmp_dir}/topics.out" 2>&1 || true
    sample_topic /mavros/state mavros_msgs/msg/State "${tmp_dir}/state.out"
    sample_topic /mavros/local_position/pose geometry_msgs/msg/PoseStamped "${tmp_dir}/local_pose.out"
    kill "$check_pid" 2>/dev/null || true
    wait "$check_pid" 2>/dev/null || true
    echo "=== MAVROS log key lines ==="
    print_key_log_lines "${tmp_dir}/mavros.log"
    echo "=== MAVROS topics ==="
    match_lines "mavros|uas" "${tmp_dir}/topics.out" || true
    echo "=== /mavros/state sample ==="
    sed -n '1,120p' "${tmp_dir}/state.out"
    echo "=== /mavros/local_position/pose sample ==="
    sed -n '1,80p' "${tmp_dir}/local_pose.out"
    echo "Full check logs: ${tmp_dir}"
    ;;

  mavros-point-check)
    source_env
    tmp_dir="$(mktemp -d /tmp/echo_mavros_point_check.XXXXXX)"
    sample_seconds="${MAVROS_POINT_CHECK_SAMPLE_SECONDS:-2}"
    stop_ros2_daemon_for_graph_queries
    ros2_node_list_no_daemon > "${tmp_dir}/nodes.out" 2>&1 || true
    ros2_topic_list_no_daemon -t > "${tmp_dir}/topics.out" 2>&1 || true
    sample_topic_if_listed /robot/current_pose geometry_msgs/msg/TransformStamped "${tmp_dir}/current_pose.out" "${tmp_dir}/topics.out" "$sample_seconds"
    sample_topic_if_listed /mavros/mavros/pose geometry_msgs/msg/PoseStamped "${tmp_dir}/mavros_vision_pose.out" "${tmp_dir}/topics.out" "$sample_seconds"
    sample_topic_if_listed /mavros/vision_pose/pose geometry_msgs/msg/PoseStamped "${tmp_dir}/legacy_vision_pose.out" "${tmp_dir}/topics.out" "$sample_seconds"
    sample_topic_if_listed /mavros/local_position/pose geometry_msgs/msg/PoseStamped "${tmp_dir}/local_pose.out" "${tmp_dir}/topics.out" "$sample_seconds"
    sample_topic_if_listed /flight_control/status std_msgs/msg/String "${tmp_dir}/adapter_status.out" "${tmp_dir}/topics.out" "$sample_seconds"
    echo "=== nodes ==="
    match_lines "pointlio|laser_mapping|livox|mavros|adapter|serial" "${tmp_dir}/nodes.out" || true
    echo "=== topics ==="
    match_lines "robot/current_pose|mavros|flight_control/status|Odometry|livox" "${tmp_dir}/topics.out" || true
    echo "=== endpoint info ==="
    print_topic_info_best_effort /robot/current_pose
    print_topic_info_best_effort /mavros/mavros/pose
    print_topic_info_best_effort /mavros/vision_pose/pose
    print_topic_info_best_effort /mavros/local_position/pose
    print_topic_info_best_effort /flight_control/status
    echo "=== /robot/current_pose sample ==="
    sed -n '1,80p' "${tmp_dir}/current_pose.out"
    echo "=== /mavros/mavros/pose sample ==="
    sed -n '1,80p' "${tmp_dir}/mavros_vision_pose.out"
    echo "=== /mavros/vision_pose/pose sample ==="
    sed -n '1,80p' "${tmp_dir}/legacy_vision_pose.out"
    echo "=== /mavros/local_position/pose sample ==="
    sed -n '1,80p' "${tmp_dir}/local_pose.out"
    echo "=== /flight_control/status sample ==="
    sed -n '1,120p' "${tmp_dir}/adapter_status.out"
    echo "Full check logs: ${tmp_dir}"
    ;;

  mavros-baud-scan|mavros-scan)
    source_env
    print_devices
    if pgrep -af "/opt/ros/foxy/lib/mavros/mavros_node" >/dev/null; then
      die "已有 MAVROS 进程正在运行；请先 Ctrl-C 停掉 mavros-real 或 mavros-check，再扫描波特率"
    fi
    fcu_device="$(detect_fcu_device)"
    [[ -e "$fcu_device" ]] || die "飞控串口不存在: ${fcu_device}；请先用 usbipd.exe attach 到 WSL"
    tmp_dir="$(mktemp -d /tmp/echo_mavros_baud_scan.XXXXXX)"
    scan_seconds="${BAUD_SCAN_SECONDS:-8}"
    scan_bauds="${FCU_BAUD_SCAN:-230400 57600 115200 921600 460800 1000000}"
    found=0
    echo "FCU device: ${fcu_device}"
    echo "Scan seconds per baud: ${scan_seconds}"
    echo "Logs: ${tmp_dir}"
    for baud in $scan_bauds; do
      fcu_url="${fcu_device}:${baud}"
      log_file="${tmp_dir}/mavros_${baud}.log"
      echo "=== testing ${fcu_url} ==="
      ros2 launch "${WS_DIR}/flight_control/launch/mavros_state.launch.py" \
        fcu_url:="${fcu_url}" \
        tgt_system:="${TARGET_SYSTEM:-1}" \
        tgt_component:="${TARGET_COMPONENT:-1}" \
        fcu_protocol:="${FCU_PROTOCOL:-v2.0}" \
        gcs_url:="${MAVROS_GCS_URL}" \
        > "$log_file" 2>&1 &
      scan_pid=$!
      sleep "$scan_seconds"
      kill "$scan_pid" 2>/dev/null || true
      wait "$scan_pid" 2>/dev/null || true
      if match_lines_quiet "CON: Got HEARTBEAT|connected\\. FCU|FCU: PX4|FCU: ArduPilot" "$log_file"; then
        echo "FOUND: FCU_URL=${fcu_url}"
        print_key_log_lines "$log_file"
        found=1
        break
      fi
      print_key_log_lines "$log_file"
    done
    if [[ "$found" -eq 0 ]]; then
      echo "未找到 heartbeat；请确认飞控端口、TELEM 线序、飞控参数和是否被地面站占用。"
    fi
    ;;

  hardware-real)
    source_env
    print_devices
    fcu_url="$(resolve_fcu_url)"
    echo "FCU URL: ${fcu_url}"
    require_fcu_device "$fcu_url"
    run_launch robot_bring_up hardware.launch.py \
      dry_run:=false \
      use_serial_manager:=true \
      use_servo:=true \
      use_mavros:=true \
      use_legacy_mavlink:=false \
      fcu_url:="${fcu_url}" \
      gcs_url:="${MAVROS_GCS_URL}" \
      "${@:2}"
    ;;

  mavros-real)
    source_env
    print_devices
    fcu_url="$(resolve_fcu_url)"
    echo "FCU URL: ${fcu_url}"
    require_fcu_device "$fcu_url"
    run_launch robot_bring_up hardware.launch.py \
      dry_run:=false \
      use_serial_manager:=true \
      use_servo:=false \
      use_mavros:=true \
      use_legacy_mavlink:=false \
      fcu_url:="${fcu_url}" \
      gcs_url:="${MAVROS_GCS_URL}" \
      "${@:2}"
    ;;

  mavros-vision|mavros-point)
    source_env
    print_devices
    fcu_url="$(resolve_fcu_url)"
    echo "FCU URL: ${fcu_url}"
    require_fcu_device "$fcu_url"
    run_launch robot_bring_up hardware.launch.py \
      dry_run:=false \
      adapter_dry_run:=true \
      use_serial_manager:=true \
      use_servo:=false \
      use_mavros:=true \
      use_legacy_mavlink:=false \
      fcu_url:="${fcu_url}" \
      gcs_url:="${MAVROS_GCS_URL}" \
      "${@:2}"
    ;;

  servo-real)
    source_env
    print_devices
    run_launch robot_bring_up servo.launch.py dry_run:=false "${@:2}"
    ;;

  # 传感器、里程计、感知、决策和整机切片入口。
  livox)
    source_env
    run_launch livox_ros_driver2 msg_MID360s_launch.py "${@:2}"
    ;;

  pointlio)
    source_env
    run_launch point_lio pointlio.launch.py config_path:="$DRONE_PARAMS" rviz:=false "${@:2}"
    ;;

  sensing)
    source_env
    run_launch robot_bring_up sensing.launch.py \
      params_file:="$DRONE_PARAMS" \
      "${@:2}"
    ;;

  sensing-costmap)
    source_env
    run_launch robot_bring_up sensing.launch.py \
      params_file:="$DRONE_PARAMS" \
      launch_costmap:=true \
      launch_rviz:=true \
      is_map:=false \
      "${@:2}"
    ;;

  sensing-nav)
    source_env
    run_launch robot_bring_up sensing_nav.launch.py \
      params_file:="$DRONE_PARAMS" \
      launch_rviz:=true \
      "${@:2}"
    ;;

  costmap)
    source_env
    run_launch robot_bring_up sensing.launch.py \
      params_file:="$DRONE_PARAMS" \
      launch_livox:=false \
      launch_pointlio:=false \
      launch_static_tf:=false \
      launch_rviz:=false \
      launch_costmap:=true \
      costmap_delay:=0.0 \
      "${@:2}"
    ;;

  nav-services)
    source_env
    run_launch robot_bring_up nav_services.launch.py \
      params_file:="$DRONE_PARAMS" \
      "${@:2}"
    ;;

  nav-test)
    source_env
    run "${WS_DIR}/tools/lidar_nav_test.sh" "${@:2}"
    ;;

  sensing-check)
    source_env
    tmp_dir="$(mktemp -d /tmp/echo_sensing_check.XXXXXX)"
    check_seconds="${CHECK_SECONDS:-25}"
    echo "+ ros2 launch robot_bring_up sensing.launch.py params_file:=$DRONE_PARAMS ${*:2}"
    ros2 launch robot_bring_up sensing.launch.py \
      params_file:="$DRONE_PARAMS" \
      "${@:2}" > "${tmp_dir}/sensing.log" 2>&1 &
    check_pid=$!
    sleep "$check_seconds"
    stop_ros2_daemon_for_graph_queries
    ros2_topic_list_no_daemon -t > "${tmp_dir}/topics.out" 2>&1 || true
    sample_topic /livox/lidar livox_ros_driver2/msg/CustomMsg "${tmp_dir}/livox_lidar.out"
    sample_topic /livox/imu sensor_msgs/msg/Imu "${tmp_dir}/livox_imu.out"
    sample_topic /Odometry nav_msgs/msg/Odometry "${tmp_dir}/odometry.out"
    sample_topic /cloud_registered_body sensor_msgs/msg/PointCloud2 "${tmp_dir}/cloud_body.out"
    sample_topic /tf tf2_msgs/msg/TFMessage "${tmp_dir}/tf.out"
    sample_topic /tf_static tf2_msgs/msg/TFMessage "${tmp_dir}/tf_static.out"
    kill "$check_pid" 2>/dev/null || true
    wait "$check_pid" 2>/dev/null || true
    echo "=== Sensing log key lines ==="
    print_key_log_lines "${tmp_dir}/sensing.log"
    echo "=== Sensing topics ==="
    match_lines "livox|Odometry|cloud_registered|tf" "${tmp_dir}/topics.out" || true
    echo "=== /livox/lidar sample ==="
    sed -n '1,80p' "${tmp_dir}/livox_lidar.out"
    echo "=== /livox/imu sample ==="
    sed -n '1,80p' "${tmp_dir}/livox_imu.out"
    echo "=== /Odometry sample ==="
    sed -n '1,100p' "${tmp_dir}/odometry.out"
    echo "=== /cloud_registered_body sample ==="
    sed -n '1,80p' "${tmp_dir}/cloud_body.out"
    echo "Full check logs: ${tmp_dir}"
    ;;

  rviz)
    source_env
    rviz_cfg="${RVIZ_CFG:-${WS_DIR}/install/point_lio/share/point_lio/rviz_cfg/loam_livox.rviz}"
    run rviz2 -d "$rviz_cfg" "${@:2}"
    ;;

  obstacle)
    source_env
    run_launch obstacle_segmentation obstacle_segmentation.launch.py params_file:="$DRONE_PARAMS" "${@:2}"
    ;;

  behavior)
    source_env
    run_launch behavior_control behavior_control.launch.py params_file:="$DRONE_PARAMS" "${@:2}"
    ;;

  mission-dry)
    source_env
    clean_fastdds_shm_if_requested
    run_launch behavior_control behavior_control.launch.py \
      params_file:="$DRONE_PARAMS" \
      dry_run:=true \
      autostart:=false \
      "${@:2}"
    ;;

  mission-graph)
    source_env
    run python3 "${WS_DIR}/tools/mission_graph_viewer.py" "${@:2}"
    ;;

  nav)
    source_env
    run_launch robot_bring_up bringup_launch.py \
      params_file:="$DRONE_PARAMS" \
      use_sim_time:=false \
      use_respawn:=false \
      "${@:2}"
    ;;

  sls-circle-dry)
    source_env
    run_launch sls_circle_controller sls_circle.launch.py \
      dry_run:=true \
      enable_real_setpoint:=false \
      "${@:2}"
    ;;

  sls-circle-sim)
    source_env
    run_launch sls_circle_controller sls_circle_sim.launch.py "${@:2}"
    ;;

  sls-circle-gazebo)
    source_env
    prepare_gazebo_env
    run_launch sls_circle_controller sls_circle_gazebo.launch.py "${@:2}"
    ;;

  sls-px4-sitl)
    source_env
    prepare_gazebo_env
    run_launch sls_circle_controller px4_sitl_sls.launch.py "${@:2}"
    ;;

  sls-circle-mission-dry)
    source_env
    run_launch sls_circle_controller sls_circle.launch.py \
      dry_run:=true \
      enable_real_setpoint:=false \
      mission_mode:=takeoff_then_circle \
      controller_mode:=qsf \
      enable_anti_wind:=true \
      post_takeoff_hold_time:=5.0 \
      "${@:2}"
    ;;

  sls-takeoff-hold-dry)
    source_env
    run_launch sls_circle_controller sls_circle.launch.py \
      dry_run:=true \
      enable_real_setpoint:=false \
      mission_mode:=takeoff_then_hold \
      controller_mode:=qsf \
      enable_anti_wind:=true \
      "${@:2}"
    ;;

  sls-real-check)
    source_env
    tmp_dir="$(mktemp -d /tmp/echo_sls_real_check.XXXXXX)"
    echo "采样目录: ${tmp_dir}"
    stop_ros2_daemon_for_graph_queries
    ros2_topic_list_no_daemon > "${tmp_dir}/topics.out" 2>&1 || true
    sample_topic /mavros/state mavros_msgs/msg/State "${tmp_dir}/mavros_state.out"
    sample_topic /mavros/local_position/pose geometry_msgs/msg/PoseStamped "${tmp_dir}/local_pose.out"
    sample_topic /Odometry nav_msgs/msg/Odometry "${tmp_dir}/pointlio_odometry.out"
    sample_topic /sls_circle/status std_msgs/msg/String "${tmp_dir}/sls_status.out"
    sample_topic /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped "${tmp_dir}/position_setpoint.out"
    sample_topic /mavros/setpoint_raw/attitude mavros_msgs/msg/AttitudeTarget "${tmp_dir}/attitude_setpoint.out"
    echo "=== topics ==="
    match_lines "mavros|sls_circle|Odometry|robot/current_pose|robot/target_pose" "${tmp_dir}/topics.out" || true
    echo "=== /mavros/state ==="
    sed -n '1,80p' "${tmp_dir}/mavros_state.out"
    echo "=== /mavros/local_position/pose ==="
    sed -n '1,80p' "${tmp_dir}/local_pose.out"
    echo "=== /Odometry ==="
    sed -n '1,80p' "${tmp_dir}/pointlio_odometry.out"
    echo "=== /sls_circle/status ==="
    sed -n '1,120p' "${tmp_dir}/sls_status.out"
    echo "=== /mavros/setpoint_position/local ==="
    sed -n '1,80p' "${tmp_dir}/position_setpoint.out"
    echo "=== /mavros/setpoint_raw/attitude ==="
    sed -n '1,80p' "${tmp_dir}/attitude_setpoint.out"
    ;;

  sls-point-real)
    require_sls_real_control "$MODE" "写 /mavros/setpoint_position/local 位置点"
    source_env
    run_launch sls_circle_controller sls_circle.launch.py \
      dry_run:=false \
      enable_real_setpoint:=false \
      mission_mode:=takeoff_then_hold \
      controller_mode:=pd \
      enable_anti_wind:=false \
      wind_compensation_gain:=0.0 \
      use_load_pose:=false \
      require_connected:=true \
      require_offboard:=false \
      require_armed:=false \
      enable_mavros_services:=false \
      "${@:2}"
    ;;

  sls-point-real-auto)
    require_sls_real_control "$MODE" "写 /mavros/setpoint_position/local 位置点"
    require_sls_auto_takeoff "$MODE"
    source_env
    run_launch sls_circle_controller sls_circle.launch.py \
      dry_run:=false \
      enable_real_setpoint:=false \
      mission_mode:=takeoff_then_hold \
      controller_mode:=pd \
      enable_anti_wind:=false \
      wind_compensation_gain:=0.0 \
      use_load_pose:=false \
      require_connected:=true \
      require_offboard:=false \
      require_armed:=false \
      enable_mavros_services:=true \
      auto_offboard:=true \
      auto_arm:=true \
      "${@:2}"
    ;;

  sls-swing-real)
    require_sls_real_control "$MODE" "写 /mavros/setpoint_position/local 和 /mavros/setpoint_raw/attitude"
    source_env
    run_launch sls_circle_controller sls_circle.launch.py \
      dry_run:=false \
      enable_real_setpoint:=true \
      mission_mode:=takeoff_then_circle \
      controller_mode:=qsf \
      enable_anti_wind:=false \
      wind_compensation_gain:=0.0 \
      post_takeoff_hold_time:=5.0 \
      require_connected:=true \
      require_offboard:=true \
      require_armed:=true \
      enable_mavros_services:=false \
      "${@:2}"
    ;;

  sls-swing-real-auto)
    require_sls_real_control "$MODE" "写 /mavros/setpoint_position/local 和 /mavros/setpoint_raw/attitude"
    require_sls_auto_takeoff "$MODE"
    source_env
    run_launch sls_circle_controller sls_circle.launch.py \
      dry_run:=false \
      enable_real_setpoint:=true \
      mission_mode:=takeoff_then_circle \
      controller_mode:=qsf \
      enable_anti_wind:=false \
      wind_compensation_gain:=0.0 \
      post_takeoff_hold_time:=5.0 \
      require_connected:=true \
      require_offboard:=true \
      require_armed:=true \
      enable_mavros_services:=true \
      auto_offboard:=true \
      auto_arm:=true \
      "${@:2}"
    ;;

  sls-swing-wind-real)
    require_sls_real_control "$MODE" "写 /mavros/setpoint_position/local 和 /mavros/setpoint_raw/attitude"
    source_env
    run_launch sls_circle_controller sls_circle.launch.py \
      dry_run:=false \
      enable_real_setpoint:=true \
      mission_mode:=takeoff_then_circle \
      controller_mode:=qsf \
      enable_anti_wind:=true \
      wind_estimator_mode:=residual \
      wind_compensation_gain:=1.0 \
      wind_compensation_warmup_time:=5.0 \
      wind_compensation_ramp_time:=5.0 \
      post_takeoff_hold_time:=5.0 \
      require_connected:=true \
      require_offboard:=true \
      require_armed:=true \
      enable_mavros_services:=false \
      "${@:2}"
    ;;

  sls-swing-wind-real-auto)
    require_sls_real_control "$MODE" "写 /mavros/setpoint_position/local 和 /mavros/setpoint_raw/attitude"
    require_sls_auto_takeoff "$MODE"
    source_env
    run_launch sls_circle_controller sls_circle.launch.py \
      dry_run:=false \
      enable_real_setpoint:=true \
      mission_mode:=takeoff_then_circle \
      controller_mode:=qsf \
      enable_anti_wind:=true \
      wind_estimator_mode:=residual \
      wind_compensation_gain:=1.0 \
      wind_compensation_warmup_time:=5.0 \
      wind_compensation_ramp_time:=5.0 \
      post_takeoff_hold_time:=5.0 \
      require_connected:=true \
      require_offboard:=true \
      require_armed:=true \
      enable_mavros_services:=true \
      auto_offboard:=true \
      auto_arm:=true \
      "${@:2}"
    ;;

  sls-takeoff-hold-real)
    require_sls_real_control "$MODE" "写 /mavros/setpoint_position/local"
    source_env
    run_launch sls_circle_controller sls_circle.launch.py \
      dry_run:=false \
      enable_real_setpoint:=false \
      mission_mode:=takeoff_then_hold \
      controller_mode:=pd \
      enable_anti_wind:=false \
      wind_compensation_gain:=0.0 \
      use_load_pose:=false \
      require_connected:=true \
      require_offboard:=false \
      require_armed:=false \
      enable_mavros_services:=false \
      "${@:2}"
    ;;

  sls-circle-real)
    require_sls_real_control "$MODE" "写 /mavros/setpoint_position/local 和 /mavros/setpoint_raw/attitude"
    source_env
    run_launch sls_circle_controller sls_circle.launch.py \
      dry_run:=false \
      enable_real_setpoint:=true \
      mission_mode:=takeoff_then_circle \
      controller_mode:=qsf \
      enable_anti_wind:=true \
      wind_estimator_mode:=residual \
      wind_compensation_gain:=1.0 \
      wind_compensation_warmup_time:=5.0 \
      wind_compensation_ramp_time:=5.0 \
      post_takeoff_hold_time:=5.0 \
      require_connected:=true \
      require_offboard:=true \
      require_armed:=true \
      enable_mavros_services:=false \
      "${@:2}"
    ;;

  sls-circle-real-auto)
    require_sls_real_control "$MODE" "写 /mavros/setpoint_position/local 和 /mavros/setpoint_raw/attitude"
    require_sls_auto_takeoff "$MODE"
    source_env
    run_launch sls_circle_controller sls_circle.launch.py \
      dry_run:=false \
      enable_real_setpoint:=true \
      mission_mode:=takeoff_then_circle \
      controller_mode:=qsf \
      enable_anti_wind:=true \
      wind_estimator_mode:=residual \
      wind_compensation_gain:=1.0 \
      wind_compensation_warmup_time:=5.0 \
      wind_compensation_ramp_time:=5.0 \
      post_takeoff_hold_time:=5.0 \
      require_connected:=true \
      require_offboard:=true \
      require_armed:=true \
      enable_mavros_services:=true \
      auto_offboard:=true \
      auto_arm:=true \
      "${@:2}"
    ;;

  full)
    source_env
    run_launch robot_bring_up drone.launch.py \
      params_file:="$DRONE_PARAMS" \
      launch_rviz:=false \
      if_map:=false \
      "${@:2}"
    ;;

  *)
    usage
    die "unknown mode: $MODE"
    ;;
esac
