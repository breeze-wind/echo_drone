#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SCENARIO="${1:-pd-hover}"
DURATION="${SLS_VALIDATE_DURATION:-30}"
WARMUP="${SLS_VALIDATE_WARMUP:-8}"
RUN_LOG="${SLS_VALIDATE_LOG:-${WS_DIR}/log/sls_gazebo_${SCENARIO}.log}"

# ROS Foxy 的生成环境脚本会读取若干尚未定义的可选变量。
set +u
source /opt/ros/foxy/setup.bash
source "${WS_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

mkdir -p "$(dirname "$RUN_LOG")"

launch_args=(gui:=false mission_mode:=circle_only control_rate:=100.0 state_rate:=100.0)
validator_args=(
  --duration "$DURATION" --warmup "$WARMUP"
  --max-xy-rms 0.25 --max-xy-max 0.60
  --max-z-rms 0.15 --max-z-max 0.35
  --max-speed 2.0 --min-control-hz 85 --min-pose-hz 120
)
qsf_legacy_gains=(
  qsf_kp_x:=24.0 qsf_kv_x:=50.0 qsf_ka_x:=35.0 qsf_kj_x:=10.0
  qsf_kp_y:=24.0 qsf_kv_y:=50.0 qsf_ka_y:=35.0 qsf_kj_y:=10.0
  qsf_kp_z:=2.0 qsf_kv_z:=3.0
)

case "$SCENARIO" in
  pd-hover)
    launch_args+=(controller_mode:=pd radius:=0.0 angular_velocity:=0.0 enable_wind:=false enable_anti_wind:=false use_load_pose:=false)
    ;;
  pd-circle)
    launch_args+=(controller_mode:=pd radius:=0.3 angular_velocity:=0.12 enable_wind:=false enable_anti_wind:=false use_load_pose:=false)
    ;;
  qsf-virtual)
    launch_args+=(controller_mode:=qsf radius:=0.3 angular_velocity:=0.12 enable_wind:=false enable_anti_wind:=false use_load_pose:=false "${qsf_legacy_gains[@]}")
    validator_args+=(--require-qsf)
    ;;
  qsf-load)
    launch_args+=(controller_mode:=qsf radius:=0.3 angular_velocity:=0.12 enable_wind:=false enable_anti_wind:=false use_load_pose:=true "${qsf_legacy_gains[@]}")
    validator_args+=(--require-qsf)
    ;;
  wind-open-loop)
    launch_args+=(controller_mode:=pd radius:=0.0 angular_velocity:=0.0 use_load_pose:=false enable_wind:=true wind_force_x:=0.30 wind_force_y:=0.15 wind_turbulence:=0.0 enable_anti_wind:=false wind_compensation_gain:=0.0)
    ;;
  wind-truth)
    launch_args+=(controller_mode:=pd radius:=0.0 angular_velocity:=0.0 use_load_pose:=false enable_wind:=true wind_force_x:=0.30 wind_force_y:=0.15 wind_turbulence:=0.0 enable_anti_wind:=true wind_estimator_mode:=actual_feedback wind_compensation_gain:=1.0 wind_compensation_warmup_time:=3.0 wind_compensation_ramp_time:=3.0)
    ;;
  *)
    echo "unknown scenario: $SCENARIO" >&2
    echo "expected: pd-hover, pd-circle, qsf-virtual, qsf-load, wind-open-loop, wind-truth" >&2
    exit 2
    ;;
esac

launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    # launch 独占一个进程组，退出时同时停止 gzserver、桥接器和控制器。
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$launch_pid" 2>/dev/null; then
      kill -TERM -- "-$launch_pid" 2>/dev/null || true
    fi
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if pgrep -f '[g]zserver|[g]azebo_mavros_bridge_node|[s]ls_circle_controller_node_cpp|sls_circle_[g]azebo.launch.py' >/dev/null; then
  echo "another Gazebo/SLS simulation is already running; stop it before validation" >&2
  exit 2
fi

echo "scenario=$SCENARIO duration=${DURATION}s warmup=${WARMUP}s"
echo "log=$RUN_LOG"
setsid "${WS_DIR}/run_echo_drone.sh" sls-circle-gazebo "${launch_args[@]}" >"$RUN_LOG" 2>&1 &
launch_pid=$!

sleep 4
if ! kill -0 "$launch_pid" 2>/dev/null; then
  echo "Gazebo launch exited before validation started" >&2
  tail -80 "$RUN_LOG" >&2
  exit 1
fi

set +e
ros2 run sls_circle_controller gazebo_stability_validator "${validator_args[@]}"
result=$?
set -e
if [[ $result -ne 0 ]]; then
  echo "validation failed; recent Gazebo log:" >&2
  tail -80 "$RUN_LOG" >&2
fi
exit "$result"
