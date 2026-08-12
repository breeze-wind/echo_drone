#!/usr/bin/env bash
# Shared ROS 2 environment for echo_drone interactive debugging.
#
# Use from a terminal:
#   source tools/echo_drone_env.bash

_echo_drone_env_sourced=0
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  _echo_drone_env_sourced=1
fi

_echo_drone_env_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ECHO_DRONE_WS="${ECHO_DRONE_WS:-$(cd "${_echo_drone_env_script_dir}/.." && pwd)}"

_echo_drone_env_local_file="${ECHO_DRONE_ENV:-${ECHO_DRONE_WS}/.echo_drone.env}"
if [[ -f "$_echo_drone_env_local_file" ]]; then
  # shellcheck source=/dev/null
  source "$_echo_drone_env_local_file"
fi

export ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
export WS_SETUP="${WS_SETUP:-${ECHO_DRONE_WS}/install/setup.bash}"
export DRONE_PARAMS="${DRONE_PARAMS:-${ECHO_DRONE_WS}/install/robot_bring_up/share/robot_bring_up/config/drone.yaml}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS2_DISCOVERY_SPIN_TIME="${ROS2_DISCOVERY_SPIN_TIME:-3.0}"

_echo_drone_env_had_nounset=0
case "$-" in
  *u*)
    _echo_drone_env_had_nounset=1
    set +u
    ;;
esac

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS setup 不存在: $ROS_SETUP" >&2
  [[ "$_echo_drone_env_had_nounset" -eq 1 ]] && set -u
  return 1 2>/dev/null || exit 1
fi

# shellcheck source=/dev/null
source "$ROS_SETUP"

if [[ ! -f "$WS_SETUP" ]]; then
  echo "ERROR: 工作区 setup 不存在: $WS_SETUP; 请先运行 './run_echo_drone.sh build'" >&2
  [[ "$_echo_drone_env_had_nounset" -eq 1 ]] && set -u
  return 1 2>/dev/null || exit 1
fi

# shellcheck source=/dev/null
source "$WS_SETUP"

[[ "$_echo_drone_env_had_nounset" -eq 1 ]] && set -u

if [[ "$_echo_drone_env_sourced" -eq 1 \
    && "${ECHO_DRONE_ENV_QUIET:-0}" != "1" \
    && "${ECHO_DRONE_WRAP_ROS2_LIST:-1}" == "1" ]]; then
  _echo_drone_ros2_has_arg() {
    local needle="$1"
    shift
    local arg
    for arg in "$@"; do
      [[ "$arg" == "$needle" ]] && return 0
    done
    return 1
  }

  _echo_drone_ros2_direct_list() {
    local group="$1"
    shift

    if _echo_drone_ros2_has_arg "-h" "$@" || _echo_drone_ros2_has_arg "--help" "$@"; then
      command ros2 "$group" list "$@"
      return
    fi

    local direct_args=()
    if ! _echo_drone_ros2_has_arg "--no-daemon" "$@"; then
      direct_args+=(--no-daemon)
    fi
    if ! _echo_drone_ros2_has_arg "--spin-time" "$@"; then
      direct_args+=(--spin-time "${ROS2_DISCOVERY_SPIN_TIME:-3.0}")
    fi

    command ros2 "$group" list "${direct_args[@]}" "$@"
  }

  ros2() {
    if [[ "$#" -ge 2 && "$2" == "list" ]]; then
      case "$1" in
        topic|node|service)
          _echo_drone_ros2_direct_list "$1" "${@:3}"
          return
          ;;
      esac
    fi

    command ros2 "$@"
  }
fi

if [[ "${ECHO_DRONE_ENV_QUIET:-0}" != "1" ]]; then
  echo "echo_drone env ready:"
  echo "  ECHO_DRONE_WS=${ECHO_DRONE_WS}"
  echo "  LOCAL_ENV=${_echo_drone_env_local_file}"
  echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  echo "  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
  echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
  echo "  ROS2_DISCOVERY_SPIN_TIME=${ROS2_DISCOVERY_SPIN_TIME}"
  if [[ "${ECHO_DRONE_WRAP_ROS2_LIST:-1}" == "1" ]]; then
    echo "  ros2 topic/node/service list use direct discovery in this shell"
  fi
fi

if [[ "$_echo_drone_env_sourced" -eq 0 ]]; then
  echo
  echo "注意：直接执行脚本不会把环境保留到当前终端。请用："
  echo "  source ${ECHO_DRONE_WS}/tools/echo_drone_env.bash"
fi

unset _echo_drone_env_sourced
unset _echo_drone_env_script_dir
unset _echo_drone_env_local_file
unset _echo_drone_env_had_nounset
