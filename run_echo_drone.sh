#!/usr/bin/env bash
set -Eeuo pipefail

# Single operator entrypoint for the refactored stack.  It keeps bench-safe
# dry-run modes, hardware-only checks, and the old full launch under one script
# so the onboard computer can be tested subsystem by subsystem.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${ECHO_DRONE_WS:-$SCRIPT_DIR}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
WS_SETUP="${WS_DIR}/install/setup.bash"
DRONE_PARAMS="${DRONE_PARAMS:-${WS_DIR}/install/robot_bring_up/share/robot_bring_up/config/drone.yaml}"
MODE="${1:-help}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Printed by help mode and kept in this file so commands and documentation do
# not drift apart during the hardware bring-up.
usage() {
  cat <<'EOF'
Usage:
  ./run_echo_drone.sh <mode> [extra ros2 launch args...]

Safe software checks:
  check             Print ROS/workspace/device status.
  build             Build the current workspace.
  topics            Print current ROS topic list.

Dry-run hardware layer:
  hardware-dry      Run serial manager, servo node, and MAVROS adapter in dry-run.
  serial-dry        Run only the serial manager in dry-run.
  servo-dry         Run only the servo node in dry-run.
  adapter-dry       Run only the MAVROS adapter with its dry-run config.

Real hardware layer:
  mavros-state      Run only MAVROS against the FCU, without adapter or commands.
  hardware-real     Run serial manager, servo node, MAVROS, and MAVROS adapter.
  mavros-real       Run serial manager, MAVROS, and MAVROS adapter without servo.
  servo-real        Run only the servo node against /dev/stm32_servo.

Sensor, odometry, perception, and decision:
  livox             Run Livox MID360 driver.
  pointlio          Run Point-LIO only.
  obstacle          Run obstacle segmentation only.
  behavior          Run behavior_control only.
  nav               Run Nav2 bringup only.
  full              Run the current full drone.launch.py with RViz disabled.

Common environment overrides:
  ROS_DOMAIN_ID=0
  ROS_LOCALHOST_ONLY=0
  FCU_URL=/dev/px4_fcu:230400
  DRONE_PARAMS=/path/to/drone.yaml
  ECHO_DRONE_WS=/home/sfx/echo_drone

Examples:
  ./run_echo_drone.sh check
  ./run_echo_drone.sh hardware-dry
  ./run_echo_drone.sh livox
  ./run_echo_drone.sh pointlio rviz:=false
  FCU_URL=/dev/ttyACM0:230400 ./run_echo_drone.sh mavros-real
EOF
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

# Source ROS and the built workspace with nounset temporarily disabled because
# several ROS Foxy setup scripts still read optional unset shell variables.
source_env() {
  [[ -f "$ROS_SETUP" ]] || die "ROS setup not found: $ROS_SETUP"
  # shellcheck source=/dev/null
  set +u
  source "$ROS_SETUP"
  set -u

  [[ -f "$WS_SETUP" ]] || die "Workspace setup not found: $WS_SETUP; run './run_echo_drone.sh build' first"
  # shellcheck source=/dev/null
  set +u
  source "$WS_SETUP"
  set -u
}

# Show stable udev aliases first, then raw USB/ACM devices for WSL/USBIP bench
# sessions where udev may not have created the aliases yet.
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
    echo "No matching serial device found."
  fi
}

# Use exec so Ctrl-C and process exit belong to the launched ROS command.
run() {
  echo "+ $*"
  exec "$@"
}

run_launch() {
  echo "+ ros2 launch $*"
  exec ros2 launch "$@"
}

case "$MODE" in
  help|-h|--help)
    usage
    ;;

  # Non-commanding checks.
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

  build)
    [[ -f "$ROS_SETUP" ]] || die "ROS setup not found: $ROS_SETUP"
    # shellcheck source=/dev/null
    set +u
    source "$ROS_SETUP"
    set -u
    cd "$WS_DIR"
    run colcon build --symlink-install
    ;;

  topics)
    source_env
    run ros2 topic list
    ;;

  # Hardware-layer dry-runs keep MAVROS adapter active but avoid FCU service
  # calls, which makes them safe for no-prop/no-motor bench validation.
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

  # Real hardware-layer modes are split so the FCU heartbeat can be verified
  # before any adapter or servo command path is enabled.
  mavros-state)
    source_env
    print_devices
    mavros_args=(
      "${WS_DIR}/flight_control/launch/mavros_state.launch.py"
      "fcu_url:=${FCU_URL:-/dev/px4_fcu:230400}"
      "tgt_system:=${TARGET_SYSTEM:-1}"
      "tgt_component:=${TARGET_COMPONENT:-1}"
      "fcu_protocol:=${FCU_PROTOCOL:-v2.0}"
    )
    run_launch "${mavros_args[@]}" "${@:2}"
    ;;

  hardware-real)
    source_env
    print_devices
    run_launch robot_bring_up hardware.launch.py \
      dry_run:=false \
      use_serial_manager:=true \
      use_servo:=true \
      use_mavros:=true \
      use_legacy_mavlink:=false \
      fcu_url:="${FCU_URL:-/dev/px4_fcu:230400}" \
      "${@:2}"
    ;;

  mavros-real)
    source_env
    print_devices
    run_launch robot_bring_up hardware.launch.py \
      dry_run:=false \
      use_serial_manager:=true \
      use_servo:=false \
      use_mavros:=true \
      use_legacy_mavlink:=false \
      fcu_url:="${FCU_URL:-/dev/px4_fcu:230400}" \
      "${@:2}"
    ;;

  servo-real)
    source_env
    print_devices
    run_launch robot_bring_up servo.launch.py dry_run:=false "${@:2}"
    ;;

  # Sensor, odometry, perception, decision, and full-stack slices.
  livox)
    source_env
    run_launch livox_ros_driver2 msg_MID360_launch.py "${@:2}"
    ;;

  pointlio)
    source_env
    run_launch point_lio pointlio.launch.py config_path:="$DRONE_PARAMS" rviz:=false "${@:2}"
    ;;

  obstacle)
    source_env
    run_launch obstacle_segmentation obstacle_segmentation.launch.py params_file:="$DRONE_PARAMS" "${@:2}"
    ;;

  behavior)
    source_env
    run_launch behavior_control behavior_control.launch.py params_file:="$DRONE_PARAMS" "${@:2}"
    ;;

  nav)
    source_env
    run_launch robot_bring_up bringup_launch.py \
      params_file:="$DRONE_PARAMS" \
      use_sim_time:=false \
      use_respawn:=false \
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
