#!/usr/bin/env bash
# Start the Livox MID360 driver directly inside a VMware Ubuntu guest.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${ECHO_DRONE_WS:-$(cd "${SCRIPT_DIR}/.." && pwd)}"

# The shared environment also reads the ignored .echo_drone.env file, where
# LIVOX_HOST_IP and LIVOX_LIDAR_IP belong.
# shellcheck source=/dev/null
source "${WS_DIR}/tools/echo_drone_env.bash"

RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/echo_drone_livox"
CONFIG_PATH="${LIVOX_VMWARE_CONFIG:-${RUNTIME_DIR}/MID360s_vmware_config.json}"

python3 "${WS_DIR}/tools/generate_livox_vmware_config.py" --output "${CONFIG_PATH}"

echo "+ ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -p user_config_path:=${CONFIG_PATH}"
exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=1 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p user_config_path:="${CONFIG_PATH}" \
  -p cmdline_input_bd_code:=livox0000000001 \
  "$@"
