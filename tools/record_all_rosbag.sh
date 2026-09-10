#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Match run_echo_drone.sh exactly.  Otherwise the recorder can silently join a
# different DDS graph (especially ROS_LOCALHOST_ONLY=0 versus 1).
local_env_file="${ECHO_DRONE_ENV:-${workspace_dir}/.echo_drone.env}"
if [[ -f "${local_env_file}" ]]; then
  set +u
  source "${local_env_file}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS2_DISCOVERY_SPIN_TIME="${ROS2_DISCOVERY_SPIN_TIME:-3.0}"

source /opt/ros/foxy/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

bag_root="${BAG_ROOT:-${workspace_dir}/bags}"
mkdir -p "${bag_root}"
bag_path="${1:-${bag_root}/all_$(date +%Y%m%d_%H%M%S)}"

splitter_executable="${workspace_dir}/install/flight_control/lib/flight_control/mavros_ambiguous_topic_splitter"
if [[ ! -x "${splitter_executable}" ]]; then
  echo "Splitter executable is missing; build flight_control first." >&2
  exit 1
fi
"${splitter_executable}" &
splitter_pid=$!

cleanup() {
  if kill -0 "${splitter_pid}" 2>/dev/null; then
    kill -TERM "${splitter_pid}" 2>/dev/null || true
    wait "${splitter_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Allow DDS discovery to see all four split outputs before freezing the list.
sleep 2

mapfile -t record_topics < <(
  ros2 topic list --no-daemon \
    --spin-time "${ROS2_DISCOVERY_SPIN_TIME}" \
    --include-hidden-topics | while IFS= read -r topic; do
    case "${topic}" in
      /mavros/mavros/in|/mavros/mavros/out) ;;
      *) printf '%s\n' "${topic}" ;;
    esac
  done
)

have_mavros_state=false
have_pose_source=false
for topic in "${record_topics[@]}"; do
  [[ "${topic}" == "/mavros/state" ]] && have_mavros_state=true
  case "${topic}" in
    /Odometry|/robot/current_pose|/mavros/mavros/pose) have_pose_source=true ;;
  esac
done

if [[ "${have_mavros_state}" != true || "${have_pose_source}" != true ]]; then
  echo "Required flight topics were not discovered; refusing to create an empty flight bag." >&2
  echo "mavros_state=${have_mavros_state} pose_source=${have_pose_source} discovered=${#record_topics[@]}" >&2
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" >&2
  exit 1
fi

echo "DDS: ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "Recording ${#record_topics[@]} uniquely typed topics to ${bag_path}"
echo "MAVROS collisions are preserved under /bag_split/mavros/*"
# Do not disable discovery here: Foxy needs the recorder's own DDS graph
# discovery to resolve message types for explicitly listed topics.  The
# frozen whitelist above still prevents the original ambiguous MAVROS in/out
# topics from being passed to rosbag2; the splitter outputs remain recordable.
ros2 bag record --include-hidden-topics \
  -o "${bag_path}" "${record_topics[@]}"
