#!/usr/bin/env bash
# Viewer-only replay. No control, Saga, or simulator processes are launched.
set -e
sim_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=$(cd -- "$sim_repo/.." && pwd)
if [[ $# -ne 1 || ! -f "$1/metadata.yaml" ]]; then
  printf 'Usage: %s /absolute/path/to/run_000/bag\n' "$0" >&2
  exit 1
fi
bag=$(cd -- "$1" && pwd)
source /opt/ros/jazzy/setup.bash
export PATH=/usr/bin:/bin:$PATH
source "$workspace/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=${AUTONOMY_REPLAY_DOMAIN_ID:-89} # Isolate viewer-only replay from live control.
# Refuse a live viewer endpoint rather than interfere with an existing run.
python3 - <<'PY'
import socket
with socket.socket() as sock:sock.bind(('127.0.0.1',8765))
PY
ros2 run foxglove_bridge foxglove_bridge --ros-args -p use_sim_time:=true -p address:=127.0.0.1 -p port:=8765 &
bridge_pid=$!
trap 'kill -INT "$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true' EXIT
printf 'Connect Foxglove to ws://localhost:8765. Space pauses/resumes in this terminal.\n'
# --clock supplies replay time; exclude recorded /clock to avoid two time sources.
ros2 bag play "$bag" --clock --start-paused --exclude-topics /clock /cmd_vel /traffic_1/cmd_vel /traffic_2/cmd_vel /traffic_3/cmd_vel --qos-profile-overrides-path "$sim_repo/sim/config/replay_qos.yaml"
