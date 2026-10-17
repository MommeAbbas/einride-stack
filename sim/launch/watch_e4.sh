#!/usr/bin/env bash
# Launch Gazebo GUI + all production nodes + 3 traffic robots + Foxglove bridge.
set -e
sim_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=$(cd -- "$sim_repo/.." && pwd)
source /opt/ros/jazzy/setup.bash
export PATH=/usr/bin:/bin:$PATH
source "$workspace/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
snapshot=${1:-$sim_repo/data/accepted_slam/slam_snapshot.json}
if [[ ! -f "$snapshot" ]]; then
  printf 'Sensor-built map snapshot missing: %s\nRun the documented E0 check first.\n' "$snapshot" >&2
  exit 1
fi
output="$workspace/runs/watch_$(date +%Y%m%d_%H%M%S)"
exec ros2 run sim sim_experiment_node --workspace "$workspace" \
  --scenario "$sim_repo/sim/config/e4.yaml" --output-dir "$output" \
  --ros-args -p headless:=false -p foxglove:=true -p map_snapshot:="$snapshot"
