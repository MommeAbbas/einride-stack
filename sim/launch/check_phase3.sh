#!/usr/bin/env bash
# Sequential E0–E4 acceptance only. No ablations or Phase 4.
set -e
sim_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=$(cd -- "$sim_repo/.." && pwd)
source /opt/ros/jazzy/setup.bash
export PATH=/usr/bin:/bin:$PATH
source "$workspace/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
output=${1:-$sim_repo/data/phase3_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$output"
output=$(cd -- "$output" && pwd)
run_stage() {
  local stage=$1 name=$2 count=$3 duration=$4 map_source=$5
  shift 5
  ros2 run sim sim_experiment_node --workspace "$workspace" \
    --scenario "$sim_repo/sim/config/e4.yaml" --output-dir "$output/$name" \
    --ros-args -p stack_stage:="$stage" -p traffic_count:="$count" -p duration:="$duration" \
    -p map_source:="$map_source" -p map_snapshot:="$output/E0/run_000/slam_snapshot.json" "$@"
  if [[ "$stage" != E3 ]]; then
    python3 "$sim_repo/sim/test/stage_check.py" "$stage" "$output/$name/run_000"
  fi
}
run_stage E0 E0 0 450.0 sensors -p stop_when_map_ready:=true
mkdir -p "$sim_repo/data/accepted_slam"
cp "$output/E0/run_000/"{slam_snapshot.json,sim_handover.json,map.yaml,map.pgm} "$sim_repo/data/accepted_slam/"
run_stage E1 E1 0 90.0 snapshot
run_stage E2 E2 0 240.0 snapshot -p event_start_delay:=8.0
# [s sim] Allow a complete source/destination route plus observation manoeuvres.
run_stage E3 E3 0 360.0 snapshot
# A safe empty-scene baseline need not LOOK. Exercise that branch separately at
# a stricter threshold, keeping the baseline mission/progress check unchanged.
run_stage E3 E3_LOOK 0 120.0 snapshot -p planning_risk_tolerance:=0.08
python3 "$sim_repo/sim/test/stage_check.py" E3 "$output/E3/run_000" --look-evidence "$output/E3_LOOK/run_000"
run_stage E4 E4_1 1 300.0 snapshot
run_stage E4 E4_3 3 300.0 snapshot
printf 'Phase 3 stage checks passed: %s\n' "$output"
