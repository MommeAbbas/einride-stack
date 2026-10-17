#!/usr/bin/env bash
# Fixed verification only: one fixed configuration, no tuning/ablations.
set -e
sim_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
workspace=$(cd -- "$sim_repo/.." && pwd)
source /opt/ros/jazzy/setup.bash
export PATH=/usr/bin:/bin:$PATH
source "$workspace/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=${AUTONOMY_SIM_DOMAIN_ID:-88}
output=${1:-$sim_repo/data/encounters_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$output"
output=$(cd -- "$output" && pwd)
snapshot="$sim_repo/data/accepted_slam/slam_snapshot.json"
[[ -f "$snapshot" ]] || { echo 'Accepted E0 snapshot missing' >&2; exit 1; }
run_case() {
  local name=$1 stage=$2 count=$3 duration=$4 scenario=$5
  shift 5
  printf 'Starting %s\n' "$name"
  if ros2 run sim sim_experiment_node --workspace "$workspace" \
      --scenario "$sim_repo/sim/config/$scenario" --output-dir "$output/$name" \
      --ros-args -p stack_stage:="$stage" -p traffic_count:="$count" \
      -p duration:="$duration" -p run_wall_timeout:=3600.0 -p headless:=true \
      -p foxglove:=false -p map_source:=snapshot -p map_snapshot:="$snapshot" "$@"; then
    if [[ "$stage" == E4 ]]; then
      python3 "$sim_repo/sim/test/encounter_check.py" "$output/$name/run_000" > "$output/$name/check.log" 2>&1 || true
    fi
  else
    printf '%s runner failed; retain evidence and continue independent cases\n' "$name" >&2
  fi
}
run_case E4_1_ignore E4 1 300.0 e4.yaml
run_case E4_3_ignore E4 3 300.0 e4.yaml
run_case E4_3_yield E4 3 300.0 e4_yield.yaml
run_case E3 E3 0 360.0 e4.yaml
run_case E3_LOOK E3 0 120.0 e3_look.yaml -p planning_risk_tolerance:=0.08
python3 "$sim_repo/sim/test/stage_check.py" E3 "$output/E3/run_000" \
  --look-evidence "$output/E3_LOOK/run_000" > "$output/E3/check.log" 2>&1 || true
printf 'Verification recordings finished; inspect check JSON/logs for PASS/FAIL: %s\n' "$output"
