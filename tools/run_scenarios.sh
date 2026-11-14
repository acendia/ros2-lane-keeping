#!/usr/bin/env bash
# Run the centered, left-offset, and right-offset evaluation scenarios.
#
# Usage (inside the Docker environment):
#   tools/run_scenarios.sh [center] [left] [right]
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "$repo_dir/install/setup.bash"
set -u

scenarios=("$@")
if [ "${#scenarios[@]}" -eq 0 ]; then
  scenarios=(center left right)
fi

target_speed="${TARGET_SPEED:-1.5}"
lookahead_gain="${LOOKAHEAD_GAIN:-1.2}"
yaw_alpha="${YAW_ALPHA:-0.35}"

for name in "${scenarios[@]}"; do
  case "$name" in
    center) offset=0.0 ;;
    left) offset=0.3 ;;
    right) offset=-0.3 ;;
    *) echo "unknown scenario: $name" >&2; exit 2 ;;
  esac

  run_dir="$repo_dir/artifacts/scenarios/$name"
  result_dir="$repo_dir/results/$name"
  mkdir -p "$run_dir" "$result_dir"
  python3 "$repo_dir/tools/make_scenario_world.py" \
    --offset "$offset" --output "$run_dir/world.sdf" >/dev/null
  start_y="$(python3 -c "print(15.0 + ($offset))")"

  echo "=== scenario: $name (offset ${offset} m) ==="

  timeout --signal=INT --kill-after=5s 420s \
    ros2 launch auto_drive_sim demo.launch.py gui:=false controller:=false \
    world:="$run_dir/world.sdf" >"$run_dir/launch.log" 2>&1 &
  launch_pid=$!

  ros2 run ros_gz_bridge bridge_node --ros-args \
    -p config_file:="$repo_dir/src/auto_drive_sim/config/eval_bridge.yaml" \
    >"$run_dir/eval_bridge.log" 2>&1 &
  bridge_pid=$!

  sleep 20

  timeout 420s ros2 run auto_drive_sim evaluator --ros-args \
    -p use_sim_time:=true \
    -p scenario:="$name" \
    -p start_x:=-10.0 \
    -p start_y:="$start_y" \
    -p start_yaw:=0.0 \
    -p output_dir:="$result_dir" \
    -p timeout_s:=130.0 \
    >"$run_dir/evaluator.log" 2>&1 &
  eval_pid=$!

  sleep 2

  ros2 run auto_drive_sim lane_controller --ros-args \
    -p use_sim_time:=true \
    -p target_speed_mps:="$target_speed" \
    -p lookahead_gain:="$lookahead_gain" \
    -p yaw_smoothing_alpha:="$yaw_alpha" \
    >"$run_dir/controller.log" 2>&1 &
  ctrl_pid=$!

  set +e
  wait "$eval_pid"
  eval_rc=$?
  set -e
  echo "evaluator exit: $eval_rc"

  kill -INT "$ctrl_pid" 2>/dev/null || true
  kill -INT "$bridge_pid" 2>/dev/null || true
  kill -INT "$launch_pid" 2>/dev/null || true
  sleep 3
  pkill -f "lane_controller" 2>/dev/null || true
  pkill -f "lane_perception" 2>/dev/null || true
  pkill -f "gz sim -s -r --headless-rendering" 2>/dev/null || true
  pkill -f "bridge_node" 2>/dev/null || true
  sleep 2

  if [ ! -f "$result_dir/result.json" ]; then
    echo "missing result.json for $name" >&2
    exit 1
  fi
  echo "result: $result_dir/result.json"
done

python3 "$repo_dir/tools/summarize_results.py"
