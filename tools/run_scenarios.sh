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
world_name="${WORLD:-car_world.sdf}"
results_base="${RESULTS_DIR:-$repo_dir/artifacts/evaluations}"
mkdir -p "$results_base"
results_root="$(mktemp -d "$results_base/run.XXXXXXXX")"
export results_root repo_dir target_speed lookahead_gain yaw_alpha world_name
python3 - <<'META'
import hashlib, json, os, platform, subprocess
from pathlib import Path
repo = Path(os.environ['repo_dir'])
def git(*args):
    return subprocess.check_output(['git', '-C', str(repo), *args], text=True).strip()
files = [p for folder in ('src', 'tools', 'docker', 'tests') for p in (repo/folder).rglob('*')
         if p.is_file() and '__pycache__' not in p.parts]
hash_value = hashlib.sha256()
for path in sorted(files):
    hash_value.update(str(path.relative_to(repo)).encode())
    hash_value.update(path.read_bytes())
metadata = dict(commit=git('rev-parse', 'HEAD'), dirty=bool(git('status', '--porcelain')),
                source_sha256=hash_value.hexdigest(), architecture=platform.machine(),
                python=platform.python_version(),
                packages=subprocess.check_output(['dpkg-query', '-W'], text=True),
                parameters={k: os.environ[k] for k in
                    ('target_speed', 'lookahead_gain', 'yaw_alpha', 'world_name')})
Path(os.environ['results_root'], 'run_metadata.json').write_text(json.dumps(metadata, indent=2)+'\n')
META
cleanup() {
  for pid in "${ctrl_pid:-}" "${bridge_pid:-}" "${launch_pid:-}" "${eval_pid:-}"; do
    if [ -n "$pid" ]; then kill -TERM -- "-$pid" 2>/dev/null || true; fi
  done
  sleep 1
  for pid in "${ctrl_pid:-}" "${bridge_pid:-}" "${launch_pid:-}" "${eval_pid:-}"; do
    if [ -n "$pid" ]; then kill -KILL -- "-$pid" 2>/dev/null || true; fi
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
source_world="$repo_dir/install/auto_drive_sim/share/auto_drive_sim/worlds/$world_name"
if [ ! -f "$source_world" ]; then
  source_world="$repo_dir/src/auto_drive_sim/worlds/$world_name"
fi
if [ ! -f "$source_world" ]; then
  echo "world not found: $world_name" >&2
  exit 2
fi
environment="${SUMMARY_ENVIRONMENT:-Docker, Ubuntu 24.04.5 arm64, ROS 2 Jazzy, Gazebo Sim 8.15.0.}"
run_root="${RUN_DIR:-$results_root/logs}"
lane_strategy="${LANE_STRATEGY:-single}"
start_center_y="${START_Y:-15.0}"
lane_center_y="${LANE_CENTER_Y:-15.0}"
centerline_path="${CENTERLINE_PATH:-}"
track_config_path="${TRACK_CONFIG_PATH:-}"

for name in "${scenarios[@]}"; do
  case "$name" in
    center) offset=0.0 ;;
    left) offset=0.3 ;;
    right) offset=-0.3 ;;
    *) echo "unknown scenario: $name" >&2; exit 2 ;;
  esac

  run_dir="$run_root/$name"
  result_dir="$results_root/$name"
  mkdir -p "$run_dir" "$result_dir"
  python3 "$repo_dir/tools/make_scenario_world.py" \
    --offset "$offset" --output "$run_dir/world.sdf" \
    --source-world "$source_world" --start-y "$start_center_y" >/dev/null
  start_y="$(python3 -c "print($start_center_y + ($offset))")"

  echo "=== scenario: $name (offset ${offset} m) ==="

  setsid timeout --signal=INT --kill-after=5s 420s \
    ros2 launch auto_drive_sim demo.launch.py gui:=false controller:=false \
    world:="$run_dir/world.sdf" lane_strategy:="$lane_strategy" \
    >"$run_dir/launch.log" 2>&1 &
  launch_pid=$!

  setsid ros2 run ros_gz_bridge bridge_node --ros-args \
    -p config_file:="$repo_dir/src/auto_drive_sim/config/eval_bridge.yaml" \
    >"$run_dir/eval_bridge.log" 2>&1 &
  bridge_pid=$!

  sleep 20

  evaluator_args=(
    -p use_sim_time:=true
    -p scenario:="$name"
    -p start_x:=-10.0
    -p start_y:="$start_y"
    -p start_yaw:=0.0
    -p lane_center_y:="$lane_center_y"
    -p output_dir:="$result_dir"
    -p timeout_s:=130.0
  )
  if [ -n "$centerline_path" ]; then
    evaluator_args+=(-p centerline_path:="$centerline_path")
  fi
  if [ -n "$track_config_path" ]; then
    evaluator_args+=(-p track_config_path:="$track_config_path")
  fi

  setsid timeout 420s ros2 run auto_drive_sim evaluator --ros-args \
    "${evaluator_args[@]}" \
    >"$run_dir/evaluator.log" 2>&1 &
  eval_pid=$!

  sleep 2

  setsid ros2 run auto_drive_sim lane_controller --ros-args \
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

  cleanup
  ctrl_pid= bridge_pid= launch_pid= eval_pid=
  if [ "$eval_rc" -ne 0 ]; then
    echo "evaluation failed: $name (exit $eval_rc); logs: $run_dir" >&2
    exit 1
  fi

  if [ ! -f "$result_dir/result.json" ]; then
    echo "missing result.json for $name" >&2
    exit 1
  fi
  echo "result: $result_dir/result.json"
done

python3 "$repo_dir/tools/summarize_results.py" \
  --results-dir "$results_root" --environment "$environment"

echo "Fresh evaluation results: $results_root"
