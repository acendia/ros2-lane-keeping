#!/usr/bin/env bash
# Milestone 1 gate: capture the oval from four camera poses.
#
# Usage (inside the Docker environment):
#   tools/run_road_camera_check.sh [output_dir]
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out_dir="${1:-$repo_dir/artifacts/road_check}"
world="$repo_dir/tests/gazebo/road_camera_check.sdf"
models_dir="$repo_dir/src/auto_drive_sim/models"

mkdir -p "$out_dir"

set +u
source /opt/ros/jazzy/setup.bash
set -u

export GZ_SIM_RESOURCE_PATH="$models_dir${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

cleanup() {
  for pid in "${bridge_pid:-}" "${gz_pid:-}"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT

echo "Starting headless Gazebo with the oval road..."
gz sim -s -r --headless-rendering -v 4 "$world" >"$out_dir/gz.log" 2>&1 &
gz_pid=$!

sleep 10
if ! kill -0 "$gz_pid" 2>/dev/null; then
  echo "Gazebo exited during startup. Last log lines:" >&2
  tail -50 "$out_dir/gz.log" >&2
  exit 1
fi

gz topic -l >"$out_dir/gz_topics.txt"

cameras=(straight_start east_entry east_apex west_apex)
bridge_args=()
for camera in "${cameras[@]}"; do
  gz_topic="/road_check/$camera/image"
  if ! grep -qx "$gz_topic" "$out_dir/gz_topics.txt"; then
    echo "Camera topic $gz_topic not advertised. Gazebo topics:" >&2
    cat "$out_dir/gz_topics.txt" >&2
    echo "--- gz.log tail ---" >&2
    tail -50 "$out_dir/gz.log" >&2
    exit 1
  fi
  bridge_args+=("$gz_topic@sensor_msgs/msg/Image[gz.msgs.Image")
done

if grep -q "Unable to find\|Could not find\|Failed to load" "$out_dir/gz.log"; then
  echo "Gazebo reported a missing or failed asset. Log excerpt:" >&2
  grep -n "Unable to find\|Could not find\|Failed to load" "$out_dir/gz.log" >&2
  exit 1
fi

echo "Bridging ${#cameras[@]} camera topics..."
ros2 run ros_gz_bridge parameter_bridge "${bridge_args[@]}" \
  >"$out_dir/bridge.log" 2>&1 &
bridge_pid=$!

for camera in "${cameras[@]}"; do
  ros_topic="/road_check/$camera/image"
  for _ in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -qx "$ros_topic"; then
      break
    fi
    sleep 1
  done
  if ! ros2 topic list 2>/dev/null | grep -qx "$ros_topic"; then
    echo "ROS topic $ros_topic did not appear. Bridge log:" >&2
    cat "$out_dir/bridge.log" >&2
    exit 1
  fi
done

failed=0
for camera in "${cameras[@]}"; do
  echo "Capturing $camera..."
  if ! python3 "$repo_dir/tools/camera_probe.py" \
      --topic "/road_check/$camera/image" \
      --duration 6 \
      --min-frames 5 \
      --output "$out_dir/$camera.png" \
      | tee "$out_dir/$camera.json"; then
    failed=1
  fi
done

echo "Gazebo log: $out_dir/gz.log"
echo "Frames: $out_dir/*.png"
exit "$failed"
