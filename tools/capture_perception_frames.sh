#!/usr/bin/env bash
# Capture the saved frames used by the lane perception tests.
#
# Usage (inside the Docker environment):
#   tools/capture_perception_frames.sh
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out_dir="${1:-$repo_dir/tests/frames}"
world="${2:-$repo_dir/tests/gazebo/perception_frames.sdf}"
log_dir="$repo_dir/artifacts/$(basename "$out_dir")"
models_dir="$repo_dir/src/auto_drive_sim/models"

read -r -a cameras <<< "${CAMERAS:-straight right_curve left_curve no_line}"

mkdir -p "$out_dir" "$log_dir"

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

gz sim -s -r --headless-rendering -v 4 "$world" >"$log_dir/gz.log" 2>&1 &
gz_pid=$!

sleep 10
if ! kill -0 "$gz_pid" 2>/dev/null; then
  echo "Gazebo exited during startup. Last log lines:" >&2
  tail -50 "$log_dir/gz.log" >&2
  exit 1
fi

gz topic -l >"$log_dir/gz_topics.txt"

topic_prefix="/$(basename "$world" .sdf)"

bridge_args=()
for camera in "${cameras[@]}"; do
  topic="$topic_prefix/$camera/image"
  if ! grep -qx "$topic" "$log_dir/gz_topics.txt"; then
    echo "Camera topic $topic not advertised." >&2
    exit 1
  fi
  bridge_args+=("$topic@sensor_msgs/msg/Image[gz.msgs.Image")
done

ros2 run ros_gz_bridge parameter_bridge "${bridge_args[@]}" \
  >"$log_dir/bridge.log" 2>&1 &
bridge_pid=$!

sleep 3
for camera in "${cameras[@]}"; do
  if ! python3 "$repo_dir/tools/camera_probe.py" \
      --topic "$topic_prefix/$camera/image" \
      --duration 4 \
      --min-frames 5 \
      --output "$out_dir/$camera.png" \
      >"$log_dir/$camera.json"; then
    if [ "$camera" != "no_line" ]; then
      echo "probe failed for $camera" >&2
      exit 1
    fi
  fi
  echo "captured $camera"
done

echo "Frames: $out_dir"
