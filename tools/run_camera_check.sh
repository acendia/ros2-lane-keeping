#!/usr/bin/env bash
# Milestone 0 gate: run Gazebo headlessly, bridge the test camera, measure frames.
#
# Usage (inside the Docker environment):
#   tools/run_camera_check.sh [output_dir]
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out_dir="${1:-$repo_dir/artifacts/camera_check}"
world="$repo_dir/tests/gazebo/camera_check.sdf"
gz_topic="/camera_check/image"
ros_topic="/camera_check/image"

mkdir -p "$out_dir"

set +u
source /opt/ros/jazzy/setup.bash
set -u

cleanup() {
  for pid in "${bridge_pid:-}" "${gz_pid:-}"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT

echo "Starting headless Gazebo..."
gz sim -s -r --headless-rendering -v 4 "$world" >"$out_dir/gz.log" 2>&1 &
gz_pid=$!

sleep 8
if ! kill -0 "$gz_pid" 2>/dev/null; then
  echo "Gazebo exited during startup. Last log lines:" >&2
  tail -40 "$out_dir/gz.log" >&2
  exit 1
fi

gz topic -l >"$out_dir/gz_topics.txt"
if ! grep -qx "$gz_topic" "$out_dir/gz_topics.txt"; then
  echo "Camera topic $gz_topic not advertised. Gazebo topics:" >&2
  cat "$out_dir/gz_topics.txt" >&2
  echo "--- gz.log tail ---" >&2
  tail -40 "$out_dir/gz.log" >&2
  exit 1
fi

echo "Bridging $gz_topic to ROS $ros_topic..."
ros2 run ros_gz_bridge parameter_bridge \
  "$gz_topic@sensor_msgs/msg/Image[gz.msgs.Image" \
  >"$out_dir/bridge.log" 2>&1 &
bridge_pid=$!

for _ in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -qx "$ros_topic"; then
    break
  fi
  sleep 1
done
if ! ros2 topic list 2>/dev/null | grep -qx "$ros_topic"; then
  echo "ROS image topic $ros_topic did not appear. Bridge log:" >&2
  cat "$out_dir/bridge.log" >&2
  exit 1
fi

ros2 topic info -v "$ros_topic" >"$out_dir/topic_info.txt" 2>&1 || true

echo "Measuring camera stream..."
set +e
python3 "$repo_dir/tools/camera_probe.py" \
  --topic "$ros_topic" \
  --duration 10 \
  --output "$out_dir/frame.png" \
  | tee "$out_dir/probe.json"
probe_rc=${PIPESTATUS[0]}
set -e

echo "Gazebo log: $out_dir/gz.log"
echo "Frame: $out_dir/frame.png"
exit "$probe_rc"
