#!/usr/bin/env bash
# Run a command in the experimental, native Apple Silicon ROS environment.
set -eo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
native_prefix="${NATIVE_ROS_PREFIX:-/opt/homebrew/opt/ros2-jazzy}"
framework_python="${NATIVE_FRAMEWORK_PYTHON:-/Library/Frameworks/Python.framework/Versions/3.11/bin/python3}"
if [ ! -x "$framework_python" ]; then
  echo "Missing framework Python: $framework_python. Install python.org Python 3.11; see docs/native_macos.md" >&2
  exit 2
fi
if [ ! -f "$native_prefix/setup.bash" ]; then
  echo "Missing native ROS bundle: $native_prefix. See docs/native_macos.md" >&2
  exit 2
fi
export PATH="$(dirname "$framework_python"):$HOME/Library/Python/3.11/bin:/opt/homebrew/bin:$PATH"
source "$native_prefix/setup.bash"
case "${LIBGL_ALWAYS_SOFTWARE:-}" in
  1|true|TRUE) echo 'Software rendering requested; native validation requires hardware Metal.' >&2; exit 2 ;;
esac
unset LIBGL_ALWAYS_SOFTWARE
export NATIVE_ROS_PREFIX="$native_prefix"
# Keep this single-Mac experiment on loopback, away from Wi-Fi/VPN discovery.
export GZ_IP=127.0.0.1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo0" multicast="true"/></Interfaces></General><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'
cd "$repo_dir"
if [ -f "$repo_dir/install-native/setup.bash" ]; then
  source "$repo_dir/install-native/local_setup.bash"
fi
exec "$@"
