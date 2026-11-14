#!/usr/bin/env bash
# Build the development image and run a command inside it.
# The repository is bind-mounted at /workspace.
#
# Usage:
#   ./docker/run.sh                                  # interactive shell
#   ./docker/run.sh colcon build --symlink-install   # one command
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${AUTO_DRIVE_SIM_IMAGE:-auto-drive-sim:jazzy-harmonic}"

docker build -t "$image" -f "$repo_dir/docker/Dockerfile" "$repo_dir"

if [ "$#" -eq 0 ]; then
  set -- bash
fi

docker_args=(--rm --shm-size=1g -p "${FOXGLOVE_PORT:-8765}:8765" \
  -v "$repo_dir:/workspace" -w /workspace)
if [ -t 0 ] && [ -t 1 ]; then
  docker_args+=(-it)
fi

exec docker run "${docker_args[@]}" "$image" "$@"
