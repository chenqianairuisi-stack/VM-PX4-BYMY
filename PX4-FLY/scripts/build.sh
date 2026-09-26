#!/usr/bin/env bash
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
cd "$REPO_DIR"
export MAKEFLAGS=-j2
if [[ ! -f external/px4_msgs/package.xml ]]; then
  git submodule update --init --recursive
fi
colcon build --symlink-install --base-paths external/px4_msgs px4_fly_interfaces px4_fly_control px4_fly_sim --parallel-workers 1
