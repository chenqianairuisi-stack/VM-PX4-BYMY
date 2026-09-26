#!/usr/bin/env bash
set -eo pipefail
export PYTHONNOUSERSITE=1
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$REPO_DIR/install/local_setup.bash"
exec ros2 run px4_fly_control keyboard_control
