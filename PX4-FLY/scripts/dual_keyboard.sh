#!/usr/bin/env bash
set -eo pipefail
export PYTHONNOUSERSITE=1
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$REPO_DIR/install/local_setup.bash"
echo 'This terminal is the target-drone operation interface.'
echo 'T takeoff | W/A/S/D move | R/F altitude | J/L yaw | X land | H help'
exec ros2 run px4_fly_control keyboard_control --ros-args -p takeoff_altitude:=3.0 "$@"
