#!/usr/bin/env bash
set -eo pipefail

# One-terminal launcher: Gazebo keeps the god-view window, while this shell
# owns the detector, tracker, and target keyboard processes.
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source "$REPO_DIR/install/local_setup.bash"

if pgrep -x px4 >/dev/null || pgrep -x gzserver >/dev/null || pgrep -x MicroXRCEAgent >/dev/null; then
  echo 'A PX4, Gazebo, or MicroXRCEAgent process is already running.' >&2
  echo 'Stop the previous simulation before starting a new one.' >&2
  exit 1
fi

mkdir -p "$REPO_DIR/runtime/dual"
children=()
start_child() {
  setsid "$@" &
  children+=("$!")
}
cleanup() {
  trap - EXIT INT TERM
  for pid in "${children[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${children[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  for pid in "${children[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM

start_child "$REPO_DIR/scripts/start_dual_uav.sh" >"$REPO_DIR/runtime/dual/launcher.log" 2>&1
echo 'Starting Gazebo god view and the two PX4 vehicles...'
sleep 8

start_child "$REPO_DIR/scripts/dual_camera.sh" >"$REPO_DIR/runtime/dual/camera.log" 2>&1
start_child "$REPO_DIR/scripts/red_detector.sh" >"$REPO_DIR/runtime/dual/detector.log" 2>&1
start_child "$REPO_DIR/scripts/follow.sh" >"$REPO_DIR/runtime/dual/follow.log" 2>&1

echo 'God view is open. This terminal now controls the target vehicle.'
echo 'Press T to take off, X to land, and Ctrl-C to land and shut down.'
"$REPO_DIR/scripts/dual_keyboard.sh"
