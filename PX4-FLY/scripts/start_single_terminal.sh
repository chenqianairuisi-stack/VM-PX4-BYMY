#!/usr/bin/env bash
set -eo pipefail

# One-terminal launcher: Gazebo keeps the god-view window, while this shell
# owns the detector, tracker, and target keyboard processes.
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
export FOLLOW_MODE="${FOLLOW_MODE:-drone}"
if [[ "$FOLLOW_MODE" == drone || "$FOLLOW_MODE" == virtual_contact ]]; then
  "$REPO_DIR/scripts/yolo_detector.sh" --check
fi
source /opt/ros/humble/setup.bash
source "$REPO_DIR/install/local_setup.bash"

if pgrep -x px4 >/dev/null || pgrep -x gzserver >/dev/null || pgrep -x MicroXRCEAgent >/dev/null; then
  echo 'A PX4, Gazebo, or MicroXRCEAgent process is already running.' >&2
  echo 'Stop the previous simulation before starting a new one.' >&2
  exit 1
fi

mkdir -p "$REPO_DIR/runtime/dual"
children=()
child_logs=()
start_child() {
  local log_file="$1"
  shift
  setsid bash "$@" >"$log_file" 2>&1 &
  children+=("$!")
  child_logs+=("$log_file")
}
check_children() {
  local i
  for i in "${!children[@]}"; do
    if ! kill -0 "${children[$i]}" 2>/dev/null; then
      echo "启动失败，请查看日志：${child_logs[$i]}" >&2
      tail -30 "${child_logs[$i]}" >&2
      exit 1
    fi
  done
}
cleanup() {
  trap - EXIT
  trap '' INT TERM
  for pid in "${children[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${children[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  for pid in "${children[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM

start_child "$REPO_DIR/runtime/dual/launcher.log" "$REPO_DIR/scripts/start_dual_uav.sh"
echo 'Starting Gazebo god view and the two PX4 vehicles...'
sleep 8
check_children

start_child "$REPO_DIR/runtime/dual/camera.log" "$REPO_DIR/scripts/dual_camera.sh"
if [[ "$FOLLOW_MODE" == drone || "$FOLLOW_MODE" == virtual_contact ]]; then
  start_child "$REPO_DIR/runtime/dual/detector.log" "$REPO_DIR/scripts/yolo_detector.sh"
else
  start_child "$REPO_DIR/runtime/dual/detector.log" "$REPO_DIR/scripts/red_detector.sh"
fi
start_child "$REPO_DIR/runtime/dual/follow.log" "$REPO_DIR/scripts/follow.sh"
sleep 2
check_children

echo 'Simulation and camera/tracking processes started. This terminal now controls the target vehicle.'
echo 'Press T to take off, X to land, and Ctrl-C to land and shut down.'
bash "$REPO_DIR/scripts/dual_keyboard.sh"
