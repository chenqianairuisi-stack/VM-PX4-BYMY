#!/usr/bin/env bash
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_PATH="$REPO_DIR/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""
# Separate Gazebo/ROS domains keep capture independent of any flight session.
export GAZEBO_MASTER_URI=http://127.0.0.1:11355
export ROS_DOMAIN_ID=55
mkdir -p "$REPO_DIR/runtime/dataset"
python3 "$REPO_DIR/scripts/collect_drone_dataset.py" --world "$REPO_DIR/runtime/dataset/capture.world"
gzserver "$REPO_DIR/runtime/dataset/capture.world" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so >"$REPO_DIR/runtime/dataset/gazebo.log" 2>&1 &
capture_pid=$!
cleanup() {
  kill -TERM "$capture_pid" 2>/dev/null || true
  for attempt in {1..30}; do
    if ! kill -0 "$capture_pid" 2>/dev/null; then break; fi
    sleep 0.1
  done
  kill -KILL "$capture_pid" 2>/dev/null || true
  wait "$capture_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
python3 "$REPO_DIR/scripts/collect_drone_dataset.py" "$@"
