#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD="${PX4_BUILD:-$PX4_DIR/build/px4_sitl_default}"

if pgrep -x px4 >/dev/null || pgrep -x gzserver >/dev/null || pgrep -x MicroXRCEAgent >/dev/null; then
  echo 'PX4, gzserver or MicroXRCEAgent is already running. Stop the previous simulation first.' >&2
  exit 1
fi
if [[ ! -x "$PX4_BUILD/bin/px4" ]]; then
  echo "Build PX4 first: cd $PX4_DIR && make px4_sitl_default sitl_gazebo-classic" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$PX4_BUILD"
export GAZEBO_MODEL_PATH="$REPO_DIR/models:${GAZEBO_MODEL_PATH:-}"
export ROS_VERSION=2
export PX4_SIM_MODEL=gazebo-classic_iris
export PX4_SIM_WORLD=empty
export PX4_SIM_HOSTNAME=localhost
export PX4_SIM_SPEED_FACTOR=1
export GAZEBO_MODEL_DATABASE_URI=""
mkdir -p "$REPO_DIR/runtime"
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM
MicroXRCEAgent udp4 -p 8888 >"$REPO_DIR/runtime/agent.log" 2>&1 &
pids+=("$!")
gzserver "$REPO_DIR/worlds/training.world" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
  >"$REPO_DIR/runtime/gazebo.log" 2>&1 &
pids+=("$!")
if [[ "${HEADLESS:-0}" != 1 ]]; then
  gzclient >"$REPO_DIR/runtime/gazebo_gui.log" 2>&1 &
  pids+=("$!")
fi
"$PX4_BUILD/bin/px4" -d -s "$REPO_DIR/config/sitl.rc" \
  -w "$REPO_DIR/runtime" "$PX4_BUILD/etc" &
pids+=("$!")
echo "Simulation running. Logs and PX4 parameters: $REPO_DIR/runtime"
wait -n "${pids[@]}"
