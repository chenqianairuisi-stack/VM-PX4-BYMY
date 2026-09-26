#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD="${PX4_BUILD:-$PX4_DIR/build/px4_sitl_default}"

if [[ ! -x "$PX4_BUILD/bin/px4" ]]; then
  echo "Build PX4 first: cd $PX4_DIR && make px4_sitl_default sitl_gazebo-classic" >&2
  exit 1
fi
if pgrep -x px4 >/dev/null || pgrep -x gzserver >/dev/null || pgrep -x MicroXRCEAgent >/dev/null; then
  echo 'PX4, gzserver or MicroXRCEAgent is already running. Stop the previous simulation first.' >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
if [[ ! -f "$REPO_DIR/install/px4_fly_sim/lib/libballoon_contact.so" ]]; then
  echo 'Build the project first: ./scripts/build.sh' >&2
  exit 1
fi
source "$REPO_DIR/install/local_setup.bash"
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$PX4_BUILD"
export GAZEBO_MODEL_PATH="$REPO_DIR/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$REPO_DIR/install/px4_fly_sim/lib:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""
export ROS_VERSION=2
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PX4_SIM_MODEL=gazebo-classic_iris
export PX4_SIM_WORLD=dual_uav
mkdir -p "$REPO_DIR/runtime/dual/target" "$REPO_DIR/runtime/dual/tracker"

pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM

MicroXRCEAgent udp4 -p 8888 >"$REPO_DIR/runtime/dual/agent.log" 2>&1 &
pids+=("$!")
gzserver "$REPO_DIR/worlds/dual_uav.world" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
  --ros-args --params-file "$REPO_DIR/config/gazebo.yaml" \
  >"$REPO_DIR/runtime/dual/gazebo.log" 2>&1 &
pids+=("$!")
if [[ "${HEADLESS:-0}" != 1 ]]; then
  gzclient >"$REPO_DIR/runtime/dual/gazebo_gui.log" 2>&1 &
  pids+=("$!")
fi

# Instance 0 is the manually controlled target and publishes /fmu/*.
"$PX4_BUILD/bin/px4" -i 0 -d -s "$REPO_DIR/config/sitl.rc" \
  -w "$REPO_DIR/runtime/dual/target" "$PX4_BUILD/etc" \
  >"$REPO_DIR/runtime/dual/target/px4.log" 2>&1 &
pids+=("$!")

# Instance 1 is the autonomous tracker and publishes /px4_1/fmu/*.
"$PX4_BUILD/bin/px4" -i 1 -d -s "$REPO_DIR/config/sitl.rc" \
  -w "$REPO_DIR/runtime/dual/tracker" "$PX4_BUILD/etc" \
  >"$REPO_DIR/runtime/dual/tracker/px4.log" 2>&1 &
pids+=("$!")

echo 'Dual simulation running.'
echo 'Target PX4 topics: /fmu/*'
echo 'Tracker PX4 topics: /px4_1/fmu/*'
echo "Logs: $REPO_DIR/runtime/dual"
wait -n "${pids[@]}"
