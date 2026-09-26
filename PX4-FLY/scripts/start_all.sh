#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONNOUSERSITE=1

if [[ "${1:-}" == "--single-terminal" ]]; then
  exec "$REPO_DIR/scripts/start_single_terminal.sh"
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  cat >&2 <<'EOF'
gnome-terminal was not found. This launcher opens the required GUI and keyboard
windows automatically. Install it with: sudo apt install gnome-terminal
EOF
  exit 1
fi

if pgrep -x px4 >/dev/null || pgrep -x gzserver >/dev/null || pgrep -x MicroXRCEAgent >/dev/null; then
  echo 'A PX4, Gazebo, or MicroXRCEAgent process is already running.' >&2
  echo 'Stop the previous simulation with Ctrl-C in its Gazebo terminal first.' >&2
  exit 1
fi

SIM_COMMAND="cd '$REPO_DIR' && ./scripts/start_dual_uav.sh"
if [[ "${1:-}" == "--headless" ]]; then
  SIM_COMMAND="cd '$REPO_DIR' && HEADLESS=1 ./scripts/start_dual_uav.sh"
fi

launch() {
  local title="$1"
  local command="$2"
  gnome-terminal --title="$title" -- bash -lc "$command"
}

launch 'PX4-FLY Gazebo 上帝视角' "$SIM_COMMAND"
sleep 8
launch 'PX4-FLY 目标机键盘' "cd '$REPO_DIR' && ./scripts/dual_keyboard.sh"
launch 'PX4-FLY 追踪机第一视角' "cd '$REPO_DIR' && ./scripts/dual_camera.sh"
launch 'PX4-FLY 红气球识别' "cd '$REPO_DIR' && ./scripts/red_detector.sh"
launch 'PX4-FLY 自主跟随' "cd '$REPO_DIR' && ./scripts/follow.sh"

echo 'All PX4-FLY windows were launched.'
echo 'Use the target keyboard window to press T for takeoff and X for landing.'
echo 'Close the simulation by pressing Ctrl-C in the Gazebo terminal window.'
