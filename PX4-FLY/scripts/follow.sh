#!/usr/bin/env bash
# 启动追踪器的环境准备和参数配置--启动包装
set -eo pipefail
export PYTHONNOUSERSITE=1
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$REPO_DIR/install/local_setup.bash"
if [[ "${FOLLOW_MODE:-drone}" == drone ]]; then
  exec ros2 run px4_fly_control safe_visual_follow --ros-args --params-file "$REPO_DIR/config/drone_follow.yaml" "$@"
fi
if [[ "${FOLLOW_MODE:-drone}" == virtual_contact ]]; then
  exec ros2 run px4_fly_control safe_visual_follow --ros-args --params-file "$REPO_DIR/config/drone_virtual_contact.yaml" "$@"
fi
exec ros2 run px4_fly_control safe_visual_follow "$@"
