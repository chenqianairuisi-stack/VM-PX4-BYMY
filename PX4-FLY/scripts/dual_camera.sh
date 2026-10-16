#!/usr/bin/env bash
# 查看实时画面，订阅并且显示相机图像
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$REPO_DIR/install/local_setup.bash"
if [[ "${FOLLOW_MODE:-drone}" == drone || "${FOLLOW_MODE:-drone}" == virtual_contact ]]; then
  exec ros2 run image_tools showimage --ros-args -r image:=/tracker/yolo/annotated
fi
exec ros2 run image_tools showimage --ros-args -r image:=/tracker/front_camera/image_raw
