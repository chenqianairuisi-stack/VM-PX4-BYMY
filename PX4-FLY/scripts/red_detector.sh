#!/usr/bin/env bash
# 红色检测器订阅 ros2 图像话题，检测红色物体并发布其位置
set -eo pipefail
export PYTHONNOUSERSITE=1
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$REPO_DIR/install/local_setup.bash"
exec ros2 run px4_fly_control red_color_detector "$@"
