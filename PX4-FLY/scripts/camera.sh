#!/usr/bin/env bash
set -eo pipefail
export GDK_BACKEND="${GDK_BACKEND:-x11}"
source /opt/ros/humble/setup.bash
exec ros2 run image_tools showimage --ros-args --log-level warn \
  -p reliability:=best_effort -p depth:=1 -r image:=/front_camera/image_raw
