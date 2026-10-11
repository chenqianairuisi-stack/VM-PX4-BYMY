#!/usr/bin/env bash
# 目标机键盘控制的环境准备和参数配置--启动包装
set -eo pipefail
export PYTHONNOUSERSITE=1
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 加载ros2和px4_fly_sim的环境
source /opt/ros/humble/setup.bash

# 加载本工程编译安装后的 ROS 2 包，让系统可以找到 px4_fly_control
source "$REPO_DIR/install/local_setup.bash"

# 加载px4的环境
echo 'This terminal is the target-drone operation interface.'

# 在终端打印键盘操作提示
echo 'T takeoff | W/A/S/D move | R/F altitude | J/L yaw | X land | H help'
exec ros2 run px4_fly_control keyboard_control --ros-args -p takeoff_altitude:=3.0 "$@"
