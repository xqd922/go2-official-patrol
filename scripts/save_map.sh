#!/usr/bin/env bash
set -euo pipefail
# 没加载过 ROS 环境时自动加载（支持新终端直接运行）
if ! command -v ros2 >/dev/null 2>&1; then
  set +u
  source "$HOME/go2_official_ws/scripts/setup.sh"
  set -u
fi
name="${1:-office_floor}"
map_dir="$HOME/go2_official_ws/src/go2_patrol_bringup/maps"
mkdir -p "$map_dir"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: {data: '${map_dir}/${name}'}"
