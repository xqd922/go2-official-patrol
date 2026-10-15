#!/usr/bin/env bash
set -euo pipefail
_go2_save_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 没加载过 ROS 环境时自动加载（支持新终端直接运行）
if ! command -v ros2 >/dev/null 2>&1; then
  set +u
  source "${_go2_save_dir}/setup.sh"
  set -u
fi
name="${1:-office_floor}"
map_dir="${_go2_save_dir}/../src/go2_patrol_bringup/maps"
mkdir -p "$map_dir"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: {data: '${map_dir}/${name}'}"
