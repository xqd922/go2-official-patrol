#!/usr/bin/env bash
set -euo pipefail
name="${1:-office_floor}"
map_dir="$HOME/go2_official_ws/src/go2_patrol_bringup/maps"
mkdir -p "$map_dir"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: {data: '${map_dir}/${name}.yaml'}"
