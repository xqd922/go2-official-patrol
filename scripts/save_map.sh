#!/usr/bin/env bash
set -euo pipefail

# 工作空间根目录由脚本自身位置推导（不依赖 $HOME/go2_official_ws 这种写死路径）。
# shellcheck source=scripts/repo_root.sh
source "$(dirname "$(realpath "${BASH_SOURCE[0]}")")/repo_root.sh"

# 没加载过 ROS 环境时自动加载（支持新终端直接运行）
if ! command -v ros2 >/dev/null 2>&1; then
  set +u
  source "${GO2_WS_ROOT}/scripts/setup.sh"
  set -u
fi
name="${1:-office_floor}"
# 直接写源码树里的 maps。maps/ 已纳入 symlink-install（见该目录下的 .gitkeep），
# 因此 install/share/go2_patrol_bringup/maps 会链接到同一份文件，两边保持一致。
map_dir="${GO2_WS_ROOT}/src/go2_patrol_bringup/maps"
mkdir -p "$map_dir"

# 注意：slam_toolbox 的 /slam_toolbox/save_map 服务会忽略请求里的 name 字段，
# 地图总是以 map_<时间戳> 写到进程当前工作目录。这里改用 nav2 的 map_saver_cli，
# 它接受显式的 -f 输出路径，不依赖调用方的当前工作目录。
out="${map_dir}/${name}"
# map_saver_cli 装在 nav2_map_server 的 lib 下，通常不在 PATH 里，这里显式解析。
saver="$(command -v map_saver_cli || true)"
if [ -z "$saver" ]; then
  saver="$(ros2 pkg prefix nav2_map_server 2>/dev/null)/lib/nav2_map_server/map_saver_cli"
fi
if [ ! -x "$saver" ]; then
  echo "[save_map] 找不到 map_saver_cli（nav2_map_server 是否已安装？）" >&2
  exit 1
fi
"$saver" -t /map -f "$out" --mode trinary

# map_saver_cli 写出的是 <out>.yaml + <out>.pgm；校验两者都存在且非空。
for suffix in yaml pgm; do
  if [ ! -s "${out}.${suffix}" ]; then
    echo "[save_map] 失败：未生成 ${out}.${suffix}" >&2
    exit 1
  fi
done
echo "[save_map] 已保存：${out}.yaml 与 ${out}.pgm"
