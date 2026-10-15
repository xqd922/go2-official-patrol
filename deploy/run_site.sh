#!/usr/bin/env bash
# 启动某个站点的巡检（systemd ExecStart；也可人工执行做现场联调）。
#   deploy/run_site.sh <站点名>
# 站点级差异全部来自 <站点目录>/site.yaml 生成的 site.env：
#   GO2_IFACE / GO2_DOG_IP / GO2_PATROL_* / GO2_RVIZ
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SITE="${1:-${GO2_SITE:-}}"

if [ -z "$SITE" ]; then
  echo "用法: run_site.sh <站点名>（或设置 GO2_SITE）" >&2
  exit 2
fi

# 机器级配置（生产机由 install.sh 写入）→ 站点级配置 → 通用环境
if [ -f /etc/go2/site.env ]; then
  # shellcheck disable=SC1091
  source /etc/go2/site.env
fi
SITES_DIR="${GO2_SITES:-${WS_ROOT}/sites}"
SITE_DIR="${SITES_DIR}/${SITE}"
if [ -f "${SITE_DIR}/site.env" ]; then
  # shellcheck disable=SC1091
  source "${SITE_DIR}/site.env"
fi
# shellcheck source=go2_env.sh
source "${SCRIPT_DIR}/go2_env.sh"

if [ ! -d "$SITE_DIR" ]; then
  echo "[run_site] 站点目录不存在: ${SITE_DIR}" >&2
  echo "[run_site] 先执行: deploy/apply_site.sh --site-dir <站点源目录> --name ${SITE}" >&2
  exit 3
fi

LAUNCH_ARGS=(
  "map:=${SITE_DIR}/map.yaml"
  "params_file:=${SITE_DIR}/nav2_params.yaml"
  "waypoints_file:=${SITE_DIR}/waypoints.yaml"
  "audio_dir:=${SITE_DIR}/audio"
  "rviz:=${GO2_RVIZ:-false}"
)
if [ -n "${GO2_PATROL_LOOPS:-}" ]; then
  LAUNCH_ARGS+=("loops:=${GO2_PATROL_LOOPS}")
fi
if [ -n "${GO2_PATROL_RETURN_HOME:-}" ]; then
  LAUNCH_ARGS+=("return_home:=${GO2_PATROL_RETURN_HOME}")
fi
if [ -n "${GO2_PATROL_BATTERY_LOW:-}" ]; then
  LAUNCH_ARGS+=("battery_low:=${GO2_PATROL_BATTERY_LOW}")
fi
if [ -n "${GO2_PATROL_ARRIVAL_TOLERANCE:-}" ]; then
  LAUNCH_ARGS+=("arrival_tolerance:=${GO2_PATROL_ARRIVAL_TOLERANCE}")
fi

echo "[run_site] 站点=${SITE} 目录=${SITE_DIR}"
echo "[run_site] ros2 launch go2_patrol_bringup patrol.launch.py ${LAUNCH_ARGS[*]}"

exec ros2 launch go2_patrol_bringup patrol.launch.py "${LAUNCH_ARGS[@]}"
