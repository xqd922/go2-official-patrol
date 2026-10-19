#!/usr/bin/env bash
# 启动前自检：由 systemd ExecStartPre 调用，也可人工执行。
#   deploy/preflight.sh <站点名>
# 硬性检查不通过时以非 0 退出（systemd 会因此拒绝启动，避免"起了但跑不动"）。
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SITE="${1:-${GO2_SITE:-}}"

fail=0
warn=0
ok() { printf '  [ OK ] %s\n' "$*"; }
bad() { printf '  [FAIL] %s\n' "$*"; fail=$((fail + 1)); }
meh() { printf '  [WARN] %s\n' "$*"; warn=$((warn + 1)); }

echo "[preflight] 站点=${SITE:-<未指定>}"

if [ -z "$SITE" ]; then
  bad "未指定站点名（用法: deploy/preflight.sh <站点名>）"
  exit 2
fi

export GO2_ENV_QUIET=1
set +u
# shellcheck source=go2_env.sh
source "${SCRIPT_DIR}/go2_env.sh"
set -u

echo "[preflight] WS=${GO2_WS} SITES=${GO2_SITES} IFACE=${GO2_IFACE} DOG=${GO2_DOG_IP}"

# --- 1. 工作空间与依赖 ---
if [ -f /opt/ros/humble/setup.bash ]; then
  ok "ROS 2 Humble 已安装"
else
  bad "缺少 /opt/ros/humble/setup.bash，先安装 ROS 2 Humble"
fi

if [ -f "${GO2_WS}/install/setup.bash" ]; then
  ok "工作空间已编译: ${GO2_WS}/install"
else
  bad "工作空间未编译: ${GO2_WS}/install/setup.bash 不存在（执行 scripts/bootstrap.sh）"
fi

if [ -f "${GO2_WS}/src/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
  ok "unitree_ros2 / CycloneDDS 已编译"
else
  bad "unitree_ros2 未编译：${GO2_WS}/src/unitree_ros2/cyclonedds_ws/install 缺失"
fi

if command -v ros2 >/dev/null 2>&1; then
  ok "ros2 命令可用（$(command -v ros2)）"
else
  bad "PATH 里找不到 ros2（环境未 source 成功）"
fi

# --- 2. 站点资产 ---
SITE_DIR="${GO2_SITES}/${SITE}"
if [ -d "$SITE_DIR" ]; then
  ok "站点目录存在: ${SITE_DIR}"
else
  bad "站点目录不存在: ${SITE_DIR}（先执行 deploy/apply_site.sh --site-dir <站点源目录>）"
fi

if [ -f "${SITE_DIR}/map.yaml" ]; then
  ok "地图描述文件存在: map.yaml"
  map_image="$(awk -F': *' '/^[[:space:]]*image:/ {print $2; exit}' "${SITE_DIR}/map.yaml" | tr -d '"'"'"'')"
  map_image="${map_image:-map.pgm}"
  case "$map_image" in
    /*) map_path="$map_image" ;;
    *) map_path="${SITE_DIR}/${map_image}" ;;
  esac
  if [ -f "$map_path" ]; then
    ok "地图数据文件存在: $(basename "$map_path")"
  else
    bad "地图数据文件缺失: ${map_path}"
  fi
else
  bad "缺少 ${SITE_DIR}/map.yaml"
fi

if [ -f "${SITE_DIR}/waypoints.yaml" ]; then
  wp_count="$(grep -cE '^[[:space:]]*-[[:space:]]*name:' "${SITE_DIR}/waypoints.yaml" 2>/dev/null || true)"
  wp_count="${wp_count:-0}"
  if [ "${wp_count:-0}" -gt 0 ]; then
    ok "巡检点位文件存在: waypoints.yaml（${wp_count} 个点位）"
  else
    meh "waypoints.yaml 里没有解析到点位（检查 name: 缩进）"
  fi
else
  bad "缺少 ${SITE_DIR}/waypoints.yaml"
fi

if [ -f "${SITE_DIR}/nav2_params.yaml" ]; then
  ok "站点 Nav2 参数存在: nav2_params.yaml"
else
  bad "缺少 ${SITE_DIR}/nav2_params.yaml（执行 deploy/apply_site.sh 生成）"
fi

if [ -d "${SITE_DIR}/audio" ]; then
  audio_count="$(find "${SITE_DIR}/audio" -maxdepth 1 -name '*.wav' 2>/dev/null | wc -l | tr -d ' ')"
  if [ "${audio_count:-0}" -gt 0 ]; then
    ok "到点语音文件: ${audio_count} 个 .wav"
  else
    meh "站点 audio/ 里没有 .wav，到点只会打印日志不发声"
  fi
else
  meh "站点没有 audio/ 目录，到点只会打印日志不发声"
fi

# 站点 site.env 里的网卡 / 狗地址优先于机器级配置——run_site.sh 就是这么做的。
# 不在这里同步的话：site.yaml 的 iface 写错时，下面只检查机器级/自动探测出来的网卡
# （存在，于是报 OK），而真正启动时 CYCLONEDDS_URI 会绑到那个不存在的网卡上，
# 表现就是"自检全绿、服务起来了、但一条狗数据都收不到"。
if [ -f "${SITE_DIR}/site.env" ]; then
  site_net="$(
    set +u; set -a
    # shellcheck disable=SC1091
    . "${SITE_DIR}/site.env" >/dev/null 2>&1 || true
    printf '%s\n%s\n' "${GO2_IFACE:-}" "${GO2_DOG_IP:-}"
  )"
  site_iface="$(printf '%s\n' "$site_net" | sed -n 1p)"
  site_dog_ip="$(printf '%s\n' "$site_net" | sed -n 2p)"
  if [ -n "$site_iface" ] && [ "$site_iface" != "$GO2_IFACE" ]; then
    echo "[preflight] 网卡按站点 site.env 覆盖: ${GO2_IFACE} → ${site_iface}"
    GO2_IFACE="$site_iface"
  fi
  if [ -n "$site_dog_ip" ] && [ "$site_dog_ip" != "$GO2_DOG_IP" ]; then
    echo "[preflight] 狗地址按站点 site.env 覆盖: ${GO2_DOG_IP} → ${site_dog_ip}"
    GO2_DOG_IP="$site_dog_ip"
  fi
fi

# --- 3. 网络 ---
if command -v ip >/dev/null 2>&1; then
  if ip link show "$GO2_IFACE" >/dev/null 2>&1; then
    ok "网卡存在: ${GO2_IFACE}"
  else
    bad "网卡不存在: ${GO2_IFACE}（在 /etc/go2/site.env 里改 GO2_IFACE，或站点 site.yaml 的 iface）"
  fi
fi

if command -v ping >/dev/null 2>&1; then
  if ping -c 1 -W 1 "$GO2_DOG_IP" >/dev/null 2>&1; then
    ok "机器狗可达: ${GO2_DOG_IP}"
  else
    meh "ping 不通 ${GO2_DOG_IP}（网线/网卡配置/狗未上电时属正常，跑起来再确认 /lf/sportmodestate 有数据）"
  fi
fi

echo "[preflight] 结果: 失败 ${fail} 项，警告 ${warn} 项"
if [ "$fail" -gt 0 ]; then
  echo "[preflight] 存在硬性失败项，拒绝启动" >&2
  exit 1
fi
