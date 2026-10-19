#!/usr/bin/env bash
# 把一份站点资产（地图/点位/语音/site.yaml）落到站点目录，并生成站点参数文件。
#
# 生产机：sudo deploy/apply_site.sh --site-dir /media/usb/site-office --name office
# 开发机：deploy/apply_site.sh --site-dir deploy/site.example --name demo --sites-dir ./sites
#
# 站点源目录约定（缺 map.yaml / 地图数据会直接失败）：
#   map.yaml            地图描述（image: 指向 .pgm）
#   map.pgm             地图数据（文件名以 map.yaml 的 image 字段为准）
#   waypoints.yaml      巡检点位（缺省用仓库示例填充，需现场标定）
#   site.yaml           站点参数（初始位姿、巡检任务参数等，可选）
#   audio/<id>.wav      到点语音（可选）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

SITE_SRC=""
SITE_NAME=""

# 机器级配置（生产机由 install.sh 写入）：GO2_SITES=/opt/go2/sites。
# 不读它的话，手工执行会把站点落到 <版本目录>/sites，而 systemd / run_site.sh
# 找的是 /opt/go2/sites —— 表现为"站点明明落地了，服务却说目录不存在"。
# 调用方显式给的 GO2_SITES 优先（否则 `GO2_SITES=./sites deploy/apply_site.sh ...`
# 会被 /etc 里的值顶掉）。
if [ -f /etc/go2/site.env ]; then
  # `${GO2_SITES+x}` 能区分"没给"和"显式给了空串"，与 go2_env.sh 的判定保持一致
  _apply_sites_set="${GO2_SITES+x}"
  _apply_caller_sites="${GO2_SITES:-}"
  # shellcheck disable=SC1091
  source /etc/go2/site.env
  if [ -n "$_apply_sites_set" ]; then
    export GO2_SITES="$_apply_caller_sites"
  fi
  unset _apply_sites_set _apply_caller_sites
fi

SITES_DIR="${GO2_SITES:-${WS_ROOT}/sites}"

usage() {
  cat <<'EOF'
用法: deploy/apply_site.sh --site-dir <站点源目录> [选项]

  --site-dir <目录>   站点源目录（含 map.yaml/map.pgm/waypoints.yaml/audio/）
  --name <站点名>     目标站点名，默认取源目录名（systemd 实例名就用它）
  --sites-dir <目录>  站点根目录，默认 /etc/go2/site.env 的 GO2_SITES，否则 <工作空间>/sites
  -h, --help          显示本帮助
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --site-dir)
      [ "$#" -ge 2 ] || { echo "--site-dir 缺少取值" >&2; exit 2; }
      SITE_SRC="$2"
      shift 2
      ;;
    --name)
      [ "$#" -ge 2 ] || { echo "--name 缺少取值" >&2; exit 2; }
      SITE_NAME="$2"
      shift 2
      ;;
    --sites-dir)
      [ "$#" -ge 2 ] || { echo "--sites-dir 缺少取值" >&2; exit 2; }
      SITES_DIR="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [ -z "$SITE_SRC" ]; then
  usage >&2
  exit 2
fi
if [ ! -d "$SITE_SRC" ]; then
  echo "[apply_site] 站点源目录不存在: ${SITE_SRC}" >&2
  exit 2
fi
SITE_SRC="$(cd "$SITE_SRC" && pwd)"
SITE_NAME="${SITE_NAME:-$(basename "$SITE_SRC")}"
DEST="${SITES_DIR}/${SITE_NAME}"

echo "[apply_site] 源目录  = ${SITE_SRC}"
echo "[apply_site] 站点目录 = ${DEST}"

# --- 0. 落地前预检：先在**源目录**上校验 site.yaml，通过了才动站点目录 ---
# 站点资产是覆盖式落地。若先拷再校验，一旦 site.yaml 写错（例如安全阈值写反），
# 已上线站点的 site.yaml 就被坏的那份换掉了，而 nav2_params.yaml / site.env 还是旧的
# ——目录进入半成品状态，服务重启后读的是旧参数配新 site.yaml，现场排查方向全错。
echo "[apply_site] 预检源目录（不写入任何文件）"
python3 "${SCRIPT_DIR}/tools/render_site.py" \
  --site-dir "$SITE_SRC" \
  --name "$SITE_NAME" \
  --ws-root "$WS_ROOT" \
  --check-only

mkdir -p "$DEST/audio"

# --- 1. 复制站点资产 ---
if [ ! -f "${SITE_SRC}/map.yaml" ]; then
  echo "[apply_site] 缺少 ${SITE_SRC}/map.yaml（建图后用 scripts/save_map.sh 生成）" >&2
  exit 3
fi
cp -f "${SITE_SRC}/map.yaml" "${DEST}/map.yaml"

map_image="$(awk -F': *' '/^[[:space:]]*image:/ {print $2; exit}' "${SITE_SRC}/map.yaml" | tr -d '"'"'"'')"
map_image="${map_image:-map.pgm}"
case "$map_image" in
  /*) map_src="$map_image" ;;
  *) map_src="${SITE_SRC}/${map_image}" ;;
esac
if [ -f "$map_src" ]; then
  cp -f "$map_src" "${DEST}/$(basename "$map_src")"
  echo "[apply_site] 已复制地图: $(basename "$map_src")"
else
  echo "[apply_site] 地图数据文件不存在: ${map_src}" >&2
  exit 3
fi

if [ -f "${SITE_SRC}/waypoints.yaml" ]; then
  cp -f "${SITE_SRC}/waypoints.yaml" "${DEST}/waypoints.yaml"
  echo "[apply_site] 已复制点位: waypoints.yaml"
else
  cp -f "${WS_ROOT}/src/go2_patrol_bringup/config/waypoints.yaml" "${DEST}/waypoints.yaml"
  echo "[apply_site] 未提供 waypoints.yaml，已用仓库示例填充——必须按现场重新标定！"
fi

if [ -f "${SITE_SRC}/site.yaml" ]; then
  cp -f "${SITE_SRC}/site.yaml" "${DEST}/site.yaml"
  echo "[apply_site] 已复制站点参数: site.yaml"
fi

if [ -d "${SITE_SRC}/audio" ]; then
  find "${SITE_SRC}/audio" -maxdepth 1 -name '*.wav' -exec cp -f {} "${DEST}/audio/" \;
fi

# --- 2. 生成站点参数（nav2_params.yaml / site.env）并校验 ---
python3 "${SCRIPT_DIR}/tools/render_site.py" \
  --site-dir "$DEST" \
  --name "$SITE_NAME" \
  --ws-root "$WS_ROOT"

# --- 3. 权限 ---
chmod -R a+rX "$DEST"
if [ "$(id -u)" -eq 0 ]; then
  chown -R root:root "$DEST"
fi

# --- 4. 启动前自检（失败不影响落地，但要把问题打出来） ---
set +e
GO2_SITES="$SITES_DIR" bash "${SCRIPT_DIR}/preflight.sh" "$SITE_NAME"
preflight_rc=$?
set -e

cat <<EOF

[apply_site] 站点「${SITE_NAME}」已就绪: ${DEST}

生产机（systemd）:
  sudo systemctl enable --now go2-patrol@${SITE_NAME}   # 开机自启 + 立即启动
  journalctl -u go2-patrol@${SITE_NAME} -f              # 跟踪日志
  sudo systemctl stop go2-patrol@${SITE_NAME}           # 停止

开发机本地联调（不装 systemd）:
  GO2_SITES=${SITES_DIR} deploy/run_site.sh ${SITE_NAME}

自检退出码: ${preflight_rc}（0=通过；非 0 说明上面有 [FAIL] 项，先解决再启动）
EOF
