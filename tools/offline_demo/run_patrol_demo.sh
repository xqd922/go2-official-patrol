#!/usr/bin/env bash
# 离线闭环巡检演示：没有机器狗、没有现场数据，也要把生产链路整条跑通并验收。
#
# 跑的链路（与生产机一致）：
#   make_demo_map.py / make_demo_audio.py   造假数据（地图/语音）
#   check_site.py                           站点资产校验
#   deploy/apply_site.sh → render_site.py   站点落地 + 生成站点 Nav2 参数
#   deploy/run_site.sh → patrol.launch.py   巡检栈（感知/驱动/Nav2/巡检/播报）
#   demo_verify.py                          逐点验收（到点顺序/精度/行程/播报）
#
# 用法:
#   tools/offline_demo/run_patrol_demo.sh                 # 跑一轮无头验收
#   tools/offline_demo/run_patrol_demo.sh --rviz          # 带 RViz（需要显示环境）
#   tools/offline_demo/run_patrol_demo.sh --loops 0 --timeout 600
#   tools/offline_demo/run_patrol_demo.sh --battery-drop-after 60   # 验证低电量返航
#   tools/offline_demo/run_patrol_demo.sh --obstacles "0.5,0,0.6,0.6,0.6"
#   tools/offline_demo/run_patrol_demo.sh --keep          # 验收后不杀进程
#   tools/offline_demo/run_patrol_demo.sh --clean         # 只清理残留进程
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SITE_SRC="${SCRIPT_DIR}/site-demo"
SITE_NAME="${GO2_SITE_NAME:-demo}"
RUN_DIR="${GO2_DEMO_RUN_DIR:-/tmp/go2_offline_demo}"
# 演示产物默认落在 RUN_DIR 下，不写进工作空间（避免 git status 变脏）；
# 要跟生产一样落到 <工作空间>/sites 就显式给 GO2_SITES。
SITES_DIR="${GO2_SITES:-${RUN_DIR}/sites}"

OBSTACLES=""
RVIZ="false"
LOOPS="1"
RETURN_HOME="true"
TIMEOUT="420"
BATTERY_DROP_AFTER=""
APPLY="true"
KEEP="false"

usage() {
  cat <<'EOF'
用法: tools/offline_demo/run_patrol_demo.sh [选项]

  --obstacles SPEC        房间障碍规格 "x,y[,sx[,sy[,h]]];…"（默认 geometry.py 的木箱）
  --rviz                  同时启动 RViz（无显示器环境不要用）
  --loops N               巡检圈数（0=无限循环，验收按一圈判定），默认 1
  --no-return-home        巡检结束不回 home 点
  --timeout SEC           验收超时，默认 420
  --battery-drop-after S  运行 S 秒后把电量跳到 15%，验证低电量返航
  --no-apply              跳过 apply_site（沿用上一次生成的站点）
  --keep                  验收结束后保留进程（手动看 RViz/日志）
  --clean                 只清理本演示启过的进程后退出
  -h, --help              显示本帮助
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --obstacles) OBSTACLES="${2:-}"; shift 2 ;;
    --rviz) RVIZ="true"; shift ;;
    --loops) LOOPS="${2:-1}"; shift 2 ;;
    --no-return-home) RETURN_HOME="false"; shift ;;
    --timeout) TIMEOUT="${2:-420}"; shift 2 ;;
    --battery-drop-after) BATTERY_DROP_AFTER="${2:-60}"; shift 2 ;;
    --no-apply) APPLY="false"; shift ;;
    --keep) KEEP="true"; shift ;;
    --clean) KEEP="false"; CLEAN_ONLY="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done
CLEAN_ONLY="${CLEAN_ONLY:-false}"

mkdir -p "$RUN_DIR"
PID_FILE="${RUN_DIR}/pids"
PIDS=()

# ---------------------------------------------------------------- 进程管理
stop_all() {
  local pid
  if [ -f "$PID_FILE" ]; then
    while read -r pid; do
      [ -n "$pid" ] || continue
      # setsid 起的进程自成一个进程组，负号 = 整组一起收，避免留下孤儿节点
      kill -INT -- "-${pid}" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    done < "$PID_FILE"
  fi
  sleep 2
  # 兜底：launch 若把子节点放到了别的进程组，按名字再收一遍（都是本演示起的节点）
  for pattern in \
    '[f]ake_go2_loop.py' '[p]atrol.launch.py' '[r]un_site.sh' '[p]atrol_manager' \
    '[v]oice_player' '[p]ointcloud_to_laserscan' '[g]o2_patrol_driver' \
    '[n]av2_amcl/amcl' '[n]av2_controller/controller_server' \
    '[n]av2_planner/planner_server' '[n]av2_bt_navigator/bt_navigator' \
    '[n]av2_behaviors/behavior_server' '[n]av2_map_server/map_server' \
    '[n]av2_lifecycle_manager/lifecycle_manager' '[r]viz2' \
    '[c]omponent_container' ; do
    pkill -f "$pattern" 2>/dev/null || true
  done
  rm -f "$PID_FILE"
}

launch_bg() {  # launch_bg <日志名> <命令...>
  local name="$1"; shift
  setsid nohup "$@" >"${RUN_DIR}/${name}.log" 2>&1 </dev/null &
  local pid=$!
  PIDS+=("$pid")
  echo "$pid" >> "$PID_FILE"
}

if [ "$CLEAN_ONLY" = "true" ]; then
  stop_all
  echo "[demo] 已清理残留进程"
  exit 0
fi

cleanup() {
  local code=$?
  if [ "$KEEP" = "true" ]; then
    echo "[demo] --keep：进程保留，日志在 ${RUN_DIR}；收工请执行 $0 --clean"
    return
  fi
  echo "[demo] 清理演示进程…"
  stop_all
  return $code
}
trap cleanup EXIT

# ---------------------------------------------------------------- 环境
if [ ! -f "${WS_ROOT}/install/setup.bash" ]; then
  echo "[demo] 工作空间未编译：${WS_ROOT}/install/setup.bash 不存在" >&2
  echo "[demo] 先执行 scripts/bootstrap.sh（或 colcon build）" >&2
  exit 3
fi
export GO2_SITES="$SITES_DIR"
export GO2_ENV_QUIET=1
# shellcheck source=../../deploy/go2_env.sh
source "${WS_ROOT}/deploy/go2_env.sh"
# go2_env.sh 会读机器级 /etc/go2/site.env；它里面若写了 GO2_SITES，演示必须仍然
# 落在本次 RUN_DIR 下（否则 run_site.sh 会去 /opt/go2/sites 找站点，直接 exit 3）。
export GO2_SITES="$SITES_DIR"

# ---------------------------------------------------------------- 1. 造数据
echo "[demo] 1/5 生成离线演示数据（地图 + 语音）"
python3 "${SCRIPT_DIR}/make_demo_map.py" --out-dir "$SITE_SRC" --name map --obstacles "$OBSTACLES"
python3 "${SCRIPT_DIR}/make_demo_audio.py" --site-dir "$SITE_SRC"
python3 "${SCRIPT_DIR}/check_site.py" --site-dir "$SITE_SRC" --obstacles "$OBSTACLES"

# ---------------------------------------------------------------- 2. 站点落地
SITE_SRC_EFFECTIVE="$SITE_SRC"
if [ "$RVIZ" = "true" ] || [ "$LOOPS" != "1" ] || [ "$RETURN_HOME" != "true" ]; then
  SITE_SRC_EFFECTIVE="${RUN_DIR}/site-src"
  rm -rf "$SITE_SRC_EFFECTIVE"
  mkdir -p "$SITE_SRC_EFFECTIVE"
  cp -r "${SITE_SRC}/." "$SITE_SRC_EFFECTIVE/"
  sed -i -E "s/^rviz:.*/rviz: ${RVIZ}/" "${SITE_SRC_EFFECTIVE}/site.yaml"
  sed -i -E "s/^  loops:.*/  loops: ${LOOPS}/" "${SITE_SRC_EFFECTIVE}/site.yaml"
  sed -i -E "s/^  return_home:.*/  return_home: ${RETURN_HOME}/" "${SITE_SRC_EFFECTIVE}/site.yaml"
  echo "[demo] 已按命令行覆盖站点参数: rviz=${RVIZ} loops=${LOOPS} return_home=${RETURN_HOME}"
fi

if [ "$APPLY" = "true" ]; then
  echo "[demo] 2/5 站点落地（apply_site → render_site）：${SITES_DIR}/${SITE_NAME}"
  bash "${WS_ROOT}/deploy/apply_site.sh" \
    --site-dir "$SITE_SRC_EFFECTIVE" --name "$SITE_NAME" --sites-dir "$SITES_DIR"
else
  echo "[demo] 2/5 跳过 apply_site（--no-apply）"
fi

SITE_DIR="${SITES_DIR}/${SITE_NAME}"
if [ ! -f "${SITE_DIR}/nav2_params.yaml" ]; then
  echo "[demo] 站点未就绪：${SITE_DIR}/nav2_params.yaml 不存在" >&2
  exit 3
fi
if [ ! -f "${SITE_DIR}/site.env" ]; then
  # run_site.sh 容忍 site.env 缺失，但验收器要靠它知道站点的到点朝向策略；
  # 缺了就直接说清楚，别等到第 5 步才出一堆莫名的朝向失败。
  echo "[demo] 站点未就绪：${SITE_DIR}/site.env 不存在（重新执行 apply_site.sh，或去掉 --no-apply）" >&2
  exit 3
fi

# ---------------------------------------------------------------- 3. 起假狗
echo "[demo] 3/5 启动闭环假机器狗（/lf/sportmodestate + /utlidar/cloud_deskewed）"
FAKE_ARGS=(--obstacles "$OBSTACLES")
if [ -n "$BATTERY_DROP_AFTER" ]; then
  FAKE_ARGS+=(--battery-drop-after "$BATTERY_DROP_AFTER")
fi
launch_bg fake_go2 python3 "${SCRIPT_DIR}/fake_go2_loop.py" "${FAKE_ARGS[@]}"
sleep 2

# ---------------------------------------------------------------- 4. 起巡检栈
echo "[demo] 4/5 启动巡检栈（run_site.sh ${SITE_NAME}，rviz=${RVIZ}）"
launch_bg patrol_stack bash "${WS_ROOT}/deploy/run_site.sh" "$SITE_NAME"

# ---------------------------------------------------------------- 5. 验收
echo "[demo] 5/5 等待 Nav2 起效并逐点验收（超时 ${TIMEOUT}s）"
VERIFY_ARGS=(
  --waypoints "${SITE_DIR}/waypoints.yaml"
  --loops "$LOOPS"
  --timeout "$TIMEOUT"
  --report "${RUN_DIR}/report.json"
)
# 站点级到点朝向策略：验收器要知道 fixed 之外的点不判朝向（travel/next 的朝向取决于来路）。
# 必须**真正 source 求值**，不能抠原文：site.env 里写的是 `GO2_PATROL_YAW_MODE=${GO2_PATROL_YAW_MODE:-travel}`
# 这种模板形式（为了让命令行能覆盖），sed 拿到的是这串模板字面量而不是 travel，
# 验收器会把它当成一个"非 fixed"的模式，站点级 fixed 的朝向检查就被静默跳过（假通过）。
# 放进子 shell 求值，免得这里的 set -a 污染当前会话；站点侧文件读不到时按 fixed（保守）处理。
SITE_YAW_MODE="$(
  set -a; set +u
  . "${SITE_DIR}/site.env" >/dev/null 2>&1 || true
  printf '%s' "${GO2_PATROL_YAW_MODE:-}"
)"
VERIFY_ARGS+=(--yaw-mode "${SITE_YAW_MODE:-fixed}")
if [ "$RETURN_HOME" = "false" ]; then
  VERIFY_ARGS+=(--no-return-home)
fi

set +e
python3 "${SCRIPT_DIR}/demo_verify.py" "${VERIFY_ARGS[@]}"
VERIFY_RC=$?
set -e

echo
echo "[demo] 日志目录: ${RUN_DIR}"
echo "[demo] 报告文件: ${RUN_DIR}/report.json"
if [ "$VERIFY_RC" -eq 0 ]; then
  echo "[demo] 验收通过 ✅（巡检链路：假数据 → Nav2 导航 → 到点播报 → 收工返航）"
else
  echo "[demo] 验收未通过（退出码 ${VERIFY_RC}），排查顺序：" >&2
  echo "[demo]   1) ${RUN_DIR}/patrol_stack.log 里搜 [ERROR]/[WARN]" >&2
  echo "[demo]   2) ros2 topic hz /scan /odom /amcl_pose 是否都有数据" >&2
  echo "[demo]   3) ros2 topic echo /robot/say 是否播报" >&2
fi
exit "$VERIFY_RC"
