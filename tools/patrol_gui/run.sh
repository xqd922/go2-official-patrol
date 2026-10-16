#!/usr/bin/env bash
# 打开"Go2 建图与巡检点工具"。
#
# 用法：
#   bash tools/patrol_gui/run.sh              # REAL：真机链路（setup.sh，AMCL）
#   bash tools/patrol_gui/run.sh --offline    # OFFLINE：无 Go2（setup_offline.sh）
#
# 环境由现有脚本决定，本脚本不自己拼 ROS 环境、不设 GO2_IFACE：
#   * REAL    走 scripts/setup.sh      -> CycloneDDS + 真机网卡 + 真机驱动
#   * OFFLINE 走 scripts/setup_offline.sh -> FastRTPS + localhost + domain 42
# 因此 OFFLINE/REAL 的隔离规则与项目其它入口完全一致。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
WS="$(cd "$HERE/../.." && pwd -P)"

MODE="real"
if [ "${1:-}" = "--offline" ]; then
  MODE="offline"
  shift
fi

if [ "$MODE" = "offline" ]; then
  # shellcheck source=/dev/null
  source "$WS/scripts/setup_offline.sh"
else
  # shellcheck source=/dev/null
  source "$WS/scripts/setup.sh"
fi

# 选一个同时具备 rclpy 与 tkinter 的解释器。conda 的 python 常常没有 rclpy，
# 系统 /usr/bin/python3 才是 ROS 2 Humble 的正式解释器。
PY="${GO2_PYTHON:-}"
if [ -z "$PY" ] || ! "$PY" -c "import rclpy, tkinter" >/dev/null 2>&1; then
  for cand in /usr/bin/python3 python3; do
    if command -v "$cand" >/dev/null 2>&1 \
       && "$cand" -c "import rclpy, tkinter" >/dev/null 2>&1; then
      PY="$(command -v "$cand")"
      break
    fi
  done
fi
if [ -z "$PY" ] || ! "$PY" -c "import rclpy, tkinter" >/dev/null 2>&1; then
  echo "[run.sh] 找不到同时具备 rclpy 与 tkinter 的 Python。" >&2
  echo "         请安装 python3-tk 后重试：sudo apt install python3-tk" >&2
  exit 1
fi

echo "[run.sh] 模式=$MODE 解释器=$PY"
echo "[run.sh] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<未设置>}"

# tools/ 加到 PYTHONPATH，使 `python -m patrol_gui.app` 能找到包。
export PYTHONPATH="$WS/tools:${PYTHONPATH:-}"
cd "$WS"
exec "$PY" -m patrol_gui.app "$@"
