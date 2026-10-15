#!/usr/bin/env bash
# 开发机用：把某个 ref 打成离线交付包（含官方 unitree_ros2 源码），供无网生产机安装。
#   deploy/make_release_bundle.sh <ref> [输出目录]
#
# 产物 go2-official-ws-<ref>-bundle.tar.gz 结构：
#   仓库文件…                 指定 ref 的完整代码
#   offline/unitree_ros2.tar.gz  官方 vcs 依赖源码（本机已 vcs import 时打包）
#   offline/manifest.txt         版本/时间/生成机/校验和
#
# 需要 bash 环境（WSL / Git Bash）。在已执行过 vcs import 的副本里运行可带上离线依赖。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

REF="${1:-}"
OUT_DIR="${2:-${WS_ROOT}/dist}"

if [ -z "$REF" ]; then
  echo "用法: deploy/make_release_bundle.sh <ref> [输出目录]" >&2
  exit 2
fi

cd "$WS_ROOT"
if ! SHA="$(git rev-parse --verify "${REF}^{commit}")"; then
  echo "找不到 ref: ${REF}" >&2
  exit 2
fi
REF_SAFE="${REF//\//_}"
TOP="go2-official-ws-${REF_SAFE}"
TMP="$(mktemp -d)"
cleanup() { rm -rf -- "$TMP"; }
trap cleanup EXIT

echo "[bundle] ref=${REF} sha=${SHA}"
mkdir -p "${TMP}/${TOP}/offline" "$OUT_DIR"

# --- 1. 仓库内容（git archive 只含已提交文件，不带 build/install/log） ---
git archive "$SHA" | tar -x -C "${TMP}/${TOP}"

# --- 2. 离线依赖：官方 unitree_ros2（vcs 拉取的那份） ---
if [ -d "${WS_ROOT}/src/unitree_ros2" ]; then
  echo "[bundle] 打包官方依赖 src/unitree_ros2（排除 build/install/log）"
  tar -czf "${TMP}/${TOP}/offline/unitree_ros2.tar.gz" \
    -C "${WS_ROOT}/src" \
    --exclude='unitree_ros2/*/build' \
    --exclude='unitree_ros2/*/install' \
    --exclude='unitree_ros2/*/log' \
    unitree_ros2
else
  echo "[bundle] 警告：本机没有 src/unitree_ros2，交付包将缺离线依赖（目标机需联网 vcs import）" >&2
fi

# --- 3. 交付清单 ---
{
  echo "交付包: ${TOP}"
  echo "仓库  : $(git remote get-url origin 2>/dev/null || echo '(无 origin)')"
  echo "ref   : ${REF}"
  echo "commit: ${SHA}"
  echo "时间  : $(date -Iseconds)"
  echo "生成于: $(hostname)"
  if [ -f "${TMP}/${TOP}/offline/unitree_ros2.tar.gz" ]; then
    echo "离线依赖: 含 src/unitree_ros2.tar.gz"
  else
    echo "离线依赖: 缺失"
  fi
  echo "文件校验(最多 200 个):"
  # head 提前退出会让上游收到 SIGPIPE，pipefail 下整条脚本会失败，所以吞掉状态
  (cd "${TMP}/${TOP}" && find . -type f -not -path './offline/*' -print0 |
    sort -z | xargs -0 sha256sum | head -200) || true
} >"${TMP}/${TOP}/offline/manifest.txt"

# --- 4. 打包 ---
BUNDLE="${OUT_DIR}/${TOP}-bundle.tar.gz"
tar -czf "$BUNDLE" -C "$TMP" "$TOP"
echo "[bundle] 产物  : ${BUNDLE}"
echo "[bundle] 大小  : $(du -h "$BUNDLE" | cut -f1)"
echo "[bundle] sha256: $(sha256sum "$BUNDLE" | cut -d' ' -f1)"
echo "[bundle] 目标机: sudo deploy/install.sh --bundle ${TOP}-bundle.tar.gz --ref ${REF} --no-apt"
