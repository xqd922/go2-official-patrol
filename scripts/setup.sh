#!/usr/bin/env bash
set -euo pipefail

# 工作空间根目录由 scripts/repo_root.sh 按脚本自身位置推导（兼容任意仓库目录名）。
# shellcheck source=scripts/repo_root.sh
source "$(dirname "$(realpath "${BASH_SOURCE[0]}")")/repo_root.sh"

# 网卡选择优先级：GO2_IFACE 环境变量 > 默认路由网卡（自动检测）> enp3s0 兜底
IFACE="${GO2_IFACE:-}"
if [ -z "$IFACE" ]; then
  IFACE="$(ip route show default 2>/dev/null | awk '/default/ {print $5; exit}')"
fi
IFACE="${IFACE:-enp3s0}"
echo "[setup.sh] GO2 DDS interface: ${IFACE} （可用 GO2_IFACE=xxx 覆盖）"

set +u
source /opt/ros/humble/setup.bash
if [ -f "$GO2_CYCLONEDDS_SETUP" ]; then
  source "$GO2_CYCLONEDDS_SETUP"
else
  echo "[setup.sh] 未找到 $GO2_CYCLONEDDS_SETUP，跳过 unitree_ros2 CycloneDDS 环境（先运行 scripts/bootstrap.sh）" >&2
fi
if [ -f "$GO2_INSTALL_SETUP" ]; then
  source "$GO2_INSTALL_SETUP"
else
  echo "[setup.sh] 未找到 $GO2_INSTALL_SETUP，跳过本工作空间 install 环境（先运行 colcon build）" >&2
fi
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
