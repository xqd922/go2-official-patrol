#!/usr/bin/env bash
set -euo pipefail

# 网卡选择优先级：GO2_IFACE 环境变量 > 默认路由网卡（自动检测）> enp3s0 兜底
IFACE="${GO2_IFACE:-}"
if [ -z "$IFACE" ]; then
  IFACE="$(ip route show default 2>/dev/null | awk '/default/ {print $5; exit}')"
fi
IFACE="${IFACE:-enp3s0}"
echo "[setup.sh] GO2 DDS interface: ${IFACE} （可用 GO2_IFACE=xxx 覆盖）"

set +u
source /opt/ros/humble/setup.bash
source "$HOME/go2_official_ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash"
source "$HOME/go2_official_ws/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
