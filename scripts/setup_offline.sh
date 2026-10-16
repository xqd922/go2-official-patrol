#!/usr/bin/env bash
# OFFLINE 环境：完全不需要 Go2 实机 / Go2 网卡 / DDS 组播网卡。
# 与 scripts/setup.sh（REAL）的区别：
#   1. 不探测、不设置 GO2_IFACE，不 source unitree CycloneDDS 环境；
#   2. RMW 用 rmw_fastrtps_cpp，DDS 通信只走本机（不挑物理网卡）；
#   3. ROS_DOMAIN_ID 隔离，避免与机器上其它 ROS 2 进程互相发现；
#   4. 导出 GO2_PYTHON：conda 的 python3 通常没有 rclpy，这里强制系统解释器。
set -euo pipefail
source "$(dirname "$(realpath "${BASH_SOURCE[0]}")")/repo_root.sh"

# 选一个能 import rclpy 的解释器（conda python 通常不行）
if [ -z "${GO2_PYTHON:-}" ]; then
  for cand in /usr/bin/python3 python3; do
    if command -v "$cand" >/dev/null 2>&1 && "$cand" -c "import rclpy" >/dev/null 2>&1; then
      GO2_PYTHON="$(command -v "$cand")"; break
    fi
  done
  GO2_PYTHON="${GO2_PYTHON:-/usr/bin/python3}"
fi
export GO2_PYTHON

set +u
source /opt/ros/humble/setup.bash
if [ -f "$GO2_INSTALL_SETUP" ]; then
  source "$GO2_INSTALL_SETUP"
else
  echo "[setup_offline.sh] 未找到 $GO2_INSTALL_SETUP，请先 colcon build" >&2
fi
set -u

export RMW_IMPLEMENTATION="${GO2_OFFLINE_RMW:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${GO2_OFFLINE_DOMAIN_ID:-42}"
# 真正的"只走本机"：ROS 的 ros_environment 钩子会把 ROS_LOCALHOST_ONLY 默认设成 0，
# 此时 FastRTPS 会在**所有**网卡（含未接线的 enp49s0、WiFi）上做组播发现。真实机
# 实测的 Nav2 bond 心跳丢失（"Have not received a heartbeat ... 4000 ms"）与
# costmap "Message Filter dropping message ... timestamp earlier than all the data in
# the transform cache" 都是 DDS 投递被拖慢/丢包的典型症状，根因就是离线链路其实并
# 没有限制在本机。这里显式置 1（FastRTPS 会把接口白名单限制为 127.0.0.1），
# 使离线通信只走 loopback/共享内存，不依赖任何物理网卡，也不会被别的网卡拖慢。
# 需要临时排查时可用 GO2_OFFLINE_LOCALHOST_ONLY=0 覆盖。
export ROS_LOCALHOST_ONLY="${GO2_OFFLINE_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/go2_roslog}"
export ROS_HOME="${ROS_HOME:-/tmp/go2_roshome}"
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
echo "[setup_offline.sh] OFFLINE 模式：不需要 Go2 网卡，RMW=$RMW_IMPLEMENTATION domain=$ROS_DOMAIN_ID localhost_only=$ROS_LOCALHOST_ONLY"
echo "[setup_offline.sh] 解释器: $GO2_PYTHON"

