#!/usr/bin/env bash
# Go2 巡检工作空间环境定义（开发机与生产机共用）。
#
# 用法：
#   source deploy/go2_env.sh       # 直接使用（生产机 systemd 走 run_site.sh）
#   source scripts/setup.sh        # 开发机惯用入口，内部转调本文件
#
# 可覆盖的环境变量：
#   GO2_WS          工作空间根（默认自动推导为本文件所在目录的上一级）
#   GO2_SITES       站点资产根（默认 ${GO2_WS}/sites；生产机由 /etc/go2/site.env 指定）
#   GO2_IFACE       接机器狗的网卡（默认取默认路由网卡，兜底 enp3s0）
#   GO2_DOG_IP      机器狗地址（默认 192.168.123.161）
#   GO2_ENV_QUIET   设为 1 时不打印环境摘要
#
# 注意：ROS 2 的 setup.bash 不兼容 set -u，本文件内部会临时关闭它。

_go2_env_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export GO2_WS="${GO2_WS:-$(cd "${_go2_env_dir}/.." && pwd)}"
export GO2_SITES="${GO2_SITES:-${GO2_WS}/sites}"
unset _go2_env_dir

# 网卡选择优先级：GO2_IFACE > 默认路由网卡 > enp3s0 兜底
if [ -z "${GO2_IFACE:-}" ] && command -v ip >/dev/null 2>&1; then
  # 只取第一条默认路由的网卡名。awk 里刻意不写 exit：一旦提前退出，
  # ip 会在还有输出时收到 SIGPIPE，在 set -e + pipefail 下会直接中断整个脚本
  # （双默认路由的机器上 100% 复现，服务会"无输出秒退"）。
  GO2_IFACE="$(ip route show default 2>/dev/null | awk '/default/ {if (!found) {print $5; found=1}}')" ||
    GO2_IFACE=""
fi
export GO2_IFACE="${GO2_IFACE:-enp3s0}"
export GO2_DOG_IP="${GO2_DOG_IP:-192.168.123.161}"

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[go2_env] 警告：未找到 /opt/ros/humble/setup.bash，请先安装 ROS 2 Humble" >&2
fi

if [ -f "${GO2_WS}/src/unitree_ros2/cyclonedds_ws/install/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source "${GO2_WS}/src/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi

if [ -f "${GO2_WS}/install/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source "${GO2_WS}/install/setup.bash"
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${GO2_IFACE}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"

if [ "${GO2_ENV_QUIET:-0}" != "1" ]; then
  echo "[go2_env] WS=${GO2_WS} SITES=${GO2_SITES} IFACE=${GO2_IFACE} DOG=${GO2_DOG_IP}（GO2_IFACE=xxx 可覆盖）"
fi
