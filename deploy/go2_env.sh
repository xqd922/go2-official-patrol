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

# 机器级配置（生产机由 install.sh 写入 /etc/go2/site.env，指定 GO2_SITES / GO2_IFACE）。
# 必须在下面几条默认值之前读，否则手工执行 preflight.sh 之类会落到
# <版本目录>/sites，而 systemd 找的是 /opt/go2/sites——两边看到的站点不是同一份。
#
# 但**调用方显式给的值优先**：apply_site.sh 会 `GO2_SITES=... bash preflight.sh`，
# run_patrol_demo.sh 也会先 export GO2_SITES 再 source 本文件。机器级文件里的
# 裸赋值会把它们顶掉（preflight 去校验另一个目录，离线演示直接 exit 3），
# 所以先记下调用方的值，读完文件再恢复。
# 用 `${VAR+x}` 而不是 `-n` 判"给过没有"：`GO2_SITES= deploy/run_site.sh demo`
# 这种"显式清空"也是合法意图，只认非空的话它会静默被机器级值顶掉。
if [ -f /etc/go2/site.env ]; then
  _go2_sites_set="${GO2_SITES+x}"
  _go2_caller_sites="${GO2_SITES:-}"
  _go2_iface_set="${GO2_IFACE+x}"
  _go2_caller_iface="${GO2_IFACE:-}"
  # shellcheck disable=SC1091
  source /etc/go2/site.env
  if [ -n "$_go2_sites_set" ]; then export GO2_SITES="$_go2_caller_sites"; fi
  if [ -n "$_go2_iface_set" ]; then export GO2_IFACE="$_go2_caller_iface"; fi
fi
unset _go2_caller_sites _go2_sites_set _go2_caller_iface _go2_iface_set

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
