#!/usr/bin/env bash
# OFFLINE 环境：完全不需要 Go2 实机 / Go2 网卡 / DDS 组播网卡。
# 与 scripts/setup.sh（REAL）的区别：
#   1. 不探测、不设置 GO2_IFACE，不 source unitree CycloneDDS 环境；
#   2. RMW 用 rmw_fastrtps_cpp，DDS 通信只走本机（不挑物理网卡）；
#   3. ROS_DOMAIN_ID 隔离，避免与机器上其它 ROS 2 进程互相发现；
#   4. 导出 GO2_PYTHON：conda 的 python3 通常没有 rclpy/pytest，这里强制系统解释器；
#      并把它放到 PATH 最前面，这样 `source ... && python3 -m pytest` 用的是同一个解释器。
#
# 重要：本脚本经常被 `source scripts/setup_offline.sh` 在**交互式终端**里使用。
# 若把 `set -e` 留在调用方 shell 里，之后任何返回非 0 的命令（比如 pytest、
# grep 没匹配）都会直接把终端关掉。所以这里先记下调用方原有的选项，脚本退出
# 前原样恢复（无论成功还是中途出错）。独立执行时（bash setup_offline.sh）也安全。
# 记录调用方原有的 shell 选项（必须在改任何选项之前取）。只恢复本脚本自己改过的
# errexit/nounset/pipefail 三个选项，不动调用方 shell 的其它设置（history 等）。
# 注意：不定义函数、不用 RETURN trap。RETURN trap 会被 /opt/ros/setup.bash 等
# 中间脚本继承，并在它们 return 时触发；Bash 在触发 RETURN trap 期间执行到函数定义/
# 函数返回值时还会把调用栈清掉，导致 "未找到命令" 或恢复逻辑跑在子 shell 里。
__GO2_ORIG_ERREXIT=0; case $- in *e*) __GO2_ORIG_ERREXIT=1 ;; esac
__GO2_ORIG_NOUNSET=0; case $- in *u*) __GO2_ORIG_NOUNSET=1 ;; esac
__GO2_ORIG_PIPEFAIL=0
if [ -o pipefail ]; then __GO2_ORIG_PIPEFAIL=1; fi
# 从这一步开始到文件末尾，任何意外退出（包括 set -e、set -u）都会落到下面的 EXIT
# trap 上完成恢复。正常 source 时不会触发 EXIT（只在本文件 return 前显式调用一次）。
__go2_setup_restore() {
  set +eu
  if [ "$__GO2_ORIG_ERREXIT" = 1 ]; then set -e; fi
  if [ "$__GO2_ORIG_NOUNSET" = 1 ]; then set -u; fi
  if [ "$__GO2_ORIG_PIPEFAIL" = 1 ]; then set -o pipefail; else set +o pipefail; fi
  unset __GO2_ORIG_ERREXIT __GO2_ORIG_NOUNSET __GO2_ORIG_PIPEFAIL
  trap - EXIT 2>/dev/null || true
  unset -f __go2_setup_restore 2>/dev/null || true
  # 清掉 RETURN/EXIT trap 触发时 Bash 临时设置的只读变量，避免泄漏进交互式终端。
  unset -v BASH_LINENO BASH_ARGC BASH_ARGV 2>/dev/null || true
}
# EXIT trap 里调用的必须是本脚本作用域内可见的东西；用 eval 让函数名在触发时才解析。
trap 'if type -t __go2_setup_restore >/dev/null; then __go2_setup_restore; fi' EXIT

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
export PATH="$(dirname "$GO2_PYTHON"):${PATH}"

set +u
source /opt/ros/humble/setup.bash
if [ -f "$GO2_INSTALL_SETUP" ]; then
  source "$GO2_INSTALL_SETUP"
else
  echo "[setup_offline.sh] 未找到 $GO2_INSTALL_SETUP，请先 colcon build" >&2
fi
set -u

# 显式声明"我是离线模式"，供 GUI（tools/patrol_gui）判断该起哪套导航 launch：
#   OFFLINE -> offline_patrol.launch.py（fake_go2 + 静态 map->odom，无 AMCL）
#   REAL    -> patrol.launch.py（真机 driver + AMCL）
# 这样 GUI 不需要去猜，也不会把离线导航错误地导向 REAL/AMCL 链路。
export GO2_OFFLINE_MODE=1
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

# 恢复调用方 shell 选项（-e/-u/pipefail 都不能泄漏到交互式终端），
# 并保证本脚本以 0 退出，避免 `source ... && 下一条命令` 被 -e 误杀。
__go2_setup_restore
true
