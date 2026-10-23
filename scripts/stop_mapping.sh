#!/usr/bin/env bash
# 停掉建图/导航相关的一切进程（建图反复重启时用）。
#
# 为什么不能只用 pkill -f "mapping.launch.py"：
#   ros2 launch 被 kill 后，它启动的子进程（slam_toolbox、pointcloud_to_laserscan）
#   会变成孤儿继续跑。实测出现过两个 pointcloud_to_laserscan 同时向 /scan 发包，
#   slam_toolbox 交替收到两份互相矛盾的扫描，地图一直糊、却看不出任何报错。
#   这里按可执行文件路径逐个清，才真的干净。
#
# 安全：本脚本只发 SIGKILL 给本工作空间启动的进程，不碰机器狗。
set -uo pipefail

PATTERNS=(
  "mapping.launch.py"
  "navigation.launch.py"
  "tf2_ros/static_transform_publisher"
  "go2_official_ws/install/go2_patrol_driver/lib"
  "pointcloud_to_laserscan"
  "sync_slam_toolbox"
  "async_slam_toolbox"
  "slam_toolbox"
  "nav2_"
)

echo "[stop] 停止建图/导航进程..."
# 反复重试：ros2 launch 被 kill 时正在 fork 子进程，单轮 pgrep 可能漏掉刚起来的那个。
# 实测漏掉一个 static_transform_publisher 就会让 /tf 里同名节点叠 7 份，
# RViz 看不出异常，但 slam_toolbox 的地图会直接冻住。
#
# 关键细节：不能用 `pgrep -f <关键字>` 直接取 PID。调用本脚本的外层命令
# （例如 `bash -lc './scripts/stop_mapping.sh'`）命令行里也带着这些关键字，
# pgrep 会把调用者自己列进来，kill 掉父 shell 后脚本"静默退出"，
# 看起来像跑完了其实只杀了一半。这里用 ps 取全命令行，并把
# bash/sh/pgrep/grep 这些"搬运工"排除掉，只杀真正的 ROS 进程。
list_targets() {
  ps -eo pid=,args= 2>/dev/null |
    grep -E "mapping\.launch\.py|navigation\.launch\.py|tf2_ros/static_transform_publisher|go2_patrol_driver/lib|pointcloud_to_laserscan|slam_toolbox|nav2_" |
    grep -vE "^ *[0-9]+ +(bash|sh|/bin/bash|/bin/sh|grep|pgrep|ps) " |
    grep -v "stop_mapping.sh" |
    awk '{print $1}'
}

for round in 1 2 3; do
  found=0
  pids="$(list_targets || true)"
  if [ -n "$pids" ]; then
    # shellcheck disable=SC2086
    kill -9 $pids 2>/dev/null && found=1
  fi
  sleep 2
  left="$(list_targets || true)"
  if [ -z "$left" ]; then
    echo "[stop] 已清空。"
    exit 0
  fi
  echo "[stop] 第 ${round} 轮仍有残留，重试..."
done

echo "[stop] 仍有残留：" >&2
ps -eo pid=,args= | grep -E "go2_patrol_driver/lib|pointcloud_to_laserscan|slam_toolbox|static_transform_publisher" | grep -v grep >&2
exit 1
