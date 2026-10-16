#!/usr/bin/env bash
# 离线演示（无需机器狗）：假数据 + 建图栈 + RViz
#
# 用法:
#   tools/offline_demo/run_demo.sh start          # 建图演示（slam_toolbox + RViz）
#   tools/offline_demo/run_demo.sh patrol [map]   # 离线定点巡检（Nav2 + patrol_manager）
#   tools/offline_demo/run_demo.sh stop           # 停止全部
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# 工作空间根目录：默认从脚本自身位置推导（tools/offline_demo -> 仓库根），
# 也支持用 GO2_WS_ROOT 覆盖，不依赖用户名或仓库目录名。
WS="${GO2_WS_ROOT:-$(cd "${DIR}/../.." && pwd -P)}"

if [ ! -f "$WS/scripts/setup.sh" ]; then
  echo "找不到工作空间环境脚本 $WS/scripts/setup.sh；请设置 GO2_WS_ROOT 指向工作空间根目录" >&2
  exit 1
fi

start() {
  # 1) 假数据：sportmodestate 50Hz（原地旋转）+ lowstate 1Hz + 点云 10Hz
  setsid nohup bash -c "source $WS/scripts/setup.sh && exec /usr/bin/python3 $DIR/fake_go2.py" >/tmp/demo_fake.log 2>&1 </dev/null &
  # 2) 演示补链 TF：base_footprint -> base_link（真机上若机器人自身已发布，请删掉这一行）
  setsid nohup bash -c "source $WS/scripts/setup.sh && exec /usr/bin/python3 /opt/ros/humble/bin/ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link" >/tmp/demo_tf.log 2>&1 </dev/null &
  # 3) 建图栈（感知 + 驱动桥 + slam_toolbox）
  setsid nohup bash -c "source $WS/scripts/setup.sh && exec /usr/bin/python3 /opt/ros/humble/bin/ros2 launch go2_patrol_bringup mapping.launch.py" >/tmp/demo_mapping.log 2>&1 </dev/null &
  sleep 8
  # 4) RViz
  setsid nohup bash -c "source $WS/scripts/setup.sh && exec rviz2 -d $DIR/demo.rviz" >/tmp/demo_rviz.log 2>&1 </dev/null &
  sleep 3
  echo "演示已启动：RViz 应显示房间地图 / 扫描点 / 里程计。日志在 /tmp/demo_*.log"
}

# 离线定点巡检：Nav2 定位导航 + patrol_manager + fake_go2 运动模型。
# 关键：通过 offline_patrol.launch.py 启动，twist_bridge 被禁用运动输出，
# 不会向真实 Go2 的 /api/sport/request 发命令。
patrol() {
  # 默认地图：优先 maps/map.yaml，否则挑第一张已保存的 <名字>.yaml。
  # save_map.sh 生成的是 <名字>.yaml，并不会改名成 map.yaml，所以不能只认 map.yaml。
  local map="${1:-}"
  if [ -z "$map" ]; then
    local dir="$WS/src/go2_patrol_bringup/maps"
    if [ -s "$dir/map.yaml" ]; then
      map="$dir/map.yaml"
    else
      map="$(find "$dir" -maxdepth 1 -name '*.yaml' 2>/dev/null | sort | head -1)"
    fi
  fi
  if [ -z "$map" ] || [ ! -s "$map" ]; then
    echo "找不到地图；请先建图并执行 scripts/save_map.sh <名字>" >&2
    exit 1
  fi
  setsid nohup bash -c "source $WS/scripts/setup.sh && exec /usr/bin/python3 /opt/ros/humble/bin/ros2 launch go2_patrol_bringup offline_patrol.launch.py map:=$map" >/tmp/demo_patrol.log 2>&1 </dev/null &
  sleep 20
  echo "离线巡检已启动（map=$map）。日志：/tmp/demo_patrol.log"
  echo "启动巡检： ros2 service call /patrol/start std_srvs/srv/Trigger"
}

stop() {
  pkill -f '[f]ake_go2' || true
  pkill -f '[m]apping.launch' || true
  pkill -f '[o]ffline_patrol.launch' || true
  pkill -f '[s]lam_toolbox' || true
  pkill -f '[p]ointcloud_to_laserscan' || true
  pkill -f 'go2_patrol_driver' || true
  pkill -f 'base_footprint base_link' || true
  pkill -f '[r]viz2' || true
  echo "演示进程已停止"
}

case "${1:-start}" in
  start) start ;;
  patrol) shift; patrol "${@:-}" ;;
  stop) stop ;;
  *) echo "用法: $0 [start|patrol [map.yaml]|stop]"; exit 1 ;;
esac
