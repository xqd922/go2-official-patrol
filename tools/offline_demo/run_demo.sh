#!/usr/bin/env bash
# 离线演示（无需机器狗）：假数据 + 建图栈 + RViz
#
# 用法:
#   tools/offline_demo/run_demo.sh start   # 启动演示
#   tools/offline_demo/run_demo.sh stop    # 停止演示
set -euo pipefail

WS="$HOME/go2_official_ws"
DIR="$(cd "$(dirname "$0")" && pwd)"

start() {
  # 1) 假数据：sportmodestate 50Hz（原地旋转）+ lowstate 1Hz + 点云 10Hz
  setsid nohup bash -c "source $WS/scripts/setup.sh && exec python3 $DIR/fake_go2.py" >/tmp/demo_fake.log 2>&1 </dev/null &
  # 2) 演示补链 TF：base_footprint -> base_link（真机上若机器人自身已发布，请删掉这一行）
  setsid nohup bash -c "source $WS/scripts/setup.sh && exec ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link" >/tmp/demo_tf.log 2>&1 </dev/null &
  # 3) 建图栈（感知 + 驱动桥 + slam_toolbox）
  setsid nohup bash -c "source $WS/scripts/setup.sh && exec ros2 launch go2_patrol_bringup mapping.launch.py" >/tmp/demo_mapping.log 2>&1 </dev/null &
  sleep 8
  # 4) RViz
  setsid nohup bash -c "source $WS/scripts/setup.sh && exec rviz2 -d $DIR/demo.rviz" >/tmp/demo_rviz.log 2>&1 </dev/null &
  sleep 3
  echo "演示已启动：RViz 应显示房间地图 / 扫描点 / 里程计。日志在 /tmp/demo_*.log"
}

stop() {
  pkill -f '[f]ake_go2' || true
  pkill -f '[m]apping.launch' || true
  pkill -f '[s]lam_toolbox' || true
  pkill -f '[p]ointcloud_to_laserscan' || true
  pkill -f 'go2_patrol_driver' || true
  pkill -f 'base_footprint base_link' || true
  pkill -f '[r]viz2' || true
  echo "演示进程已停止"
}

case "${1:-start}" in
  start) start ;;
  stop) stop ;;
  *) echo "用法: $0 [start|stop]"; exit 1 ;;
esac
