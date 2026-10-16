"""Go2 可视化建图与巡检点设置工具。

模块划分（全部无 ROS 依赖即可 import，方便单测）：
  proc.py            进程/进程组/domain 工具
  launch_manager.py  启动与停止现有 launch、调用 save_map.sh
  ros_bridge.py      后台 spin 的 ROS 节点（map 位姿 / 巡检服务）
  waypoint_store.py  航点增删查（复用 go2_patrol_patrol.waypoints）
  cleanup.py         退出时的 domain 维度兜底清理
  gui.py             tkinter 界面
  app.py             入口
"""
