#!/usr/bin/env python3
"""CLI：把机器人当前位置记成巡检点（打印 waypoints.yaml 片段）。

关键修正：必须取 **map 坐标系** 的位姿，而不是 /odom。

  odom_bridge 的 /odom 是 odom 系里程计（odom->base_footprint），odom 原点是
  "上电时刻的机器人位置"，会漂移、且通常与地图原点不重合。若直接把 /odom 的
  x/y 当成地图坐标写进 waypoints.yaml，记录出来的航点会整体偏移（差一个
  map->odom）。正确做法是查 TF：map -> base_footprint，这一段的
  map->odom 由定位模块提供（REAL=AMCL，建图=slam_toolbox，OFFLINE=静态）。

本文件的换算与 GUI（tools/patrol_gui）共用同一个实现
go2_patrol_patrol.map_pose，保证两个入口记录出的坐标完全一致。
"""
import argparse
import sys

import rclpy
from rclpy.node import Node


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("name")
    parser.add_argument("--dwell", type=float, default=4.0)
    parser.add_argument("--home", action="store_true")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="等待 map->base_footprint TF 的超时（秒）")
    options = parser.parse_args(args)

    from go2_patrol_patrol.map_pose import lookup_map_pose

    rclpy.init()
    node = Node("record_waypoint")
    try:
        pose = lookup_map_pose(node, timeout_sec=options.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if pose is None:
        # 明确失败，而不是打印一个可能错误（odom 系）的坐标。
        print(
            f"[record_waypoint] 取不到 map 坐标系位姿（map->base_footprint）。\n"
            f"  请确认定位已就绪：REAL 模式需 AMCL/导航在跑，建图模式需 "
            f"slam_toolbox 在跑。",
            file=sys.stderr,
        )
        return 1

    x, y, yaw = pose
    print(
        f"  - name: {options.name}\n"
        f"    x: {x:.3f}\n"
        f"    y: {y:.3f}\n"
        f"    yaw: {yaw:.3f}\n"
        f"    dwell: {options.dwell}\n"
        f"    speech: {options.name}"
        + ("\n    home: true" if options.home else "")
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
