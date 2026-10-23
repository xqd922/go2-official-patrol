#!/usr/bin/env python3
"""统计 /scan 的有效回波比例与角度覆盖，用来判断点云/高度窗口是否合适。

用法：
    python3 tools/probe_scan.py
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = Node("probe_scan")
    received = {}
    node.create_subscription(
        LaserScan, "/scan", lambda m: received.setdefault("scan", m), qos_profile_sensor_data
    )

    deadline = time.time() + 20
    while time.time() < deadline and "scan" not in received:
        rclpy.spin_once(node, timeout_sec=0.2)

    if "scan" not in received:
        print("超时未收到 /scan")
        raise SystemExit(1)

    scan = received["scan"]
    ranges = np.array(scan.ranges, dtype=np.float32)
    valid = np.isfinite(ranges)
    count = int(valid.sum())

    print(f"frame_id = {scan.header.frame_id}")
    print(f"角度格 = {len(ranges)}，有效 = {count}（{100.0 * count / len(ranges):.1f}%）")
    print(f"angle_increment = {scan.angle_increment:.4f} rad "
          f"（{math.degrees(scan.angle_increment):.2f}°/格）")
    print(f"range = [{scan.range_min:.2f}, {scan.range_max:.2f}] m")

    if count:
        values = ranges[valid]
        print(f"距离 = min {values.min():.2f} / 中位 {np.median(values):.2f} / max {values.max():.2f}")
        half = len(ranges) // 2
        front = int(valid[half - 90 : half + 90].sum())   # ±0.78 rad 约 ±45°
        print(f"正前方 ±45° 有效 = {front} / 180")

        # 按 30° 分区看覆盖，找出“看不见”的方向
        sectors = 12
        per = len(ranges) // sectors
        for i in range(sectors):
            seg = valid[i * per : (i + 1) * per]
            angle = math.degrees(scan.angle_min + (i + 0.5) * per * scan.angle_increment)
            print(f"  {angle:+7.1f}° : {int(seg.sum()):3d}/{per}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
