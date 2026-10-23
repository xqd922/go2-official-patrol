#!/usr/bin/env python3
"""诊断机载点云：点数、z 分布、以及 pointcloud_to_laserscan 的高度窗口命中率。

用法：
    python3 tools/probe_cloud.py
"""
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def decode_xyzi(msg: PointCloud2) -> np.ndarray:
    """按 PointField 描述解码出 (N, 4) 的 x/y/z/intensity。"""
    names = [f.name for f in msg.fields]
    wanted = [n for n in ("x", "y", "z") if n in names]
    if len(wanted) != 3:
        raise SystemExit(f"点云缺少 x/y/z 字段，实际字段: {names}")

    # 用 itemsize + offsets 描述整条 point_step，未命名的尾部字节自动成为 padding，
    # 这样 buffer 长度一定对得上（Go2 的 point_step=32，但字段只到 offset 16）。
    dtype = np.dtype(
        {
            "names": wanted,
            "formats": [np.float32] * len(wanted),
            "offsets": [next(f.offset for f in msg.fields if f.name == n) for n in wanted],
            "itemsize": msg.point_step,
        }
    )
    raw = np.frombuffer(msg.data, dtype=dtype)
    return np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float32)


def main():
    rclpy.init()
    node = Node("probe_cloud")
    received = {}
    node.create_subscription(
        PointCloud2,
        "/utlidar/cloud_deskewed",
        lambda m: received.setdefault("cloud", m),
        qos_profile_sensor_data,
    )

    import time

    deadline = time.time() + 20
    while time.time() < deadline and "cloud" not in received:
        rclpy.spin_once(node, timeout_sec=0.2)

    if "cloud" not in received:
        print("超时未收到点云")
        raise SystemExit(1)

    msg = received["cloud"]
    points = decode_xyzi(msg)
    total = len(points)
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    z = points[:, 2]

    print(f"frame_id = {msg.header.frame_id}")
    print(f"点数 = {total}（有效 {len(points)}）")
    print(f"x 范围 = [{points[:,0].min():.2f}, {points[:,0].max():.2f}]")
    print(f"y 范围 = [{points[:,1].min():.2f}, {points[:,1].max():.2f}]")
    print(f"z 范围 = [{z.min():.2f}, {z.max():.2f}]")
    print(f"z 分位 = 5%:{np.percentile(z,5):.2f} 25%:{np.percentile(z,25):.2f} "
          f"50%:{np.percentile(z,50):.2f} 75%:{np.percentile(z,75):.2f} 95%:{np.percentile(z,95):.2f}")

    hist, edges = np.histogram(z, bins=[-0.5, -0.2, -0.05, 0.0, 0.05, 0.2, 0.5, 1.0, 2.0])
    print("z 直方图:")
    for i, count in enumerate(hist):
        print(f"  [{edges[i]:+.2f}, {edges[i+1]:+.2f}) : {count:6d}")

    r = np.hypot(points[:, 0], points[:, 1])
    print(f"水平距离分位 50%:{np.percentile(r,50):.2f} 90%:{np.percentile(r,90):.2f} "
          f"99%:{np.percentile(r,99):.2f}")

    for lo, hi in ((0.05, 1.20), (-0.20, 1.20), (0.0, 2.0)):
        inside = int(((z >= lo) & (z <= hi)).sum())
        print(f"高度窗口 [{lo}, {hi}] 命中 {inside} 点 "
              f"({100.0 * inside / max(1, len(points)):.1f}%)")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
