#!/usr/bin/env python3
"""Offline demo fixture: simulated Unitree Go2 sensor data (no robot needed).

Publishes:
  /lf/sportmodestate       unitree_go/SportModeState   50 Hz (robot spins in place)
  /lf/lowstate             unitree_go/LowState          1 Hz (battery 87%)
  /utlidar/cloud_deskewed  sensor_msgs/PointCloud2     10 Hz (room walls + box)
"""
import math
import random
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from unitree_go.msg import LowState, SportModeState


def build_room():
    """Room x in [-3,3], y in [-4,4], plus a 0.5m box at (1.5, 1.5)."""
    pts = []
    x = -3.0
    while x <= 3.0001:
        pts.append((x, -4.0, 0.0))
        pts.append((x, 4.0, 0.0))
        x += 0.05
    y = -4.0
    while y <= 4.0001:
        pts.append((-3.0, y, 0.0))
        pts.append((3.0, y, 0.0))
        y += 0.05
    x = 1.25
    while x <= 1.7501:
        pts.append((x, 1.25, 0.0))
        pts.append((x, 1.75, 0.0))
        x += 0.05
    y = 1.25
    while y <= 1.7501:
        pts.append((1.25, y, 0.0))
        pts.append((1.75, y, 0.0))
        y += 0.05
    return pts


def make_cloud(frame_id, stamp, points):
    cloud = PointCloud2()
    cloud.header.stamp = stamp
    cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.point_step = 12
    cloud.row_step = 12 * len(points)
    cloud.is_bigendian = False
    cloud.is_dense = True
    buf = bytearray()
    for px, py, pz in points:
        buf += struct.pack("<fff", px, py, pz)
    cloud.data = bytes(buf)
    return cloud


class FakeGo2(Node):
    def __init__(self):
        super().__init__("fake_go2_data")
        self.omega = 0.35
        self.t0 = self.get_clock().now()
        self.room = build_room()
        self.pub_state = self.create_publisher(SportModeState, "/lf/sportmodestate", 10)
        self.pub_low = self.create_publisher(LowState, "/lf/lowstate", 10)
        self.pub_cloud = self.create_publisher(PointCloud2, "/utlidar/cloud_deskewed", 10)
        self.create_timer(1.0 / 50.0, self.on_state)
        self.create_timer(1.0, self.on_low)
        self.create_timer(1.0 / 10.0, self.on_cloud)
        self.get_logger().info("fake Go2 data started")

    def yaw(self):
        return self.omega * (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def on_state(self):
        yaw = self.yaw()
        msg = SportModeState()
        msg.imu_state.quaternion = [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]
        msg.position = [0.0, 0.0, 0.0]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.yaw_speed = self.omega
        self.pub_state.publish(msg)

    def on_low(self):
        msg = LowState()
        msg.bms_state.soc = 87
        self.pub_low.publish(msg)

    def on_cloud(self):
        yaw = self.yaw()
        c, s = math.cos(-yaw), math.sin(-yaw)
        pts = []
        for x, y, z in self.room:
            nx = c * x - s * y + random.uniform(-0.01, 0.01)
            ny = s * x + c * y + random.uniform(-0.01, 0.01)
            pts.append((nx, ny, z))
        self.pub_cloud.publish(make_cloud("utlidar_lidar", self.get_clock().now().to_msg(), pts))


def main():
    rclpy.init()
    node = FakeGo2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
