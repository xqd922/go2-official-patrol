#!/usr/bin/env python3
"""Offline demo fixture: simulated Unitree Go2 sensor data (no robot needed).

Publishes:
  /lf/sportmodestate       unitree_go/SportModeState   50 Hz (robot pose from /cmd_vel)
  /lf/lowstate             unitree_go/LowState          1 Hz (battery 87%)
  /utlidar/cloud_deskewed  sensor_msgs/PointCloud2     10 Hz (room walls + box)

Subscribes:
  /cmd_vel                 geometry_msgs/Twist         drives a 2D unicycle model

This is the offline-only stand-in for the Go2 driver chain. It never talks to the
real robot: it only publishes simulated sensor data and integrates /cmd_vel.
"""
import math
import random
import struct

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from unitree_go.msg import LowState, SportModeState


def build_room():
    """3D room: walls at x=+-3, y=+-4 (z 0~2m) plus a 0.5m box at (1.5, 1.5).

    Mimics a real L1 4D LiDAR sweep: vertical wall surfaces covered with
    multiple horizontal scan layers, so RViz shows a genuine 3D point cloud
    instead of a single flat ring.
    """
    pts = []
    for z in [0.1, 0.4, 0.8, 1.2, 1.6, 2.0]:
        # two long walls (y = +-4)
        x = -3.0
        while x <= 3.0001:
            pts.append((x, -4.0, z))
            pts.append((x, 4.0, z))
            x += 0.06
        # two short walls (x = +-3)
        y = -4.0
        while y <= 4.0001:
            pts.append((-3.0, y, z))
            pts.append((3.0, y, z))
            y += 0.06
        # box (0.5 x 0.5 x 0.5 m at (1.5, 1.5))
        if z <= 0.5:
            bx = 1.25
            while bx <= 1.7501:
                pts.append((bx, 1.25, z))
                pts.append((bx, 1.75, z))
                bx += 0.06
            by = 1.25
            while by <= 1.7501:
                pts.append((1.25, by, z))
                pts.append((1.75, by, z))
                by += 0.06
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
        # 2D unicycle state, integrated from /cmd_vel.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_cmd_time = None
        self.room = build_room()
        self.pub_state = self.create_publisher(SportModeState, "/lf/sportmodestate", 10)
        self.pub_low = self.create_publisher(LowState, "/lf/lowstate", 10)
        self.pub_cloud = self.create_publisher(PointCloud2, "/utlidar/cloud_deskewed", 10)
        # Offline: accept the Nav2 velocity command and integrate it below.
        # 真机上这一步由 go2_patrol_driver 负责，本节点只在离线演示中使用。
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_timer(1.0 / 50.0, self.on_state)
        self.create_timer(1.0, self.on_low)
        self.create_timer(1.0 / 10.0, self.on_cloud)
        self.get_logger().info("fake Go2 data started (offline motion model on /cmd_vel)")

    def on_cmd_vel(self, twist: Twist):
        self.vx = float(twist.linear.x)
        self.vy = float(twist.linear.y)
        self.wz = float(twist.angular.z)

    def integrate(self):
        """Advance the unicycle model by the elapsed time since the last step."""
        now = self.get_clock().now()
        if self.last_cmd_time is None:
            self.last_cmd_time = now
            return
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        self.last_cmd_time = now
        # 保护：定时器抖动或暂停可能产生异常 dt。
        if dt <= 0.0 or dt > 0.5:
            return
        # 标准二维独轮车模型：先按当前朝向平移，再更新朝向。
        # Nav2/Go2 当前配置只下发 linear.x 与 angular.z；vy 仍一并累积，
        # 以便将来启用全向移动时无需改本节点。
        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        self.yaw += self.wz * dt

    def on_state(self):
        self.integrate()
        msg = SportModeState()
        msg.imu_state.quaternion = [0.0, 0.0, math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)]
        msg.position = [self.x, self.y, 0.0]
        msg.velocity = [self.vx, self.vy, 0.0]
        msg.yaw_speed = self.wz
        self.pub_state.publish(msg)

    def on_low(self):
        msg = LowState()
        msg.bms_state.soc = 87
        self.pub_low.publish(msg)

    def on_cloud(self):
        # 把房间点从 map/odom 系旋转回机器人本体坐标系（雷达 frame）。
        # 先做平移（减去机器人当前位置），再按 -yaw 旋转。
        yaw = self.yaw
        c, s = math.cos(-yaw), math.sin(-yaw)
        pts = []
        for x, y, z in self.room:
            dx = x - self.x
            dy = y - self.y
            nx = c * dx - s * dy + random.uniform(-0.01, 0.01)
            ny = s * dx + c * dy + random.uniform(-0.01, 0.01)
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
