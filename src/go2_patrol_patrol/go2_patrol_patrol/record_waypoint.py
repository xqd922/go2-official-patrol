#!/usr/bin/env python3
"""采集当前位置，打印成可直接粘进 waypoints.yaml 的片段。

坐标系必须是 map —— Nav2 的目标就是在 map 帧下的（patrol_manager 发目标时
frame_id 取 map），所以默认订阅 /amcl_pose。
订阅 /odom 拿到的只是 odom 帧坐标：AMCL 的 map→odom 变换不为零时，
采出来的点和静态地图对不上（现场表现：采的点发过去，狗走到旁边去了）。
只有显式给了 --allow-odom-fallback 且 /amcl_pose 一直没有数据时才会退回 /odom，
并且会大声警告。
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def yaw_from_quaternion(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )


class WaypointRecorder(Node):
    def __init__(self, pose_topic: str):
        super().__init__("record_waypoint")
        self.received = None
        self.frame_id = ""
        self.source = ""
        self.pose_topic = pose_topic
        # /amcl_pose 是 sensor data QoS 发的，用默认 reliable 订阅会收不到
        self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self.on_pose, qos_profile_sensor_data
        )

    def on_pose(self, message):
        self.received = message.pose.pose
        self.frame_id = message.header.frame_id or "map"
        self.source = "amcl_pose"

    def start_odom_fallback(self):
        """退路：/amcl_pose 一直没数据时才订 /odom（结果是 odom 帧的坐标）。"""
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.get_logger().warn("改用 /odom 采集：坐标属于 odom 帧，粘进 waypoints.yaml 前必须核对")

    def on_odom(self, message):
        if self.received is not None:
            return      # 已经拿到 map 坐标了，不用 odom 覆盖
        self.received = message.pose.pose
        self.frame_id = message.header.frame_id or "odom"
        self.source = "odom"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="采集一个巡检点位并打印 YAML 片段")
    parser.add_argument("name", help="点位名（同时作为 speech id 的建议值）")
    parser.add_argument("--pose-topic", default="/amcl_pose", help="位姿话题，默认 /amcl_pose（map 帧）")
    parser.add_argument("--timeout", type=float, default=10.0, help="每阶段等待秒数，默认 10")
    parser.add_argument(
        "--allow-odom-fallback",
        action="store_true",
        help="没有 /amcl_pose 时退回 /odom（坐标是 odom 帧，需自行换算/核对）",
    )
    return parser.parse_args(argv)


def wait_for_pose(node, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and node.received is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    return node.received is not None


def main(argv=None):
    options = parse_args(argv)
    rclpy.init()
    node = WaypointRecorder(options.pose_topic)
    try:
        if not wait_for_pose(node, options.timeout) and options.allow_odom_fallback:
            node.start_odom_fallback()
            wait_for_pose(node, options.timeout)
        if node.received is None:
            print(
                f"[record_waypoint] {options.timeout:.0f}s 内没收到 {options.pose_topic} 的位姿。"
                "先确认定位在跑（ros2 topic hz /amcl_pose），或加 --allow-odom-fallback 用 /odom 采"
                "（坐标是 odom 帧）。",
                file=sys.stderr,
            )
            return 1
        pose = node.received
        yaw = yaw_from_quaternion(pose.orientation)
        if node.source == "odom":
            print(
                "[record_waypoint] 警告：这份坐标采自 /odom（odom 帧），"
                "与 patrol_manager 使用的 map 帧不一致，粘进 waypoints.yaml 前务必核对。",
                file=sys.stderr,
            )
        print(f"  # 采自 {node.source}，坐标系 {node.frame_id}")
        print(
            f"  - name: {options.name}\n"
            f"    x: {pose.position.x:.3f}\n"
            f"    y: {pose.position.y:.3f}\n"
            f"    yaw: {yaw:.3f}\n"
            f"    dwell: 4.0\n"
            f"    speech: {options.name}"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
