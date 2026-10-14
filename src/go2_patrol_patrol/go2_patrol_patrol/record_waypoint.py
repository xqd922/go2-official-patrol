#!/usr/bin/env python3
import argparse
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__("record_waypoint")
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.received = None

    def on_odom(self, message):
        self.received = message.pose.pose


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("name")
    options = parser.parse_args(args)
    rclpy.init()
    node = WaypointRecorder()
    while rclpy.ok() and node.received is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    pose = node.received
    yaw = math.atan2(2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
                     1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2))
    print(
        f"  - name: {options.name}\n"
        f"    x: {pose.position.x:.3f}\n"
        f"    y: {pose.position.y:.3f}\n"
        f"    yaw: {yaw:.3f}\n"
        f"    dwell: 4.0\n"
        f"    speech: {options.name}"
    )
    node.destroy_node()
    rclpy.shutdown()
