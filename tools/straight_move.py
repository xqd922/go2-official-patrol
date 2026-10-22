#!/usr/bin/env python3
"""Drive the Go2 straight for a fixed distance using /odom feedback.

Safety guards, all of which stop the robot immediately:
  * front /scan sector closer than --min-front metres
  * tilt (roll/pitch) beyond --max-tilt degrees
  * /odom silent for --odom-timeout seconds
  * wall-clock timeout derived from distance/speed plus a margin

The node keeps publishing zero velocity for a short hold period at the end so
twist_bridge forwards a stop instead of relying on its command timeout.
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def front_min(scan: LaserScan, half_angle_deg: float) -> float:
    half = math.radians(half_angle_deg)
    angle = scan.angle_min
    value = math.inf
    for r in scan.ranges:
        r = float(r)
        if math.isfinite(r) and r >= scan.range_min and abs(angle) <= half:
            value = min(value, r)
        angle += scan.angle_increment
    return value


class StraightMove(Node):
    def __init__(self, distance: float, speed: float, min_front: float, max_tilt: float):
        super().__init__("go2_straight_move")
        self.distance = distance
        self.speed = speed
        self.min_front = min_front
        self.max_tilt = max_tilt
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.start = None
        self.last_position = None
        self.travelled = 0.0
        self.front = math.inf
        self.last_scan = None
        self.tilt = 0.0
        self.last_odom = None
        self.reason = ""

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        if self.last_position is not None:
            self.travelled += math.hypot(p.x - self.last_position[0], p.y - self.last_position[1])
        self.last_position = (p.x, p.y)
        q = msg.pose.pose.orientation
        self.tilt = abs(math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))))
        if self.start is None:
            self.start = (p.x, p.y)
        self.last_odom = time.time()

    def on_scan(self, msg: LaserScan):
        self.front = front_min(msg, 30.0)
        self.last_scan = time.time()

    def stop(self, reason: str):
        self.reason = reason
        command = Twist()
        for _ in range(20):
            self.pub.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance", type=float, default=0.5)
    parser.add_argument("--speed", type=float, default=0.10)
    parser.add_argument("--min-front", type=float, default=0.40)
    parser.add_argument("--max-tilt", type=float, default=15.0)
    parser.add_argument("--odom-timeout", type=float, default=1.0)
    parser.add_argument("--scan-timeout", type=float, default=0.5)
    options = parser.parse_args()

    rclpy.init()
    node = StraightMove(options.distance, options.speed, options.min_front, options.max_tilt)
    deadline = time.time() + options.distance / options.speed * 2.0 + 3.0

    while node.start is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.start is None:
        print("ABORT: no /odom")
        node.destroy_node()
        rclpy.shutdown()
        return

    print("start %.3f,%.3f target %.2f m at %.2f m/s" % (node.start[0], node.start[1], options.distance, options.speed))

    # 先等第一帧扫描再起步：节点刚起来时 last_scan 还是 None，若直接进主循环
    # 会被 scan_timeout 立刻判定为断流，表现为"一动没动就报 scan timeout"。
    wait_until = time.time() + 3.0
    while node.last_scan is None and time.time() < wait_until:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.last_scan is None:
        print("ABORT: no /scan")
        node.destroy_node()
        rclpy.shutdown()
        return

    command = Twist()
    command.linear.x = options.speed
    reason = "done"
    try:
        while time.time() < deadline:
            if node.front < options.min_front:
                reason = "front obstacle %.2f m" % node.front
                break
            # 扫描断流时 front 会停在 inf 上，把它当成"前方空旷"继续走是最糟的
            # 失败方式。宁可原地停下，也不能盲走。
            if node.last_scan is None or time.time() - node.last_scan > options.scan_timeout:
                reason = "scan timeout"
                break
            if node.tilt > options.max_tilt:
                reason = "tilt %.1f deg" % node.tilt
                break
            if node.last_odom is not None and time.time() - node.last_odom > options.odom_timeout:
                reason = "odom timeout"
                break
            if node.travelled >= options.distance:
                break
            node.pub.publish(command)
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.stop(reason)

    if reason == "done":
        print("done: travelled %.3f m (front %.2f m, tilt %.1f deg)" % (node.travelled, node.front, node.tilt))
    else:
        print("STOPPED EARLY (%s): travelled %.3f m, front %.2f m, tilt %.1f deg" % (reason, node.travelled, node.front, node.tilt))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
