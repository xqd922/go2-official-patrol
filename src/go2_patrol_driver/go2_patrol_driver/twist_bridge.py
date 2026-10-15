#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from unitree_api.msg import Request

from go2_patrol_driver.request_utils import sport_request
from go2_patrol_driver.sport_api_ids import (
    ROBOT_SPORT_API_ID_MOVE,
    ROBOT_SPORT_API_ID_STOP_MOVE,
)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def front_range(scan: LaserScan, half_angle_deg: float) -> float:
    half = math.radians(half_angle_deg)
    angle = scan.angle_min
    minimum = math.inf
    for value in scan.ranges:
        value = float(value)
        if math.isfinite(value) and value >= scan.range_min and abs(angle) <= half:
            minimum = min(minimum, value)
        angle += scan.angle_increment
    return minimum


class TwistBridge(Node):
    """Convert Nav2 /cmd_vel to official Unitree Sport MOVE requests."""

    def __init__(self):
        super().__init__("go2_twist_bridge")
        self.declare_parameter("max_linear", 0.7)
        self.declare_parameter("max_angular", 1.0)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("stop_distance", 0.35)
        self.declare_parameter("slow_distance", 0.60)
        self.declare_parameter("emergency_distance", 0.20)
        self.declare_parameter("front_half_angle_deg", 30.0)
        self.declare_parameter("brake_hold_sec", 0.4)

        gp = self.get_parameter
        self.max_linear = float(gp("max_linear").value)
        self.max_angular = float(gp("max_angular").value)
        self.stop_distance = float(gp("stop_distance").value)
        self.slow_distance = float(gp("slow_distance").value)
        self.emergency_distance = float(gp("emergency_distance").value)
        self.half_angle = float(gp("front_half_angle_deg").value)
        self.brake_hold_sec = float(gp("brake_hold_sec").value)

        self.commands_sent = 0
        self.last_emergency = self.get_clock().now() - rclpy.duration.Duration(seconds=10.0)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        # 必须用 sensor data QoS：pointcloud_to_laserscan 以 best_effort 发布 /scan，
        # 默认的 reliable 订阅与之不兼容，日志会报 incompatible QoS 且收不到任何扫描，
        # 导致前置急停（stop_distance / emergency_distance）静默失效。
        self.create_subscription(
            LaserScan, gp("scan_topic").value, self.on_scan, qos_profile_sensor_data
        )
        self.pub_request = self.create_publisher(Request, "/api/sport/request", 10)
        self.timer = self.create_timer(0.05, self.republish_stop)

    def on_scan(self, scan):
        self.minimum = front_range(scan, self.half_angle)

    def should_stop_for_safety(self):
        minimum = getattr(self, "minimum", math.inf)
        if minimum <= self.emergency_distance:
            self.last_emergency = self.get_clock().now()
            return True
        return minimum <= self.stop_distance

    def on_cmd_vel(self, twist: Twist):
        vx = clamp(float(twist.linear.x), self.max_linear)
        wz = clamp(float(twist.angular.z), self.max_angular)

        minimum = getattr(self, "minimum", math.inf)
        if minimum < self.slow_distance:
            vx *= 0.5
        if self.should_stop_for_safety():
            self.send_stop_move()
            self.get_logger().info(f"front safety stop: {minimum:.2f}m", throttle_duration_sec=1.0)
            return

        request = sport_request(
            ROBOT_SPORT_API_ID_MOVE,
            {"x": vx, "y": float(twist.linear.y), "z": wz},
            self.commands_sent,
        )
        self.commands_sent += 1
        self.pub_request.publish(request)

    def send_stop_move(self):
        request = sport_request(ROBOT_SPORT_API_ID_STOP_MOVE, {}, self.commands_sent)
        self.commands_sent += 1
        self.pub_request.publish(request)

    def republish_stop(self):
        if self.get_clock().now() - self.last_emergency < rclpy.duration.Duration(seconds=self.brake_hold_sec):
            self.send_stop_move()


def main(args=None):
    rclpy.init(args=args)
    node = TwistBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
