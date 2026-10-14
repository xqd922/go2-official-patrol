#!/usr/bin/env python3
import argparse

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request

from go2_patrol_driver.request_utils import sport_request


ACTIONS = {
    "balance_stand": 1002,
    "stop_move": 1003,
    "stand_up": 1004,
    "stand_down": 1005,
    "recovery_stand": 1006,
    "damp": 1001,
}


class SportCommand(Node):
    def __init__(self, action: str, counter: int = 0):
        super().__init__("go2_sport_command")
        if action not in ACTIONS:
            raise ValueError(f"unknown action: {action}")
        self.publisher = self.create_publisher(Request, "/api/sport/request", 10)
        self.timer = self.create_timer(0.2, lambda: self.publish(action, counter))
        self.sent = False

    def publish(self, action, counter):
        if self.sent:
            return
        self.publisher.publish(sport_request(ACTIONS[action], {}, counter))
        self.sent = True
        self.get_logger().info(f"sent {action}")


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=sorted(ACTIONS))
    options = parser.parse_args(args)
    rclpy.init(args=None)
    node = SportCommand(options.action)
    try:
        end = node.get_clock().now() + rclpy.duration.Duration(seconds=0.5)
        while rclpy.ok() and not node.sent and node.get_clock().now() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
