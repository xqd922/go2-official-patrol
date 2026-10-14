#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from unitree_go.msg import LowState


class BatteryBridge(Node):
    def __init__(self):
        super().__init__("go2_battery_bridge")
        self.declare_parameter("lowstate_topic", "/lf/lowstate")
        self.declare_parameter("topic", "battery_state")
        self.publisher = self.create_publisher(BatteryState, self.get_parameter("topic").value, 10)
        self.create_subscription(LowState, self.get_parameter("lowstate_topic").value, self.on_state, 10)

    def on_state(self, state: LowState):
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.percentage = float(state.bms_state.soc) / 100.0
        message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
