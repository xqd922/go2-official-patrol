#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from unitree_go.msg import SportModeState


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)
    return quaternion


def yaw_from_imu(quaternion) -> float:
    x, y, z, w = quaternion
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class OdomBridge(Node):
    """Publish official Go2 odometry in the form expected by ROS 2 navigation."""

    def __init__(self):
        super().__init__("go2_odom_bridge")
        self.declare_parameter("state_topic", "/lf/sportmodestate")
        self.declare_parameter("imu_topic", "/lf/imu")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("enable_odom_tf", True)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.enable_odom_tf = bool(self.get_parameter("enable_odom_tf").value)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            SportModeState,
            self.get_parameter("state_topic").value,
            self.on_state,
            5,
        )
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "go2/current_pose", 10)

        imu_node_topic = self.get_parameter("imu_topic").value
        self.pub_imu = self.create_publisher(Imu, imu_node_topic, 10)
        self.create_subscription(
            SportModeState,
            self.get_parameter("state_topic").value,
            self.publish_imu_from_state,
            5,
        )

    def publish_imu_from_state(self, state: SportModeState):
        imu = Imu()
        imu.header.frame_id = "base_link"
        q = state.imu_state.quaternion
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        self.pub_imu.publish(imu)

    def on_state(self, state: SportModeState):
        stamp = self.get_clock().now().to_msg()
        x, y, z = state.position
        yaw = yaw_from_imu(state.imu_state.quaternion)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = float(z)
        odom.pose.pose.orientation = quaternion_from_yaw(yaw)
        odom.twist.twist.linear.x = float(state.velocity[0])
        odom.twist.twist.linear.y = float(state.velocity[1])
        odom.twist.twist.angular.z = float(state.yaw_speed)
        self.pub_odom.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pub_pose.publish(pose)

        if not self.enable_odom_tf:
            return
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = float(x)
        transform.transform.translation.y = float(y)
        transform.transform.translation.z = float(z)
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
