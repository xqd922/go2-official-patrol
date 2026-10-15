#!/usr/bin/env python3
"""巡检调度节点：把巡检点位序列交给 Nav2，并在到点后播报、停留、收官。

调度规则（圈数/返航/低电量）全部收在 patrol_sequence.PatrolSequence 里，
本节点只负责 ROS 交互：发目标、听结果、播音、报状态。
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from go2_patrol_patrol.patrol_sequence import PatrolSequence
from go2_patrol_patrol.waypoints import Waypoint, load_waypoints


def pose_message(navigator: BasicNavigator, waypoint: Waypoint) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = (
        navigator.get_parameter("map_frame").value
        if navigator.has_parameter("map_frame")
        else "map"
    )
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = waypoint.x
    pose.pose.position.y = waypoint.y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(waypoint.yaw * 0.5)
    pose.pose.orientation.w = math.cos(waypoint.yaw * 0.5)
    return pose


class PatrolManager(Node):
    def __init__(self):
        super().__init__("patrol_manager")
        self.declare_parameter("waypoints_file", "waypoints.yaml")
        self.declare_parameter("loops", 1)
        self.declare_parameter("return_home", True)
        self.declare_parameter("battery_low", 20.0)
        self.declare_parameter("arrival_tolerance", 0.30)

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active(localizer="amcl")

        path = self.get_parameter("waypoints_file").value
        self.battery_low = float(self.get_parameter("battery_low").value)
        self.arrival_tolerance = float(self.get_parameter("arrival_tolerance").value)
        self.waypoints = load_waypoints(path)
        self.sequence = PatrolSequence(
            self.waypoints,
            loops=int(self.get_parameter("loops").value),
            return_home=bool(self.get_parameter("return_home").value),
        )

        self.paused = False
        self.stop_requested = False
        self.finished = False
        self.low_battery = False
        self.current_goal = False
        self.active_waypoint = None
        self.amcl_pose = None
        self.dwell_until = self.get_clock().now()

        speech_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub_say = self.create_publisher(String, "/robot/say", speech_qos)
        self.create_subscription(String, "/robot/status", self.on_status, 10)
        self.create_subscription(BatteryState, "/battery_state", self.on_battery, 10)
        # AMCL 位姿 = Nav2 判定"到没到"所用的位姿，用来复核到点精度（见 log_arrival）
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, qos_profile_sensor_data
        )
        self.create_service(Trigger, "/patrol/start", self.on_start)
        self.create_service(Trigger, "/patrol/pause", self.on_pause)
        self.create_service(Trigger, "/patrol/resume", self.on_resume)
        self.create_service(Trigger, "/patrol/stop", self.on_stop)
        self.timer = self.create_timer(0.5, self.tick)

        if not self.waypoints:
            self.get_logger().error(f"{path} 里没有点位，巡检无事可做")
        self.get_logger().info(
            f"巡检任务就绪：点位 {len(self.waypoints)} 个，{self.sequence.describe()}，"
            f"低电量阈值 {self.battery_low:.0f}%，到点期望容差 {self.arrival_tolerance:.2f}m"
        )

    # ---------------------------------------------------------------- 播报
    def say(self, waypoint: Waypoint):
        """每个点到点播报一次（多圈巡检每圈都会播）。"""
        message = String()
        message.data = waypoint.speech or waypoint.name
        self.pub_say.publish(message)

    def log_arrival(self, waypoint: Waypoint):
        if self.amcl_pose is None:
            return
        distance = math.hypot(self.amcl_pose[0] - waypoint.x, self.amcl_pose[1] - waypoint.y)
        if distance > self.arrival_tolerance:
            self.get_logger().warn(
                f"{waypoint.name} 报成功但 AMCL 位姿距目标 {distance:.2f}m"
                f"（期望 ≤ {self.arrival_tolerance:.2f}m）：核对站点 nav2_params.yaml 的 "
                "goal_checker.xy_goal_tolerance 与地图/起点标定"
            )

    # ---------------------------------------------------------------- 回调
    def on_pose(self, message: PoseWithCovarianceStamped):
        self.amcl_pose = (message.pose.pose.position.x, message.pose.pose.position.y)

    def on_status(self, message: String):
        if message.data.strip() == "stop":
            self.request_stop("收到 /robot/status=stop")

    def on_battery(self, message: BatteryState):
        percentage = message.percentage * 100.0 if message.percentage <= 1.0 else message.percentage
        if self.low_battery or percentage > self.battery_low:
            return
        self.low_battery = True
        self.get_logger().warn(
            f"电量 {percentage:.1f}% ≤ 阈值 {self.battery_low:.1f}%，中止巡检并返航 home"
        )
        if self.current_goal:
            self.navigator.cancelTask()
        self.sequence.request_return_home()

    def on_start(self, request, response):
        if self.finished:
            response.success = False
            response.message = "patrol already finished"
            return response
        self.paused = False
        self.stop_requested = False
        response.success = True
        response.message = "patrol started"
        return response

    def on_pause(self, request, response):
        if self.current_goal:
            self.navigator.cancelTask()
        self.paused = True
        response.success = True
        response.message = "paused"
        return response

    def on_resume(self, request, response):
        self.paused = False
        response.success = True
        response.message = "resumed"
        return response

    def on_stop(self, request, response):
        self.request_stop("收到 /patrol/stop 服务")
        response.success = True
        response.message = "stopped"
        return response

    def request_stop(self, reason: str):
        if self.current_goal:
            self.navigator.cancelTask()
        self.paused = False
        self.stop_requested = True
        self.get_logger().info(f"收到停止请求（{reason}）")

    # ---------------------------------------------------------------- 调度
    def send_goal(self, waypoint: Waypoint):
        self.navigator.goToPose(pose_message(self.navigator, waypoint))
        self.current_goal = True
        self.active_waypoint = waypoint
        self.get_logger().info(f"前往 {waypoint.name} ({waypoint.x:.2f}, {waypoint.y:.2f})")

    def tick(self):
        if self.finished:
            return
        if self.stop_requested:
            self.finish("停止请求")
            return
        if self.paused:
            return
        if self.get_clock().now() < self.dwell_until:
            return

        if not self.current_goal:
            waypoint = self.sequence.next_goal()
            if waypoint is None:
                self.finish("巡检任务完成")
                return
            self.send_goal(waypoint)
            return

        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()
        waypoint = self.active_waypoint
        self.current_goal = False
        self.active_waypoint = None
        if waypoint is None:
            return

        if result == TaskResult.SUCCEEDED:
            self.say(waypoint)
            self.log_arrival(waypoint)
            self.dwell_until = self.get_clock().now() + Duration(seconds=waypoint.dwell)
            self.sequence.on_arrival(waypoint)
            self.get_logger().info(f"到达 {waypoint.name} → {self.sequence.describe()}")
        elif result == TaskResult.FAILED:
            self.get_logger().warn(f"前往 {waypoint.name} 失败，稍后重试")
        else:
            self.get_logger().info(f"目标 {waypoint.name} 被取消（{result}）")

    def finish(self, reason: str):
        if self.finished:
            return
        self.finished = True
        self.get_logger().info(f"巡检结束（{reason}），关闭 Nav2 生命周期节点")
        self.timer.cancel()
        try:
            self.navigator.lifecycleShutdown()
        except Exception as exc:  # 真机上 Nav2 可能已被 systemd 重启，这里只提示不抛出
            self.get_logger().warn(f"lifecycleShutdown 失败：{exc}")


def main(args=None):
    rclpy.init(args=args)
    manager = PatrolManager()
    try:
        while rclpy.ok() and not manager.finished:
            rclpy.spin_once(manager, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
