#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from go2_patrol_patrol.waypoints import Waypoint, load_waypoints


IDLE = "IDLE"
RUNNING = "RUNNING"
PAUSED = "PAUSED"


def pose_message(navigator: BasicNavigator, waypoint: Waypoint) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = navigator.get_parameter("map_frame").value if navigator.has_parameter("map_frame") else "map"
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
        # 离线链路没有 AMCL（定位由静态 map->odom + 真值里程计提供），
        # 此时 waitUntilNav2Active 不能再等 amcl/get_state，否则永久阻塞。
        # 真机默认仍是 amcl，行为不变。
        self.declare_parameter("localizer", "amcl")

        self.navigator = BasicNavigator()
        localizer = self.get_parameter("localizer").value
        if localizer == "":
            # 离线：跳过定位节点等待，只等导航栈可用。
            localizer = "bt_navigator"
        self.navigator.waitUntilNav2Active(localizer=localizer)

        path = self.get_parameter("waypoints_file").value
        self.loop_count = int(self.get_parameter("loops").value)
        self.return_home = bool(self.get_parameter("return_home").value)
        self.battery_low = float(self.get_parameter("battery_low").value)
        self.arrival_tolerance = float(self.get_parameter("arrival_tolerance").value)
        self.waypoints = load_waypoints(path)
        self.loop_index = 0
        self.waypoint_index = 0
        self.spoken = set()
        # 单一状态字段：IDLE / RUNNING / PAUSED，替代原来的 paused+stopped 组合。
        # 构造完成后处于 IDLE，必须调用 /patrol/start 才开始巡检。
        self.state = IDLE
        self.low_battery = False
        self.current_goal = None
        self.dwell_until = self.get_clock().now()

        speech_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub_say = self.create_publisher(String, "/robot/say", speech_qos)
        self.create_subscription(String, "/robot/status", self.on_status, 10)
        from sensor_msgs.msg import BatteryState
        self.create_subscription(BatteryState, "/battery_state", self.on_battery, 10)
        self.create_service(Trigger, "/patrol/start", self.on_start)
        self.create_service(Trigger, "/patrol/pause", self.on_pause)
        self.create_service(Trigger, "/patrol/resume", self.on_resume)
        self.create_service(Trigger, "/patrol/stop", self.on_stop)
        self.timer = self.create_timer(0.5, self.tick)

    def say(self, waypoint: Waypoint):
        if waypoint.name in self.spoken:
            return
        message = String()
        message.data = waypoint.speech or waypoint.name
        self.pub_say.publish(message)
        self.spoken.add(waypoint.name)

    def on_status(self, message: String):
        if message.data == "stop" and self.current_goal:
            self.navigator.cancelTask()

    def on_battery(self, message):
        percentage = message.percentage * 100.0 if message.percentage <= 1.0 else message.percentage
        if not self.low_battery and percentage <= self.battery_low:
            self.low_battery = True
            if self.current_goal:
                self.navigator.cancelTask()

    def on_start(self, request, response):
        if self.state == RUNNING:
            response.success = False
            response.message = "patrol already running"
            return response
        if self.state == PAUSED:
            # 从暂停恢复，保留当前航点进度。
            self.state = RUNNING
            response.success = True
            response.message = "patrol resumed"
            return response
        # IDLE：开始新一轮巡检，复位进度。
        self.reset_run()
        self.state = RUNNING
        response.success = True
        response.message = "patrol started"
        return response

    def on_pause(self, request, response):
        if self.state != RUNNING:
            response.success = False
            response.message = f"cannot pause: patrol is {self.state.lower()}"
            return response
        if self.current_goal:
            self.navigator.cancelTask()
        self.state = PAUSED
        response.success = True
        response.message = "paused"
        return response

    def on_resume(self, request, response):
        if self.state != PAUSED:
            response.success = False
            response.message = f"cannot resume: patrol is {self.state.lower()}"
            return response
        self.state = RUNNING
        response.success = True
        response.message = "resumed"
        return response

    def on_stop(self, request, response):
        if self.state == IDLE:
            response.success = False
            response.message = "patrol already idle"
            return response
        if self.current_goal:
            self.navigator.cancelTask()
        self.reset_run()
        self.state = IDLE
        response.success = True
        response.message = "stopped"
        return response

    def reset_run(self):
        """清除当前巡检进度，回到可重新 start 的干净状态。"""
        self.loop_index = 0
        self.waypoint_index = 0
        self.spoken = set()
        self.low_battery = False
        self.current_goal = None
        self.dwell_until = self.get_clock().now()

    def send_goal(self, waypoint: Waypoint):
        goal = pose_message(self.navigator, waypoint)
        self.navigator.goToPose(goal)
        self.current_goal = True

    def tick(self):
        if self.state != RUNNING:
            return

        if self.get_clock().now() < self.dwell_until:
            return

        if not self.current_goal:
            if self.low_battery:
                self.send_goal(self.waypoints[0])
                return
            self.send_goal(self.waypoints[self.waypoint_index])
            return

        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            waypoint = self.waypoints[self.waypoint_index]
            self.say(waypoint)
            self.dwell_until = self.get_clock().now() + Duration(seconds=waypoint.dwell)
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.loop_index += 1
                self.waypoint_index = 0
            if self.loop_count > 0 and self.loop_index >= self.loop_count:
                self.state = IDLE
        elif result == TaskResult.FAILED:
            self.get_logger().warn("navigation goal failed; retrying")

        self.current_goal = None


def main(args=None):
    rclpy.init(args=args)
    navigator = PatrolManager()
    try:
        rclpy.spin(navigator)
    finally:
        navigator.destroy_node()
        rclpy.shutdown()
