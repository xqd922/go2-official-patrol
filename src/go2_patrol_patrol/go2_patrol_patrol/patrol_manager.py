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

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active(localizer="amcl")

        path = self.get_parameter("waypoints_file").value
        self.loop_count = int(self.get_parameter("loops").value)
        self.return_home = bool(self.get_parameter("return_home").value)
        self.battery_low = float(self.get_parameter("battery_low").value)
        self.arrival_tolerance = float(self.get_parameter("arrival_tolerance").value)
        self.waypoints = load_waypoints(path)
        self.loop_index = 0
        self.waypoint_index = 0
        self.spoken = set()
        self.paused = False
        self.stopped = False
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
            self.navigator.cancelTask()

    def on_start(self, request, response):
        self.paused = False
        self.stopped = False
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
        if self.current_goal:
            self.navigator.cancelTask()
        self.stopped = True
        self.paused = False
        response.success = True
        response.message = "stopped"
        return response

    def send_goal(self, waypoint: Waypoint):
        goal = pose_message(self.navigator, waypoint)
        self.navigator.goToPose(goal)
        self.current_goal = True

    def tick(self):
        if self.stopped:
            self.navigator.lifecycleShutdown()
            rclpy.shutdown()
            return
        if self.paused:
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
                self.stopped = True
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
