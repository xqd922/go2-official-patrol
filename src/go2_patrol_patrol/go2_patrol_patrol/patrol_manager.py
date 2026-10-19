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

from go2_patrol_patrol.waypoints import (Waypoint, load_waypoints, normalize_orientation_mode,
                                         yaw_towards)


IDLE = "IDLE"
RUNNING = "RUNNING"
PAUSED = "PAUSED"


def pose_message(navigator: BasicNavigator, waypoint: Waypoint, yaw=None) -> PoseStamped:
    """把航点转成 Nav2 的 PoseStamped。

    yaw 为 None 时严格使用 waypoint.yaw（手动朝向）；patrol_manager 计算好
    "朝向下一航点"的实际 yaw 后会显式传入，保证每条 goal 的最终姿态明确。
    """
    goal_yaw = waypoint.yaw if yaw is None else float(yaw)
    pose = PoseStamped()
    pose.header.frame_id = navigator.get_parameter("map_frame").value if navigator.has_parameter("map_frame") else "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = waypoint.x
    pose.pose.position.y = waypoint.y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(goal_yaw * 0.5)
    pose.pose.orientation.w = math.cos(goal_yaw * 0.5)
    return pose


def resolve_goal_yaw(waypoints, index, loop_index, loop_count, return_home):
    """计算第 index 个航点到达时的最终朝向（弧度）。

    规则（普通巡检点 default auto_next）：
      * 手动朝向（orientation_mode: fixed）：严格用 waypoint.yaw。
      * 自动朝向（auto_next）：朝向"本趟下一个要去的点"，即 waypoints[index+1]；
      * 最后一个点：若之后还会返回 home（return_home 或还有下一轮循环），
        朝向 home（home 点优先，否则序列第一个点）；否则保持自身 yaw，
        **不为了不存在的下一点额外旋转**。
    """
    waypoint = waypoints[index]
    if normalize_orientation_mode(waypoint.orientation_mode, waypoint.yaw) != "auto_next":
        return float(waypoint.yaw)
    if index + 1 < len(waypoints):
        return yaw_towards(waypoint, waypoints[index + 1])
    # 最后一个点：看之后是否真的还有"下一个目标"。
    # 两种情况都意味着"确实还要去下一个地方"：
    #   1) return_home=True：本趟结束后会导航回 home（tick() 里真正执行）；
    #   2) 还有下一轮循环（loop_count<=0 为无限循环）。
    has_next_loop = loop_count <= 0 or (loop_index + 1) < loop_count
    if return_home or has_next_loop:
        home = find_home(waypoints)
        if home is not None:
            return yaw_towards(waypoint, home)
        return yaw_towards(waypoint, waypoints[0])
    # 巡检到此真正结束：不做无意义的额外转向
    return float(waypoint.yaw)


def find_home(waypoints):
    """返回标记 home 的航点（没有则 None）。"""
    for waypoint in waypoints:
        if waypoint.home:
            return waypoint
    return None


class PatrolManager(Node):
    def __init__(self):
        super().__init__("patrol_manager")
        self.declare_parameter("waypoints_file", "waypoints.yaml")
        self.declare_parameter("loops", 1)
        # return_home 现在真正生效（最后一趟结束后导航回 home 点）。
        # 默认 False 保持"跑完 loops 次即停止"的既有/文档化行为，需要回充/回起点时
        # 再显式开：ROS 参数 return_home:=true（或 launch 参数）。
        self.declare_parameter("return_home", False)
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
        # return_home 生效时：最后一趟结束后"回家"这一段正在走。
        self.returning_home = False
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
        self.returning_home = False
        self.dwell_until = self.get_clock().now()

    def send_goal(self, waypoint: Waypoint, yaw=None):
        goal = pose_message(self.navigator, waypoint, yaw=yaw)
        self.navigator.goToPose(goal)
        self.current_goal = True

    def home_goal(self):
        """返回 home 航点；没有标记 home 的则退回序列第一个点。"""
        home = find_home(self.waypoints)
        return home if home is not None else (self.waypoints[0] if self.waypoints else None)

    def goal_yaw_for(self, index: int) -> float:
        """第 index 个航点到达时的最终朝向。"""
        return resolve_goal_yaw(self.waypoints, index, self.loop_index,
                                self.loop_count, self.return_home)

    def tick(self):
        if self.state != RUNNING:
            return

        if self.get_clock().now() < self.dwell_until:
            return

        # ---- 当前没有在跑的 goal：决定下一个目标 ----
        if not self.current_goal:
            if self.low_battery:
                # 低电量：回 home（而不是盲目回序列第 0 个点），朝向用 home 自身 yaw。
                home = self.home_goal()
                if home is not None:
                    self.send_goal(home, yaw=home.yaw)
                    self.returning_home = True
                return
            index = self.waypoint_index
            self.send_goal(self.waypoints[index], yaw=self.goal_yaw_for(index))
            self.returning_home = False
            return

        # ---- 有 goal 在跑：等它结束 ----
        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            if self.returning_home:
                # 已回到 home：本趟彻底结束，不 dwell、不再发目标。
                self.returning_home = False
                self.current_goal = None
                self.state = IDLE
                return
            waypoint = self.waypoints[self.waypoint_index]
            self.say(waypoint)
            self.dwell_until = self.get_clock().now() + Duration(seconds=waypoint.dwell)
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.loop_index += 1
                self.waypoint_index = 0
            finished = self.loop_count > 0 and self.loop_index >= self.loop_count
            if finished:
                home = self.home_goal() if self.return_home else None
                if home is not None:
                    # return_home 真正生效：最后一趟结束后再导航回 home，
                    # 到点（下一个 tick 的 returning_home 分支）才转 IDLE。
                    self.current_goal = None
                    self.returning_home = True
                    self.send_goal(home, yaw=home.yaw)
                    return
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
