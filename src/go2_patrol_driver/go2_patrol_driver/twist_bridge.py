#!/usr/bin/env python3
import math
import signal
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from unitree_api.msg import Request

from go2_patrol_driver.request_utils import sport_request
from go2_patrol_driver.safety import (
    ACTION_EMERGENCY,
    ACTION_NO_SCAN,
    ACTION_SLOW,
    ACTION_STOP,
    front_action as decide_front_action,
    is_fresh,
    scan_is_fresh,
)
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
        # 前置急停的看门狗：扫描断流时宁可停车，也不要"当作前方畅通"继续走。
        # 旧实现里"没收到过扫描"和"前方很空"都落到 inf 上，急停会静默失效。
        self.declare_parameter("scan_timeout", 1.0)
        self.declare_parameter("require_scan", True)
        # 指令断流的看门狗：官方 MOVE 会保持最后一次速度，Nav2 / patrol 一退出
        # /cmd_vel 就停了，狗会带着最后那条速度一直走下去，日志里一个字都没有。
        self.declare_parameter("cmd_vel_timeout", 0.5)

        gp = self.get_parameter
        self.scan_topic = str(gp("scan_topic").value)
        self.max_linear = float(gp("max_linear").value)
        self.max_angular = float(gp("max_angular").value)
        self.stop_distance = float(gp("stop_distance").value)
        self.slow_distance = float(gp("slow_distance").value)
        self.emergency_distance = float(gp("emergency_distance").value)
        self.half_angle = float(gp("front_half_angle_deg").value)
        self.brake_hold_sec = float(gp("brake_hold_sec").value)
        self.scan_timeout = float(gp("scan_timeout").value)
        self.require_scan = bool(gp("require_scan").value)
        self.cmd_vel_timeout = float(gp("cmd_vel_timeout").value)

        self.commands_sent = 0
        self.minimum = math.inf
        self.last_scan_time = None
        self.last_cmd_time = None
        # 上一条 /cmd_vel 是不是"要求走"。收尾/断流时靠它判断要不要补一脚刹车：
        # 如果导航栈本来就是靠发零速度停下来的，那就没必要再动。
        self.commanded_motion = False
        # 进入退出流程后不再接受任何 /cmd_vel：stop_on_shutdown 里为了把刹车帧刷出去
        # 要转几圈 spin_once，那会正常派发订阅回调，队列里积压的非零 /cmd_vel 会被
        # 重新处理成一条 MOVE 并**排在刹车之后**（官方 MOVE 保持最后速度，等于退出时
        # 又放狗走）。置上这个标志，退出路径上的 /cmd_vel 一律丢弃。
        self.shutting_down = False
        self.last_emergency = self.get_clock().now() - rclpy.duration.Duration(seconds=10.0)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        # 必须用 sensor data QoS：pointcloud_to_laserscan 以 best_effort 发布 /scan，
        # 默认的 reliable 订阅与之不兼容，日志会报 incompatible QoS 且收不到任何扫描，
        # 导致前置急停（stop_distance / emergency_distance）静默失效。
        self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data
        )
        self.pub_request = self.create_publisher(Request, "/api/sport/request", 10)
        self.timer = self.create_timer(0.05, self.watchdog)

    def on_scan(self, scan):
        self.minimum = front_range(scan, self.half_angle)
        self.last_scan_time = self.get_clock().now()

    def current_action(self) -> str:
        """这一拍按"前方最近距离 + 扫描新不新"决定动作（判断本身在 safety.py，可单测）。"""
        return decide_front_action(
            self.minimum,
            self.emergency_distance,
            self.stop_distance,
            self.slow_distance,
            fresh=scan_is_fresh(
                self.last_scan_time.nanoseconds if self.last_scan_time is not None else None,
                self.get_clock().now().nanoseconds,
                self.scan_timeout,
            ),
            require_scan=self.require_scan,
        )

    def on_cmd_vel(self, twist: Twist):
        if self.shutting_down:
            # 退出流程已经在发刹车了，别再被队列里的旧指令翻回去。
            return
        vx = clamp(float(twist.linear.x), self.max_linear)
        wz = clamp(float(twist.angular.z), self.max_angular)

        self.last_cmd_time = self.get_clock().now()
        self.commanded_motion = bool(vx or wz or float(twist.linear.y))

        action = self.current_action()
        if action == ACTION_NO_SCAN:
            # 没有新鲜扫描不再当作"前方畅通"：停车并把它喊出来。
            # 静默失效是这类安全逻辑最糟的失败方式——真机上表现为"急停有时不灵"。
            self.send_stop_move()
            self.get_logger().warn(
                f"没有新的 /{self.scan_topic.lstrip('/')}（超过 {self.scan_timeout:.1f}s 或从未收到），"
                "按安全策略停车：检查机载点云 /utlidar/cloud_deskewed 与 pointcloud_to_laserscan；"
                "确认无雷达的台架才允许把 require_scan 设为 false",
                throttle_duration_sec=5.0,
            )
            return
        if action == ACTION_EMERGENCY:
            self.last_emergency = self.get_clock().now()
        if action in (ACTION_EMERGENCY, ACTION_STOP):
            self.send_stop_move()
            self.get_logger().info(f"front safety stop: {self.minimum:.2f}m", throttle_duration_sec=1.0)
            return
        if action == ACTION_SLOW:
            vx *= 0.5

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

    def watchdog(self):
        """20Hz 兜底：急停后压住刹车，且 /cmd_vel 断流时补一脚刹车。

        官方 MOVE 是"保持最后一次速度"，所以进程被杀 / 导航栈退出 / 网络抖动
        都不会让狗自己停下来。这里以"收到过要求走、之后 0.5s 再没有新指令"
        作为断流判据——Nav2 正常停车时会持续发零速度，不会被误判。
        """
        now = self.get_clock().now()
        if now - self.last_emergency < rclpy.duration.Duration(seconds=self.brake_hold_sec):
            self.send_stop_move()
            return
        if not self.commanded_motion:
            return
        if is_fresh(
            self.last_cmd_time.nanoseconds if self.last_cmd_time is not None else None,
            now.nanoseconds,
            self.cmd_vel_timeout,
        ):
            return
        self.commanded_motion = False
        self.send_stop_move()
        self.get_logger().warn(
            f"/cmd_vel 超过 {self.cmd_vel_timeout:.1f}s 没有新指令，已补发 STOP_MOVE 刹车"
            "（导航栈/巡逻节点是不是退出了？官方 MOVE 会保持最后一次速度）",
            throttle_duration_sec=5.0,
        )

    def stop_on_shutdown(self):
        """退出前补一脚刹车，并给 DDS 留出把这一帧发出去的时间。

        官方 MOVE 是"保持最后速度"，所以被 Ctrl-C / systemd stop 掉时不给刹车
        等于放狗自己走。
        """
        self.shutting_down = True      # 先关掉指令通道，见 __init__ 里的说明
        if not self.commanded_motion:
            return
        self.commanded_motion = False
        try:
            self.send_stop_move()
        except Exception as exc:  # noqa: BLE001 - 退出路径必须走到 destroy_node
            # 这里用 print 而不是 get_logger：context 若已关，ROS 日志本身也会抛。
            print(f"[twist_bridge] 退出前补发 STOP_MOVE 失败: {exc}", flush=True)
            return
        # 刚 publish 完就 destroy_node 会丢包，转几圈让中间件把这一帧送出去。
        for _ in range(4):
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:  # noqa: BLE001 - context 已关时刷帧失败不影响刹车已发出
                break


def main(args=None):
    # 自己接管 SIGINT/SIGTERM：rclpy 默认的处理器会**先关掉 context** 再让 spin 返回，
    # 等走到 finally 时 publisher 已经失效（rcl_publish 返回 RCL_RET_PUBLISHER_INVALID），
    # 退出前那一脚刹车就发不出去——而 systemd 的 KillSignal 正是 SIGINT。
    # 这里让 context 活到刹车发完为止（SignalHandlerOptions.NONE = 不装 rclpy 的处理器）。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NONE)
    node = TwistBridge()
    stopping = threading.Event()

    def on_signal(signum, _frame):
        print(f"[twist_bridge] 收到信号 {signum}：先补一脚刹车再退出", flush=True)
        stopping.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, on_signal)
        except (ValueError, OSError):   # 非主线程/不支持的平台：退回默认行为
            pass

    try:
        while rclpy.ok() and not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.stop_on_shutdown()
        node.destroy_node()
        rclpy.shutdown()
