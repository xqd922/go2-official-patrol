#!/usr/bin/env python3
"""按需补链 TF：base_footprint → base_link。

为什么需要这个节点
------------------
生产 launch 里这一段 TF 没有任何发布者：`odom_bridge` 只发
`odom → base_footprint`，`perception.launch.py` 只发 `base_link → utlidar_lidar`。
而 `pointcloud_to_laserscan` 的 `target_frame` 正是 `base_footprint`
（见 config/perception.yaml），TF 树断在这里就变不出 `/scan`，
AMCL 没有观测 → 导航整条链起不来。
离线假狗（tools/offline_demo/fake_go2_loop.py）一直自己补这一段，
所以这个问题在离线验收里永远测不出来，只有真机上才暴露。

但反过来，机器狗自身若已经发这段 TF，再多一个发布者会让 Nav2 报
"TF 冲突 / 多个父节点"，这也是文档里反复强调的坑。所以默认 mode=auto：
先只监听、判断，确认没人发才开始补链；发现别处已经在发就完全不发。
两种世界的处理都写进日志，现场可用 `ros2 run tf2_tools view_frames` 复核。

mode（launch 参数 `base_footprint_tf`，默认取环境变量 GO2_BASE_FOOTPRINT_TF）：
  auto    默认。探测窗口内没人发布才补
  always  无条件补（现场已确认狗自身不发这段 TF 时最干净）
  never   不补（现场已确认狗自身会发，或想完全手工控制）
"""
import os
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster

from go2_patrol_driver.frame_chain import (
    MODE_ALWAYS,
    MODE_NEVER,
    OURS_MEMORY,
    is_ours,
    is_target_link,
    normalize_frame,
    normalize_mode,
    publish_due,
    should_publish,
)


class TfChainGuard(Node):
    def __init__(self):
        super().__init__("go2_tf_chain_guard")
        self.declare_parameter("mode", os.environ.get("GO2_BASE_FOOTPRINT_TF", "auto"))
        self.declare_parameter("parent_frame", "base_footprint")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("probe_seconds", 5.0)
        self.declare_parameter("publish_period", 1.0)

        self.mode = normalize_mode(self.get_parameter("mode").value)
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.probe_seconds = float(self.get_parameter("probe_seconds").value)
        self.publish_period = float(self.get_parameter("publish_period").value)

        self.started = self.get_clock().now()
        self.external_seen = False
        self.external_parent = ""
        self.last_published_ns = None
        self.published_stamps = deque(maxlen=OURS_MEMORY)
        self.publishing = False
        self.stand_down_logged = False
        self.conflict_reported = False
        self.parent_mismatch_reported = False

        self.broadcaster = StaticTransformBroadcaster(self)
        # /tf_static 必须用 transient_local 订阅：static_transform_publisher 与
        # 机器狗自身发的静态 TF 都是 latched 的，用默认 QoS 订阅收不到任何一条，
        # 于是"探测"变成永远探测不到，反而会去和机器狗抢着发。
        static_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(TFMessage, "/tf_static", self.on_tf, static_qos)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        # 定时器固定 0.2s 跑探测/判断（探测要快），实际重发节流在 publish_due 里按
        # publish_period 控制——不能把周期塞进定时器：那会让探测窗口的响应变慢。
        self.create_timer(0.2, self.tick)

        self.get_logger().info(
            f"TF 补链守护已启动：mode={self.mode}，探测 {self.probe_seconds:.1f}s，"
            f"关注 {self.parent_frame} → {self.child_frame}"
        )
        if self.mode == MODE_NEVER:
            self.get_logger().info("mode=never：本节点不会发布任何 TF")

    # ---------------------------------------------------------------- 监听
    def on_tf(self, message: TFMessage):
        for transform in message.transforms:
            if normalize_frame(transform.child_frame_id) != normalize_frame(self.child_frame):
                continue
            parent = transform.header.frame_id or ""
            if not is_target_link(parent, transform.child_frame_id, self.parent_frame, self.child_frame):
                # 别人发的是 base_link 的**别的**父变换（例如 odom → base_link）。
                # 只看子帧名就当成"这段链已经有人负责"，会让 auto 模式从此不补链，
                # 而 base_footprint → base_link 实际还是断的。
                if self.publishing and not self.parent_mismatch_reported:
                    self.parent_mismatch_reported = True
                    self.get_logger().warn(
                        f"收到 {parent or '<空>'} → {self.child_frame} 的变换，但本节点负责的是 "
                        f"{self.parent_frame} → {self.child_frame}，这一条不算数（继续补链）"
                    )
                continue
            stamp_ns = (
                int(transform.header.stamp.sec) * 1_000_000_000
                + int(transform.header.stamp.nanosec)
            )
            if is_ours(stamp_ns, self.published_stamps):
                continue      # 自己刚发的那一份，不算"别人在发"
            if not self.external_seen:
                self.external_seen = True
                self.external_parent = parent
                self.get_logger().info(
                    f"探测到 {self.child_frame} 的父变换已由外部发布"
                    f"（{self.external_parent} → {self.child_frame}），按 mode={self.mode} 处理"
                )
            if self.publishing and not self.conflict_reported:
                # mode=always 是"明知别人也在发也要补"的选择，此时必须把冲突喊出来，
                # 否则现场只会看到 Nav2 报多个父节点却不知道为什么。
                self.conflict_reported = True
                self.get_logger().error(
                    f"外部节点也在发布 {self.parent_frame} → {self.child_frame}，会出现 TF 冲突"
                    "（Nav2 报多个父节点）。请把站点 site.yaml 里 env 的 "
                    "GO2_BASE_FOOTPRINT_TF 设为 never 后重启服务。"
                )

    # ---------------------------------------------------------------- 决策
    def tick(self):
        elapsed_s = (self.get_clock().now() - self.started).nanoseconds * 1e-9
        if should_publish(self.mode, self.external_seen, elapsed_s, self.probe_seconds):
            if not self.publishing:
                self.publishing = True
                if self.mode == MODE_ALWAYS:
                    self.get_logger().info(
                        f"mode=always：无条件补链 {self.parent_frame} → {self.child_frame}"
                    )
                else:
                    self.get_logger().warn(
                        f"探测 {self.probe_seconds:.1f}s 内没有任何节点发布 "
                        f"{self.parent_frame} → {self.child_frame}，本节点开始补链。"
                        "这通常就是真机上「点云变不出 /scan、导航起不来」的根因；"
                        "若机器狗自身其实会发这段 TF，请把 GO2_BASE_FOOTPRINT_TF 设为 never。"
                    )
            if publish_due(self.last_published_ns, self.get_clock().now().nanoseconds, self.publish_period):
                self.publish_once()
            return
        if self.mode != MODE_NEVER and self.external_seen and not self.stand_down_logged:
            self.stand_down_logged = True
            self.get_logger().info(
                f"不补链：{self.child_frame} 的父变换已由外部发布（{self.external_parent}）。"
                "真机上这一段应由机器狗自身提供；若现场发现 TF 树是断的，"
                "把 GO2_BASE_FOOTPRINT_TF 设为 always 即可由本节点补上。"
            )

    def publish_once(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        # 按 publish_period（默认 1s）重发：静态 TF 是 latched 的，但晚加入的节点仍可能
        # 错过第一份（离线假狗里是同样的处理，见 fake_go2_loop）。
        self.broadcaster.sendTransform(transform)
        stamp_ns = (
            int(transform.header.stamp.sec) * 1_000_000_000
            + int(transform.header.stamp.nanosec)
        )
        self.last_published_ns = stamp_ns
        # 记下自己发的 stamp，on_tf 靠它把回环回来的那一份认出来（精确匹配，不用时间窗）
        self.published_stamps.append(stamp_ns)


def main(args=None):
    rclpy.init(args=args)
    node = TfChainGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
