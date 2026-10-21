#!/usr/bin/env python3
"""把机载点云的时间戳对齐到本机时钟。

为什么需要这个节点
------------------
Go2 的点云与状态都带**机器狗自己的时钟**，而机器狗起机后没有可靠对时
（离线现场常见：比电脑慢几分钟）。这些消息进入 ROS 后，TF 缓冲按本机时钟
记录，于是 `pointcloud_to_laserscan` 的 message filter 会把每一帧都判成
"比 transform 缓存里所有数据都旧" 并丢弃：

    Message Filter dropping message: frame 'odom' at time ... for reason
    'the timestamp on the message is earlier than all the data in the transform cache'

表现是点云 `/utlidar/cloud_deskewed` 有频率、`/scan` 却一帧都没有，
Nav2/AMCL 整条链起不来。离线假狗用本机时钟发包，所以离线验收永远测不出来。

节点对每一帧做 `stamp += (本机接收时刻 - 该帧 stamp)` 的**平滑**修正：
单帧抖动只影响一点点，真实时钟偏差会在几帧内收敛；不做硬跳变，
避免在 TF 里产生时间倒流。
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def seconds_to_stamp(stamp, seconds: float) -> None:
    nanoseconds = int(round(max(0.0, seconds) * 1e9))
    stamp.sec = nanoseconds // 1_000_000_000
    stamp.nanosec = nanoseconds % 1_000_000_000


def next_offset(current, measured, smoothing, hard_sync_threshold):
    """这一帧要用的时间修正量。

    current 为 None 表示第一次见到点云：直接对齐。
    偏差突然跳变（狗重启、时钟被改）也直接对齐，否则要几十帧才追上。
    其余情况按 smoothing 慢慢收敛，避免单帧抖动写进 TF 造成时间倒流。
    返回值第二项表示是否发生了"硬对齐"，供调用方决定要不要喊一声。
    """
    if current is None or abs(measured - current) > hard_sync_threshold:
        return measured, True
    return current + smoothing * (measured - current), False


class CloudTimeSync(Node):
    """按"接收时刻 - 消息时刻"的滑动估计，把点云时间戳平移到本机时钟。"""

    def __init__(self):
        super().__init__("go2_cloud_time_sync")
        self.declare_parameter("input_topic", "/utlidar/cloud_deskewed")
        self.declare_parameter("output_topic", "/utlidar/cloud_deskewed_ros")
        # 0.2 表示每帧只追赶 20% 的偏差：295s 的偏差约 40 帧（~3s）收敛，
        # 单帧抖动（几毫秒）几乎看不出来。
        self.declare_parameter("smoothing", 0.2)
        # 超过这个偏差就直接对齐（起机首次、狗重启后时钟跳变），否则要等几十帧。
        self.declare_parameter("hard_sync_threshold", 5.0)
        self.declare_parameter("max_correction", 86400.0)

        self.smoothing = float(self.get_parameter("smoothing").value)
        self.hard_sync_threshold = float(self.get_parameter("hard_sync_threshold").value)
        self.max_correction = float(self.get_parameter("max_correction").value)
        self.offset = None
        self.frames = 0

        self.pub = self.create_publisher(PointCloud2, self.get_parameter("output_topic").value, 10)
        self.create_subscription(
            PointCloud2,
            self.get_parameter("input_topic").value,
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"点云时间戳对齐已启动：{self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}"
        )

    def on_cloud(self, cloud: PointCloud2):
        now = time.time()
        measured = now - stamp_to_seconds(cloud.header.stamp)

        if abs(measured) > self.max_correction:
            self.get_logger().warn(
                f"点云时间偏差 {measured:+.1f}s 超过上限 {self.max_correction:.0f}s，本帧不改时间戳",
                throttle_duration_sec=5.0,
            )
            self.pub.publish(cloud)
            return

        offset, hard = next_offset(
            self.offset, measured, self.smoothing, self.hard_sync_threshold
        )
        if hard:
            if self.offset is None:
                self.get_logger().info(
                    f"点云与电脑时钟相差 {measured:+.1f}s，开始按本机时钟重打时间戳"
                )
            else:
                self.get_logger().warn(
                    f"点云时钟偏差跳变 {self.offset:+.1f}s -> {measured:+.1f}s，直接对齐",
                    throttle_duration_sec=5.0,
                )
        self.offset = offset

        seconds_to_stamp(cloud.header.stamp, stamp_to_seconds(cloud.header.stamp) + self.offset)
        self.pub.publish(cloud)
        self.frames += 1


def main(args=None):
    rclpy.init(args=args)
    node = CloudTimeSync()
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
