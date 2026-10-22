#!/usr/bin/env python3
"""把机载点云的时间戳对齐到本机时钟，并去掉缓冲区里的空点。

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

为什么还要滤空点
----------------
`/utlidar/cloud_deskewed` 每帧固定 1 万多个点，但其中**前 10000 个点坐标恒为
`(0, 0, 0)`**，真实回波只有 500~600 个，全部排在缓冲区尾部（实测 30 帧：
总数中位数 10584，非零点中位数 584）。这些零点不是缺失值：`is_dense=True`、
也没有 NaN，`pointcloud_to_laserscan` 会把它们当成"距离 0 的障碍物"，
再被 `range_min` 裁到 0.15 m，于是在机器人周围糊出一圈**并不存在的墙**。
建图表现为"放射状散点/原地一圈毛边"，slam_toolbox 用这种扫描做匹配会漂。

所以这里顺手把全零点删掉并按 `point_step` 重打包（`width` 同步改小）。
保留原有字段布局，不依赖具体字段名，换雷达型号也不会错位。
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


def xyz_offsets(cloud: PointCloud2):
    """返回 (x_offset, y_offset, z_offset)；字段缺失时返回 None。

    按字段名查偏移，而不是假定 0/4/8：Go2 现在是 x/y/z/intensity，
    换固件后可能多出 ring/time 或调整顺序。
    """
    offsets = {}
    for field in cloud.fields:
        if field.name in ("x", "y", "z"):
            offsets[field.name] = field.offset
    if len(offsets) != 3:
        return None
    return offsets["x"], offsets["y"], offsets["z"]


def keep_mask(cloud: PointCloud2, zero_eps: float = 1e-4):
    """返回每个点是否要保留的布尔数组；字段不全时返回 None。

    只把"三个轴都约等于 0"判为空点。真实的近距离回波（比如贴脸的墙）
    至少会有一个轴明显非零，不会误删。

    用 numpy 向量化是必须的：一帧 1 万点、15 Hz，纯 Python 循环要
    45 万次 struct 解包/秒，在 WSL 里会白吃半个核，还会拖慢整条建图链。
    """
    xyz = xyz_offsets(cloud)
    if xyz is None:
        return None

    import numpy as np

    step = cloud.point_step
    total = len(cloud.data) // step
    if total == 0:
        return np.zeros(0, dtype=bool)

    # 把整段缓冲区当成 (total, point_step) 的 uint8 矩阵，再按字段偏移取出
    # 4 字节 float32。比 numpy structured dtype 更省事：point_step 不必被
    # 4 整除、尾部 padding 也不影响。
    raw = np.frombuffer(cloud.data, dtype=np.uint8, count=total * step).reshape(total, step)
    keep = np.zeros(total, dtype=bool)
    for offset in xyz:
        column = raw[:, offset : offset + 4].copy().view(np.float32).ravel()
        keep |= np.abs(column) > zero_eps
    return keep


def compact_points(cloud: PointCloud2, zero_eps: float = 1e-4):
    """删掉全零填充点，返回 (新 data, 有效点数)；字段不全时返回 None。"""
    keep = keep_mask(cloud, zero_eps)
    if keep is None:
        return None

    import numpy as np

    step = cloud.point_step
    kept = int(keep.sum())
    raw = np.frombuffer(cloud.data, dtype=np.uint8, count=keep.size * step).reshape(keep.size, step)
    return raw[keep].tobytes(), kept


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
        # 全零填充点的判定阈值（米）。实测这类点坐标精确为 0.0，留 1e-4 余量
        # 足够；设成 0 反而可能因驱动写出 1e-8 这类尘埃值而漏删。
        self.declare_parameter("drop_zero_points", True)
        self.declare_parameter("zero_epsilon", 1e-4)

        self.smoothing = float(self.get_parameter("smoothing").value)
        self.hard_sync_threshold = float(self.get_parameter("hard_sync_threshold").value)
        self.max_correction = float(self.get_parameter("max_correction").value)
        self.drop_zero_points = bool(self.get_parameter("drop_zero_points").value)
        self.zero_epsilon = float(self.get_parameter("zero_epsilon").value)
        self.offset = None
        self.frames = 0
        self.dropped_total = 0

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

        if self.drop_zero_points:
            compacted = compact_points(cloud, self.zero_epsilon)
            if compacted is not None:
                data, kept = compacted
                dropped = len(cloud.data) // cloud.point_step - kept
                if dropped:
                    cloud.data = data
                    cloud.width = kept
                    cloud.height = 1
                    cloud.row_step = kept * cloud.point_step
                    self.dropped_total += dropped

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
        if self.frames % 300 == 0:
            self.get_logger().info(
                f"已转发 {self.frames} 帧，累计剔除空点 {self.dropped_total} 个"
            )


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
