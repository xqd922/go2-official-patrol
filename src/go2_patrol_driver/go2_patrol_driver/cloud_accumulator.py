#!/usr/bin/env python3
"""把 Go2 的多帧点云累积成一帧"完整扫描"，供 pointcloud_to_laserscan 使用。

为什么需要这个节点
------------------
Go2 的 L1 雷达是**非重复扫描**（Livox 类）：每帧约 65 ms、1400 个点，
只覆盖 242/360 个方位（实测 30 帧：中位数 242，最少 234），相邻两帧的
方位重叠只有 65%。单帧直接转 LaserScan 的问题不是"点少"，而是**同一方位
在不同帧里时有时无**：

    25 帧 /scan 统计：723 个 bin 里只有 90 个在 >=80% 的帧里出现过；
    这些稳定 bin 的距离标准差中位数 0.15 m、相对标准差 37%。

`slam_toolbox` 每帧都拿这样一份"随机缺 1/3 方位、且距离抖动"的扫描去做
匹配，位姿自然来回跳，地图就糊成放射状散点（实测：静止不动 10 s，
`map->odom` 也在原地反复修正）。

累积 10 帧后方位覆盖 360/360，墙面连成线。所以这里按"最近 N 帧"滚动
合并，把每个点变换到**当前时刻**的 base_frame 再发布：

  * 必须做变换。累积窗口内有 0.7 s，机器狗以 0.2 m/s 走也会移动 14 cm，
    直接把不同时刻的原始坐标拼起来会把墙糊成厚带。
  * 用 TF（odom -> base_footprint）而不是里程计消息：TF 与点云共用一条
    时间轴，cloud_time_sync 已经把 stamp 对齐到本机时钟，查得到。
  * 输出 frame_id 就填 base_frame，`pointcloud_to_laserscan` 再转一次
    base_footprint 也无所谓（恒等）。填 odom 反而会让下游误以为是世界系。

安全：本节点只读点云和 TF，不发布任何速度/运动指令。
"""
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener


def xyz_offsets(cloud: PointCloud2):
    """按字段名取 x/y/z 的字节偏移；缺失时返回 None。"""
    offsets = {}
    for field in cloud.fields:
        if field.name in ("x", "y", "z"):
            offsets[field.name] = field.offset
    if len(offsets) != 3:
        return None
    return offsets["x"], offsets["y"], offsets["z"]


def read_xyz(cloud: PointCloud2):
    """读出 (N, 3) float32 坐标；字段不符时返回 None。"""
    offsets = xyz_offsets(cloud)
    if offsets is None:
        return None
    step = cloud.point_step
    total = len(cloud.data) // step
    if total == 0:
        return None
    raw = np.frombuffer(cloud.data, dtype=np.uint8, count=total * step).reshape(total, step)
    return np.stack(
        [raw[:, off : off + 4].copy().view(np.float32).ravel() for off in offsets],
        axis=1,
    )


def yaw_of(transform) -> float:
    q = transform.rotation
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def transform_points(points: np.ndarray, transform) -> np.ndarray:
    """把点从 transform 的子坐标系变换到父坐标系（只处理平面 yaw + 平移）。

    Go2 在平地上跑，roll/pitch 不到 1 度，只用 yaw 足够；带上 z 平移是为了
    将来换带高度的点云时不至于静默丢信息。
    """
    yaw = yaw_of(transform)
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    x = points[:, 0] * cos_y - points[:, 1] * sin_y + transform.translation.x
    y = points[:, 0] * sin_y + points[:, 1] * cos_y + transform.translation.y
    z = points[:, 2] + transform.translation.z
    return np.stack([x, y, z], axis=1)


def lookup_with_grace(buffer, target: str, source: str, stamp, timeout: float, grace: float):
    """查 TF；精确时刻落空时按 grace 一点点往前退，返回 (transform, 实际退了多少秒)。

    为什么必须退：点云打的是"扫描起始"时间戳，而 `odom -> base_footprint`
    由 sport state（约 20 Hz）转出来，最新一条永远晚几毫秒。直接查点云时刻
    会稳定报 "extrapolation into the future"（实测：点云 stamp 比最新 TF
    还新 2 ms），于是每一帧都被丢掉、`/scan` 一条不出。

    退的代价是位姿有 up to grace 的滞后；grace 取 0.1 s 时，0.2 m/s 下不过
    2 cm，对 5 cm 栅格没有可见影响。查不到才返回 None。

    返回的是 `TransformStamped.transform`，调用方直接取 translation/rotation。
    """
    deadline = time.monotonic() + max(0.0, timeout)
    step = 0.02
    back = 0.0
    while True:
        try:
            stamped = buffer.lookup_transform(
                target, source, stamp - Duration(seconds=back), Duration(seconds=0.0)
            )
            return stamped.transform, back
        except Exception:  # noqa: BLE001 - 依次尝试更早的时刻
            if back >= grace or time.monotonic() > deadline:
                return None, back
            back = min(grace, back + step)


def build_cloud(points: np.ndarray, stamp, frame_id: str) -> PointCloud2:
    """把 (N,3) float32 打包成 XYZI 的 PointCloud2。

    intensity 固定 0：下游 pointcloud_to_laserscan 不用它；保留字段是为了
    兼容"必须有 intensity"的旧版 RViz 插件。
    """
    points = np.asarray(points, dtype=np.float32)
    cloud = PointCloud2()
    cloud.header.stamp = stamp
    cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = int(len(points))
    cloud.point_step = 16
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    packed = np.zeros((len(points), 4), dtype=np.float32)
    if len(points):
        packed[:, :3] = points
    cloud.data = packed.tobytes()
    return cloud


class CloudAccumulator(Node):
    def __init__(self):
        super().__init__("go2_cloud_accumulator")
        self.declare_parameter("input_topic", "/utlidar/cloud_deskewed_ros")
        self.declare_parameter("output_topic", "/utlidar/cloud_accumulated")
        self.declare_parameter("base_frame", "base_footprint")
        # 10 帧 = 0.7 s。实测 10 帧方位覆盖 360/360；再少就会出现空洞，
        # 再多则 0.2 m/s 下走动时旧帧拖影明显。
        self.declare_parameter("window_frames", 10)
        # 只在"够一帧的量"时才落盘：按时间滑窗的话，机器人静止时同一批点会被
        # 重复累加 10 次，强度/密度失真；按位移 + 时间双门限更贴近雷达的
        # 实际扫描进度。0.02 m / 0.03 s 约等于 L1 一个扫描周期的量。
        self.declare_parameter("min_travel_distance", 0.02)
        self.declare_parameter("min_time_interval", 0.03)
        # 单点最多向前兼容多久的 TF（秒）。TF 查不到就丢这一帧，不瞎猜变换。
        self.declare_parameter("tf_timeout", 0.2)
        # 精确时刻查不到时最多往回退多久（秒），见 lookup_with_grace 的注释。
        self.declare_parameter("tf_grace", 0.1)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.window = max(1, int(self.get_parameter("window_frames").value))
        self.min_travel = float(self.get_parameter("min_travel_distance").value)
        self.min_interval = float(self.get_parameter("min_time_interval").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.tf_grace = float(self.get_parameter("tf_grace").value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.frames = []  # [(stamp, np.ndarray(N,3), 该帧时的 base 位置)]
        self.last_pose = None
        self.dropped_tf = 0
        self.cached = None  # 上次合并结果（np.ndarray），窗口没变就复用

        self.pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("output_topic").value), 5
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("input_topic").value),
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"点云累积已启动：{self.get_parameter('input_topic').value} -> "
            f"{self.get_parameter('output_topic').value}，窗口 {self.window} 帧"
        )

    def on_cloud(self, cloud: PointCloud2):
        points = read_xyz(cloud)
        if points is None:
            return
        stamp = Time.from_msg(cloud.header.stamp)

        # 当前帧时刻的 base_frame 坐标系作为输出坐标系
        target, _ = lookup_with_grace(
            self.buffer, self.base_frame, cloud.header.frame_id, stamp, self.tf_timeout, self.tf_grace
        )
        if target is None:
            self.dropped_tf += 1
            self.get_logger().warn(
                f"查不到 {self.base_frame} <- {cloud.header.frame_id} 的 TF，丢弃本帧"
                f"（累计 {self.dropped_tf} 帧）",
                throttle_duration_sec=5.0,
            )
            return

        # 位姿门限：够一段位移或够一段时间才收进窗口
        pose = (target.translation.x, target.translation.y, yaw_of(target))
        accept = True
        if self.last_pose is not None and self.frames:
            prev_stamp, _, prev_pose = self.frames[-1]
            moved = np.hypot(pose[0] - prev_pose[0], pose[1] - prev_pose[1])
            turned = abs(pose[2] - prev_pose[2])
            elapsed = (stamp - prev_stamp).nanoseconds * 1e-9
            if moved < self.min_travel and turned < self.min_travel and elapsed < self.min_interval:
                accept = False
        if accept:
            self.last_pose = pose
            self.frames.append((stamp, points, pose))
            while len(self.frames) > self.window:
                self.frames.pop(0)
            self.cached = None

        if self.cached is None:
            merged = []
            for frame_stamp, frame_points, _ in self.frames:
                transform, _ = lookup_with_grace(
                    self.buffer,
                    self.base_frame,
                    cloud.header.frame_id,
                    frame_stamp,
                    self.tf_timeout,
                    self.tf_grace,
                )
                if transform is None:  # 旧帧的 TF 已滚出缓冲
                    continue
                merged.append(transform_points(frame_points, transform))

            if not merged:
                return
            self.cached = np.concatenate(merged, axis=0)

        # 即使这一帧没被收进窗口，也要用新时间戳重发一次：
        # twist_bridge 的 require_scan 看的是 /scan 是否新鲜，静止时断流会被
        # 判成"雷达挂了"而拒绝执行遥控。内容不变、时间戳更新才是正确语义。
        self.pub.publish(build_cloud(self.cached, cloud.header.stamp, self.base_frame))


def main(args=None):
    rclpy.init(args=args)
    node = CloudAccumulator()
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
