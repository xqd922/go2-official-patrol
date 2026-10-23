#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from go2_patrol_reflective.geometry import euclidean_cluster, limit_nearest, ransac_line


def read_xyz_intensity(cloud: PointCloud2):
    names = {field.name for field in cloud.fields}
    if not {"x", "y", "z", "intensity"}.issubset(names):
        return None
    raw = pc2.read_points(cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True)
    arr = np.asarray(raw)
    if arr.size == 0:
        return None
    if arr.dtype.names:
        arr = np.column_stack([arr["x"], arr["y"], arr["z"], arr["intensity"]])
    else:
        arr = arr.reshape(-1, 4)
    return arr[np.isfinite(arr).all(axis=1)]


def filter_reflective_points(points, intensity_min, height_min, height_max, max_range):
    distance = np.linalg.norm(points[:, :2], axis=1)
    mask = (
        (points[:, 3] >= intensity_min)
        & (points[:, 2] >= height_min)
        & (points[:, 2] <= height_max)
        & (distance <= max_range)
        & (distance >= 0.15)
    )
    return points[mask]


class ReflectiveWallDetector(Node):
    """Turn high-intensity nearby line-like clusters into a virtual wall marker."""

    def __init__(self):
        super().__init__("reflective_wall_detector")
        self.declare_parameter("cloud_topic", "/utlidar/cloud_deskewed")
        self.declare_parameter("intensity_min", 30.0)
        self.declare_parameter("height_min", -0.10)
        self.declare_parameter("height_max", 0.10)
        self.declare_parameter("max_range", 6.0)
        self.declare_parameter("cluster_eps", 0.18)
        self.declare_parameter("cluster_min_points", 3)
        self.declare_parameter("min_line_points", 5)
        self.declare_parameter("inlier_distance", 0.08)
        self.declare_parameter("required_hits", 3)
        self.declare_parameter("wall_half_width", 0.10)
        self.declare_parameter("wall_height", 0.7)
        self.declare_parameter("max_wall_length", 4.0)
        # 单帧参与聚类的点数上限（按距离取最近的；保证 5.5~11Hz 下节点跟得上）
        self.declare_parameter("max_points", 10000)

        gp = self.get_parameter
        self.intensity_min = float(gp("intensity_min").value)
        self.height_min = float(gp("height_min").value)
        self.height_max = float(gp("height_max").value)
        self.max_range = float(gp("max_range").value)
        self.cluster_eps = float(gp("cluster_eps").value)
        self.cluster_min_points = int(gp("cluster_min_points").value)
        self.min_line_points = int(gp("min_line_points").value)
        self.inlier_distance = float(gp("inlier_distance").value)
        self.required_hits = int(gp("required_hits").value)
        self.wall_half_width = float(gp("wall_half_width").value)
        self.wall_height = float(gp("wall_height").value)
        self.max_wall_length = float(gp("max_wall_length").value)
        self.max_points = int(gp("max_points").value)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub_markers = self.create_publisher(MarkerArray, "/reflective_wall_markers", marker_qos)
        self.pub_active = self.create_publisher(Bool, "/reflective_wall_active", marker_qos)
        self.pub_status = self.create_publisher(String, "/reflective_wall_status", marker_qos)
        self.create_subscription(
            PointCloud2,
            gp("cloud_topic").value,
            self.on_cloud,
            sensor_qos,
        )
        self.hits = 0
        self.last_markers = MarkerArray()
        self.publish_rate = 2.0
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_status)
        self.get_logger().info("reflective wall detector started in test mode")

    def on_cloud(self, cloud: PointCloud2):
        points = read_xyz_intensity(cloud)
        if points is None:
            # 这里原来调了一个并不存在的 self.warn_once()，第一帧没有 intensity 字段时
            # 直接 AttributeError 把节点打挂；改成带节流的正常告警。
            self.get_logger().warn(
                "点云没有 x/y/z/intensity 字段，无法做反光条检测"
                "（检查 /utlidar/cloud_deskewed 的字段定义）",
                throttle_duration_sec=10.0,
            )
            self.deactivate()
            return

        points = filter_reflective_points(
            points,
            self.intensity_min,
            self.height_min,
            self.height_max,
            self.max_range,
        )
        if len(points) < self.min_line_points:
            self.deactivate()
            return

        # 先按同一规则截断，再聚类：聚类返回的标签是与截断后的点一一对应的，
        # 直接用原始数组索引会在点数超过 max_points 时下标越界。
        points = limit_nearest(points, self.max_points)
        labels = euclidean_cluster(
            points[:, :2], self.cluster_eps, self.cluster_min_points, max_points=self.max_points
        )
        candidates = []
        frame_id = cloud.header.frame_id or "base_footprint"
        for label in np.unique(labels):
            if label < 0:
                continue
            cluster = points[labels == label]
            if len(cluster) < self.min_line_points:
                continue
            line, inliers = ransac_line(
                cluster[:, :2],
                min_points_for_line=self.min_line_points,
                inlier_distance=self.inlier_distance,
            )
            if line is None:
                continue
            candidates.append((cluster, line, inliers))

        if candidates:
            self.hits += 1
            if self.hits >= self.required_hits:
                self.last_markers = self.make_markers(candidates, frame_id)
        else:
            self.deactivate()

    def deactivate(self):
        """这一步没有检测到反光条：清掉命中计数**和上一次的 Marker**。

        只归零 hits 是不够的：publish_status 以固定频率反复发布 last_markers，
        并按 `len(last_markers.markers) > 0` 算 active，于是反光条离开视野后
        /reflective_wall_active 会一直卡在 true、RViz 里那堵虚拟墙也一直画着，
        看起来像"墙还在"。三个调用点（点云缺字段、反光点太少、聚类候选为空）
        都表示"这一帧没看见墙"，所以一律清空。
        """
        self.hits = 0
        self.last_markers = MarkerArray()

    def make_markers(self, candidates, frame_id):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        index = 0
        for _, (cluster, line, _) in enumerate(candidates[:4]):
            line_points = cluster[:, :2]
            ix = np.argsort(line_points[:, 0] + line_points[:, 1])
            ordered = line_points[ix]
            start = ordered[0]
            end = ordered[-1]
            center = (start + end) / 2.0
            length = float(np.linalg.norm(end - start))
            if length > self.max_wall_length or length < 0.20:
                continue
            yaw = math.atan2(float(end[1] - start[1]), float(end[0] - start[0]))

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = now
            marker.ns = "reflective_wall"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = 0.0
            marker.pose.orientation.z = math.sin(yaw * 0.5)
            marker.pose.orientation.w = math.cos(yaw * 0.5)
            marker.scale.x = length + 0.06
            marker.scale.y = max(0.04, self.wall_half_width * 2.0)
            marker.scale.z = self.wall_height
            marker.color.r = 1.0
            marker.color.g = 0.35
            marker.color.b = 0.0
            marker.color.a = 0.65
            marker.lifetime = Duration(seconds=0.6).to_msg()
            markers.markers.append(marker)
            index += 1
        return markers

    def publish_status(self):
        self.pub_markers.publish(self.last_markers)
        active = len(self.last_markers.markers) > 0
        status = String()
        status.data = f"active={active} markers={len(self.last_markers.markers)}"
        self.pub_status.publish(status)
        active_msg = Bool()
        active_msg.data = active
        self.pub_active.publish(active_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReflectiveWallDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
