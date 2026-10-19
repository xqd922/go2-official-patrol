"""把机器人位姿与巡检点发布成 RViz Marker，方便"一眼看到机器人在哪"。

为什么需要：离线/真机都可能没有可靠的 RobotModel（URDF/robot_description），
而 RViz 光靠 TF 很难直观看出"机器人在哪、朝哪"。所以这里发布两路 Marker：

  /go2/pose_marker       机器人位置（圆）+ yaw 朝向（箭头），frame_id=map
  /patrol/waypoint_markers  巡检点 A/B/C/D + 名称，frame_id=map

关键约束（踩过的坑）：
  * 位置一律来自 TF `map -> base_footprint`，**绝不**直接拿 /odom 的 x/y。
    /odom 是 odom 系里程计（odom->base_footprint），把它当地图坐标会整体
    偏移一个 map->odom，RViz 里机器人就会画错位置。
  * 没有 TF 时不发布 Marker（RViz 显示为空），而不是假装在 (0,0)。
  * 只发布 Marker，不发布任何 TF —— 避免出现第二个 map->odom 或
    odom->base_footprint 发布者。
"""
import math
import time

MARKER_POSE_TOPIC = "/go2/pose_marker"
MARKER_WAYPOINT_TOPIC = "/patrol/waypoint_markers"

# 与 GUI 地图配色保持一致，便于两边对照
ROBOT_COLOR = (0.08, 0.40, 0.75, 1.0)      # 蓝
ROBOT_HEAD_COLOR = (0.90, 0.22, 0.21, 1.0)  # 红（朝向）
WAYPOINT_COLOR = (0.18, 0.49, 0.20, 1.0)    # 绿
WAYPOINT_HOME_COLOR = (0.94, 0.42, 0.00, 1.0)  # 橙


def yaw_to_quaternion(yaw):
    """yaw（绕 z 轴）-> (x, y, z, w)。"""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MarkerPublisher:
    """后台定时发布 pose marker 与 waypoint marker（rclpy 节点）。"""

    def __init__(self, node, map_frame="map", base_frame="base_footprint",
                 tf_buffer=None, waypoints_path=None, period=0.2, logger=print,
                 message_types=None):
        """message_types 只在测试里注入假 Marker/MarkerArray。

        生产路径仍从 visualization_msgs 取真实消息类型；把导入放在构造时
        （而不是模块顶层）是为了让本模块在没有 ROS 的机器上也能被 import，
        同时方便单测在不污染 sys.modules 的前提下替换消息类。
        """
        if message_types is None:
            from visualization_msgs.msg import Marker, MarkerArray
        else:
            Marker, MarkerArray = message_types

        self.node = node
        self.log = logger
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.buffer = tf_buffer
        self.waypoints_path = waypoints_path
        self._Marker = Marker
        self._MarkerArray = MarkerArray

        self.pose_pub = node.create_publisher(Marker, MARKER_POSE_TOPIC, 10)
        self.waypoint_pub = node.create_publisher(MarkerArray,
                                                  MARKER_WAYPOINT_TOPIC, 10)
        self._waypoints_mtime = None
        self._waypoint_cache = []
        self._last_tf_warn = 0.0

        # 用节点时钟定时；周期 0.2s 足够 RViz 流畅显示，又不会刷爆 DDS。
        self.node.create_timer(period, self.publish_all)

    # ---------- 位姿 ----------
    def lookup_map_pose(self):
        """查 map -> base_footprint，返回 (x, y, yaw) 或 None。

        用 Time()（= 0 = "取最新可用变换"）而不是 get_clock().now()：
        与 ros_bridge._refresh_pose 完全一致的取法，保证
        "GUI 显示的机器人位姿 == RViz marker 位置"。
        历史坑：早先用 now() 会因 TF 时间戳与节点实时钟存在微小错位而
        间歇性抛 LookupException，表现成 RViz 里机器人标记时有时无。
        """
        if self.buffer is None:
            return None
        try:
            try:
                from rclpy.time import Time
                when = Time()
            except Exception:
                # 没有 rclpy（只做单测/离线纯逻辑）时退化为 None：
                # tf2_ros 的 lookup_transform 接受 None 表示"最新可用变换"，
                # 语义与 Time() 完全一致。
                when = None
            transform = self.buffer.lookup_transform(
                self.map_frame, self.base_frame, when)
        except Exception:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
                         1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2))
        return float(t.x), float(t.y), yaw

    def build_pose_markers(self, pose):
        """(x, y, yaw) -> [圆盘 Marker, 箭头 Marker]，frame_id=map。"""
        x, y, yaw = pose
        stamp = self.node.get_clock().now().to_msg()

        disc = self._Marker()
        disc.header.frame_id = self.map_frame
        disc.header.stamp = stamp
        disc.ns = "go2_pose"
        disc.id = 0
        disc.type = self._Marker.CYLINDER
        disc.action = self._Marker.ADD
        disc.pose.position.x = x
        disc.pose.position.y = y
        disc.pose.position.z = 0.05
        disc.pose.orientation.w = 1.0
        disc.scale.x = 0.30
        disc.scale.y = 0.30
        disc.scale.z = 0.10
        disc.color.r, disc.color.g, disc.color.b, disc.color.a = ROBOT_COLOR

        arrow = self._Marker()
        arrow.header.frame_id = self.map_frame
        arrow.header.stamp = stamp
        arrow.ns = "go2_pose"
        arrow.id = 1
        arrow.type = self._Marker.ARROW
        arrow.action = self._Marker.ADD
        arrow.pose.position.x = x
        arrow.pose.position.y = y
        arrow.pose.position.z = 0.15
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        arrow.pose.orientation.x = qx
        arrow.pose.orientation.y = qy
        arrow.pose.orientation.z = qz
        arrow.pose.orientation.w = qw
        # ARROW 沿 +x 指向，正好对应 yaw=0 朝前
        arrow.scale.x = 0.60
        arrow.scale.y = 0.09
        arrow.scale.z = 0.09
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = ROBOT_HEAD_COLOR
        return [disc, arrow]

    # ---------- 巡检点 ----------
    def _load_waypoints(self):
        """读 waypoints.yaml；文件 mtime 没变就复用缓存，避免每 0.2s 读盘。"""
        import os
        path = self.waypoints_path
        if not path or not os.path.isfile(path):
            return []
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return self._waypoint_cache
        if mtime == self._waypoints_mtime:
            return self._waypoint_cache
        try:
            import yaml
            with open(path, "r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
            items = data.get("waypoints", []) or []
        except Exception as exc:
            self.log(f"[marker] 读取 waypoints 失败：{exc}")
            return self._waypoint_cache
        self._waypoint_cache = items
        self._waypoints_mtime = mtime
        return items

    def build_waypoint_markers(self, items):
        """waypoints.yaml 的条目 -> MarkerArray（A/B/C… + 名称）。"""
        array = self._MarkerArray()
        stamp = self.node.get_clock().now().to_msg()
        for index, item in enumerate(items):
            try:
                x = float(item["x"])
                y = float(item["y"])
            except (KeyError, TypeError, ValueError):
                continue
            name = str(item.get("name", f"wp{index}"))
            home = bool(item.get("home", False))
            label = chr(ord("A") + index) if index < 26 else str(index + 1)

            point = self._Marker()
            point.header.frame_id = self.map_frame
            point.header.stamp = stamp
            point.ns = "patrol_waypoints"
            point.id = index
            point.type = self._Marker.SPHERE
            point.action = self._Marker.ADD
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.position.z = 0.15
            point.pose.orientation.w = 1.0
            point.scale.x = point.scale.y = 0.22
            point.scale.z = 0.22
            if home:
                point.color.r, point.color.g, point.color.b, point.color.a = WAYPOINT_HOME_COLOR
            else:
                point.color.r, point.color.g, point.color.b, point.color.a = WAYPOINT_COLOR

            text = self._Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = stamp
            text.ns = "patrol_waypoint_labels"
            text.id = index
            text.type = self._Marker.TEXT_VIEW_FACING
            text.action = self._Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.55
            text.pose.orientation.w = 1.0
            text.scale.z = 0.22
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            # 字母 + 名称，和 GUI 地图上的标注一致
            text.text = f"{label} {name}"
            array.markers.append(point)
            array.markers.append(text)

        # 空数组也要发一次，否则 RViz 会残留上一次的旧航点
        return array

    # ---------- 定时入口 ----------
    def publish_all(self):
        pose = self.lookup_map_pose()
        if pose is not None:
            markers = self.build_pose_markers(pose)
            for marker in markers:
                self.pose_pub.publish(marker)
        else:
            now = time.monotonic()
            if now - self._last_tf_warn > 10.0:
                self._last_tf_warn = now
                self.log(f"[marker] 暂无可用的 {self.map_frame}->{self.base_frame} TF，"
                         f"暂不发布机器人标记（RViz 中不会显示机器人）")
        self.waypoint_pub.publish(self.build_waypoint_markers(self._load_waypoints()))
