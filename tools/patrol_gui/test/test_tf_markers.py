"""机器人/巡检点 Marker：位置必须来自 map 坐标系，且绝不发布 TF。

关键点（对应 RViz 里"看不到机器人 / 机器人画错位置"的两类事故）：
  * 位置来源是 TF `map -> base_footprint`，**不是** /odom 的 x/y。
    直接拿 /odom 当地图坐标会整体差一个 map->odom 偏移。
  * 没有 TF 时**不发布** Marker（RViz 里空白），而不是假装在 (0,0)。
  * 只发 Marker，不发任何 TF，避免出现第二个 map->odom /
    odom->base_footprint 发布者。

这里通过 `message_types=` 注入假 Marker/MarkerArray，而不是往 sys.modules
里塞桩：同一 pytest 进程里还有别的测试目录需要真实的 std_msgs / geometry_msgs
（例如 patrol_manager 用 std_msgs.String），污染 sys.modules 会连带把它们打挂。
"""
import math

import pytest

from patrol_gui import tf_markers


# ---------------- 假消息类型（只存字段，不做校验） ----------------

class _Bag:
    pass


class FakeMarker:
    CYLINDER = 3
    ARROW = 0
    SPHERE = 2
    TEXT_VIEW_FACING = 9
    ADD = 0

    def __init__(self):
        self.header = _Bag()
        self.header.frame_id = ""
        self.header.stamp = None
        self.ns = ""
        self.id = 0
        self.type = 0
        self.action = 0
        self.pose = _Bag()
        self.pose.position = _Bag()
        self.pose.orientation = _Bag()
        self.scale = _Bag()
        self.color = _Bag()
        self.text = ""


class FakeMarkerArray:
    def __init__(self):
        self.markers = []


FAKE_TYPES = (FakeMarker, FakeMarkerArray)


# ---------------- 假 TF / 节点 ----------------

class _Vec:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class _Transform:
    def __init__(self, x, y, yaw):
        self.transform = _Vec(
            translation=_Vec(x=x, y=y, z=0.0),
            rotation=_Vec(x=0.0, y=0.0, z=math.sin(yaw / 2.0),
                          w=math.cos(yaw / 2.0)),
        )


class _FakeBuffer:
    def __init__(self, transform=None):
        self.transform = transform
        self.lookups = []
        self.time_points = []

    def lookup_transform(self, parent, child, time_point=None):
        self.lookups.append((parent, child))
        self.time_points.append(time_point)
        if self.transform is None:
            raise RuntimeError("no transform")
        return self.transform


class _Clock:
    class _Now:
        def to_msg(self):
            return "stamp"

    def now(self):
        return _Clock._Now()


class _FakeNode:
    """最小 rclpy Node 替身：记录 publisher / timer，绝不提供 TF 广播接口。"""

    def __init__(self):
        self.published = []        # (topic, msg)
        self.topics = []
        self.timers = []
        self.clock_calls = 0       # 记录 get_clock() 调用次数

    def create_publisher(self, msg_type, topic, qos):
        self.topics.append(topic)

        class _Pub:
            def __init__(self, owner, topic_name):
                self.owner = owner
                self.topic = topic_name

            def publish(self, msg):
                self.owner.published.append((self.topic, msg))

        return _Pub(self, topic)

    def create_timer(self, period, callback):
        self.timers.append((period, callback))
        return object()

    def get_clock(self):
        self.clock_calls += 1
        return _Clock()


# ---------------- 夹具 ----------------

def _publisher(node, buffer, waypoints_path=None, **kw):
    return tf_markers.MarkerPublisher(node, tf_buffer=buffer,
                                      waypoints_path=waypoints_path,
                                      message_types=FAKE_TYPES,
                                      logger=lambda *_: None, **kw)


# ---------------- 机器人位姿 marker ----------------

def test_pose_marker_uses_map_coordinates_and_yaw():
    node = _FakeNode()
    buffer = _FakeBuffer(_Transform(1.25, -0.75, 1.57))
    pub = _publisher(node, buffer)

    pose = pub.lookup_map_pose()
    assert pose is not None
    x, y, yaw = pose
    assert abs(x - 1.25) < 1e-9 and abs(y - (-0.75)) < 1e-9
    assert abs(yaw - 1.57) < 1e-6
    # 查的必须是 map -> base_footprint，而不是 odom
    assert buffer.lookups[-1] == ("map", "base_footprint")

    markers = pub.build_pose_markers(pose)
    disc, arrow = markers[0], markers[1]
    assert disc.header.frame_id == "map"
    assert disc.ns == "go2_pose" and disc.id == 0
    assert abs(disc.pose.position.x - 1.25) < 1e-9
    assert abs(disc.pose.position.y - (-0.75)) < 1e-9
    # 朝向必须在箭头 Marker 上（yaw=1.57 -> 绕 z 约 90°）
    assert arrow.header.frame_id == "map" and arrow.id == 1
    assert abs(arrow.pose.orientation.z - math.sin(1.57 / 2.0)) < 1e-6
    assert abs(arrow.pose.orientation.w - math.cos(1.57 / 2.0)) < 1e-6


def test_no_tf_means_no_pose_marker():
    """没有 map->base_footprint 时不能假装机器人在 (0,0)。"""
    node = _FakeNode()
    pub = _publisher(node, _FakeBuffer(None))
    assert pub.lookup_map_pose() is None
    pub.publish_all()
    poses = [m for t, m in node.published if t == tf_markers.MARKER_POSE_TOPIC]
    assert poses == []


def test_no_buffer_means_no_pose_marker():
    node = _FakeNode()
    pub = _publisher(node, None)
    assert pub.lookup_map_pose() is None


def test_publish_all_emits_pose_markers_when_tf_available():
    node = _FakeNode()
    pub = _publisher(node, _FakeBuffer(_Transform(2.0, 0.0, 0.0)))
    pub.publish_all()
    poses = [m for t, m in node.published if t == tf_markers.MARKER_POSE_TOPIC]
    assert [m.id for m in poses] == [0, 1]
    assert all(m.header.frame_id == "map" for m in poses)


def test_pose_marker_self_heals_when_tf_appears_later():
    """TF 一开始不可用（RViz 暂不画机器人）→ 之后可解析，Marker 自动跟上。

    对应本次修复：不能因为"启动时 TF 还没就绪"就永久不显示机器人。
    只要 TF 后来可解析，后续 publish_all 必须开始发布 Marker，
    并且位置跟着最新 map->base_footprint 走。
    """
    node = _FakeNode()
    buf = _FakeBuffer(None)                 # 一开始查不到
    pub = _publisher(node, buf)

    pub.publish_all()
    assert [m for t, m in node.published
            if t == tf_markers.MARKER_POSE_TOPIC] == []   # 暂无 Marker

    buf.transform = _Transform(0.5, 0.0, 0.0)             # TF 就绪
    pub.publish_all()
    poses = [m for t, m in node.published if t == tf_markers.MARKER_POSE_TOPIC]
    assert [m.id for m in poses] == [0, 1]
    assert abs(poses[0].pose.position.x - 0.5) < 1e-9

    buf.transform = _Transform(2.0, -0.1, 0.05)           # 机器人移动
    pub.publish_all()
    latest = [m for t, m in node.published
              if t == tf_markers.MARKER_POSE_TOPIC][-2:]
    assert abs(latest[0].pose.position.x - 2.0) < 1e-9
    assert abs(latest[0].pose.position.y - (-0.1)) < 1e-9


def test_no_tf_broadcaster_is_created():
    """不能发布任何 TF（否则就是第二个 map->odom / odom->base_footprint）。"""
    node = _FakeNode()
    pub = _publisher(node, _FakeBuffer(_Transform(0.0, 0.0, 0.0)))
    assert not hasattr(pub, "tf_broadcaster")
    assert not hasattr(pub, "send_transform")
    names = [n for n in dir(pub) if "broadcast" in n.lower() or n.endswith("_tf")]
    assert names == []


def test_only_marker_topics_are_created():
    node = _FakeNode()
    _publisher(node, _FakeBuffer(_Transform(0.0, 0.0, 0.0)))
    assert set(node.topics) == {tf_markers.MARKER_POSE_TOPIC,
                                tf_markers.MARKER_WAYPOINT_TOPIC}


# ---------------- 巡检点 marker ----------------

def _write_waypoints(tmp_path):
    path = tmp_path / "waypoints.yaml"
    path.write_text(
        "waypoints:\n"
        "  - name: entrance\n"
        "    x: 0.0\n"
        "    y: 0.0\n"
        "    yaw: 0.0\n"
        "    dwell: 4.0\n"
        "    home: true\n"
        "  - name: equipment_zone\n"
        "    x: 2.0\n"
        "    y: 0.0\n"
        "    yaw: 0.0\n"
        "    dwell: 4.0\n", encoding="utf-8")
    return str(path)


def test_waypoint_markers_carry_letters_and_names(tmp_path):
    node = _FakeNode()
    path = _write_waypoints(tmp_path)
    pub = _publisher(node, _FakeBuffer(None), waypoints_path=path)
    pub.publish_all()
    arrays = [m for t, m in node.published
              if t == tf_markers.MARKER_WAYPOINT_TOPIC]
    assert arrays, "必须发布 /patrol/waypoint_markers"
    markers = arrays[-1].markers
    texts = [m.text for m in markers if m.type == FakeMarker.TEXT_VIEW_FACING]
    assert "A entrance" in texts
    assert "B equipment_zone" in texts
    # 全部在 map 坐标系
    assert all(m.header.frame_id == "map" for m in markers)
    # 位置就是 waypoints.yaml 里的 x/y
    spheres = [m for m in markers if m.type == FakeMarker.SPHERE]
    assert abs(spheres[0].pose.position.x - 0.0) < 1e-9
    assert abs(spheres[1].pose.position.x - 2.0) < 1e-9


def test_waypoint_home_is_highlighted(tmp_path):
    node = _FakeNode()
    path = _write_waypoints(tmp_path)
    pub = _publisher(node, _FakeBuffer(None), waypoints_path=path)
    pub.publish_all()
    array = [m for t, m in node.published
             if t == tf_markers.MARKER_WAYPOINT_TOPIC][-1]
    spheres = [m for m in array.markers if m.type == FakeMarker.SPHERE]
    home, other = spheres[0], spheres[1]
    assert (home.color.r, home.color.g, home.color.b) == tf_markers.WAYPOINT_HOME_COLOR[:3]
    assert (other.color.r, other.color.g, other.color.b) == tf_markers.WAYPOINT_COLOR[:3]


def test_empty_waypoints_still_publishes_empty_array(tmp_path):
    """空数组也要发，否则 RViz 会残留上一次的旧航点。"""
    node = _FakeNode()
    pub = _publisher(node, _FakeBuffer(None),
                     waypoints_path=str(tmp_path / "missing.yaml"))
    pub.publish_all()
    arrays = [m for t, m in node.published
              if t == tf_markers.MARKER_WAYPOINT_TOPIC]
    assert arrays and arrays[-1].markers == []


def test_yaw_to_quaternion():
    qx, qy, qz, qw = tf_markers.yaw_to_quaternion(0.0)
    assert (qx, qy, qz) == (0.0, 0.0, 0.0) and abs(qw - 1.0) < 1e-12
    qx, qy, qz, qw = tf_markers.yaw_to_quaternion(math.pi)
    assert abs(qz - 1.0) < 1e-9 and abs(qw) < 1e-9

def test_pose_marker_lookup_does_not_use_node_clock():
    """必须用 Time()（=最新可用变换）查 TF，而不是 node.get_clock().now()。

    历史坑：早先用 get_clock().now() 会因 TF 时间戳与实时钟的微小错位间歇性
    抛 LookupException，表现为 RViz 机器人标记时有时无；且与
    ros_bridge._refresh_pose（用 Time()）取法不一致，可能导致 GUI 与 RViz
    显示的机器人位置不同。这里不 import rclpy（沙箱/未 source 环境也能跑），
    改为断言"查 TF 时没有用到节点实时钟"这一可观察契约。
    """
    node = _FakeNode()
    buffer = _FakeBuffer(_Transform(0.5, 0.5, 0.0))
    pub = _publisher(node, buffer)
    assert pub.lookup_map_pose() is not None
    # FakeNode.get_clock() 被调用说明用到了实时钟 —— 必须为 0 次。
    assert node.clock_calls == 0
    # 查的仍是 map -> base_footprint
    assert buffer.lookups[-1] == ("map", "base_footprint")
