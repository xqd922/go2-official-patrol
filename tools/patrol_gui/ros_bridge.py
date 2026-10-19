"""GUI 与 ROS 2 之间的桥：位姿 / 巡检状态 / 服务调用。

线程模型：rclpy 在一个后台线程里 spin，GUI 线程只读写受锁保护的缓存，
因此界面永远不会被 DDS 回调阻塞。所有 ROS import 都延迟到启动时，
让无 ROS 的环境也能 import 本模块做单测。
"""
import threading
import time

from . import map_pose, proc  # noqa: F401  (re-export 便于调用方统一 import)


def latched_qos():
    """与 map_server 的 /map 匹配的 latched(TRANSIENT_LOCAL) QoS。

    单独抽成函数便于单测：map_server 只在地图加载时发一次 /map，订阅端必须
    用 TRANSIENT_LOCAL 才能拿到"订阅之前"已发布的那条；用默认 volatile 会
    永远收不到，导致 GUI 误报"地图未加载"。
    """
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    return QoSProfile(
        depth=1, history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class RosBridge:
    """后台 spin 的 ROS 节点：缓存最新 map 位姿、巡检状态、当前目标。"""

    def __init__(self, node_name="patrol_gui", map_frame="map",
                 base_frame="base_footprint", logger=print,
                 waypoints_path=None, publish_markers=True):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        self._rclpy = rclpy
        self.log = logger
        self.map_frame = map_frame
        self.base_frame = base_frame

        self.node = Node(node_name)
        self._lock = threading.Lock()
        self._pose = None          # (x, y, yaw)
        self._pose_stamp = 0.0
        self._status = "IDLE"      # 巡检状态（由 patrol_manager 的 /robot/status 推导）
        self._goal = ""            # 当前目标点
        self._running = True
        self._tf_error = None

        from geometry_msgs.msg import PoseStamped
        from std_srvs.srv import Trigger
        from tf2_ros import Buffer, TransformListener

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self.node)

        # 把 map->base_footprint 的位姿同时发布成 RViz Marker，
        # 让 RViz 里也能"一眼看到机器人在哪、朝哪"。位置来源与 GUI 完全一致。
        self.markers = None
        if publish_markers:
            try:
                from .tf_markers import MarkerPublisher
                self.markers = MarkerPublisher(
                    self.node, map_frame=map_frame, base_frame=base_frame,
                    tf_buffer=self._buffer, waypoints_path=waypoints_path,
                    logger=logger)
                self.log(f"[gui] 已启用 RViz 标记发布：/go2/pose_marker、"
                         f"/patrol/waypoint_markers")
            except Exception as exc:
                self.log(f"[gui] RViz 标记发布不可用（不影响其它功能）：{exc}")
                self.markers = None

        # patrol_manager 在每次到达航点时通过 /robot/say 播报航点名；
        # GUI 用它显示"当前目标点"（不新增话题，完全复用现有链路）。
        self.node.create_subscription(String, "/robot/say", self._on_say, 10)

        # /map 元数据缓存：GUI 用它确认 map_server 真的加载了地图，
        # 而不是"launch 进程还活着"就报"导航已启动"。
        self._map_info = None       # (width, height, resolution, frame_id, stamp)
        try:
            from nav_msgs.msg import OccupancyGrid
            # map_server 的 /map 是 **latched（transient_local）**：它只在地图
            # 加载时发一次。若用默认（volatile）QoS 订阅，GUI 在导航起来之后才
            # 建订阅，就永远收不到那一条已发的消息，于是界面会一直误报
            # “地图未加载”。这里必须用匹配的 TRANSIENT_LOCAL QoS 才能拿到。
            map_qos = latched_qos()
            self.node.create_subscription(OccupancyGrid, "/map", self._on_map,
                                          map_qos)
        except Exception as exc:
            self.log(f"[gui] 无法订阅 /map（地图校验将不可用）：{exc}")
        # 部分部署会把巡检状态发到 /robot/status；有就显示，没有也不影响。
        self.node.create_subscription(String, "/robot/status", self._on_status, 10)

        self._trigger = Trigger
        self._clients = {
            name: self.node.create_client(Trigger, f"/patrol/{name}")
            for name in ("start", "pause", "resume", "stop")
        }

        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    # ---------- 后台 spin ----------
    def _spin(self):
        while self._running and self._rclpy.ok():
            try:
                self._rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception as exc:  # 单个回调异常不应终止 spin 线程
                self.log(f"[gui] ROS spin 异常：{exc}")
            self._refresh_pose()

    def _refresh_pose(self):
        try:
            transform = self._buffer.lookup_transform(
                self.map_frame, self.base_frame, self._rclpy.time.Time())
        except Exception as exc:
            # 明确区分"整条链都缺"和"只缺某一段"，界面直接告诉用户缺哪段。
            detail = str(exc)
            if "map" in detail and "odom" in detail:
                detail = (f"缺少 {self.map_frame}->{self.base_frame} TF"
                          f"（map->odom 或 odom->base_footprint 未发布）：{detail}")
            with self._lock:
                self._tf_error = detail
            return
        # 用共享实现换算 (x, y, yaw)，与 CLI record_waypoint 完全一致，
        # 避免 GUI 与其它入口对同一 TF 得出不同 yaw。
        x, y, yaw = map_pose.pose_from_transform(transform)
        with self._lock:
            self._pose = (x, y, yaw)
            self._pose_stamp = time.monotonic()
            self._tf_error = None

    def _on_say(self, message):
        with self._lock:
            self._goal = message.data

    def _on_map(self, message):
        info = message.info
        with self._lock:
            self._map_info = (int(info.width), int(info.height),
                              float(info.resolution), str(info.origin.position.x),
                              str(info.origin.position.y),
                              str(message.header.frame_id), time.monotonic())

    def map_info(self):
        """返回 /map 元数据 dict，或 None（还没收到）。"""
        with self._lock:
            info = self._map_info
        if info is None:
            return None
        return {"width": info[0], "height": info[1], "resolution": info[2],
                "origin_x": info[3], "origin_y": info[4], "frame_id": info[5],
                "age": time.monotonic() - info[6]}

    def _on_status(self, message):
        text = (message.data or "").strip().upper()
        with self._lock:
            if text in ("IDLE", "RUNNING", "PAUSED", "STOPPED"):
                self._status = "IDLE" if text == "STOPPED" else text

    # ---------- 供 GUI 线程读取 ----------
    def pose(self):
        """返回 (x, y, yaw) 或 None（TF 还没就绪）。"""
        with self._lock:
            return self._pose

    def pose_age(self):
        with self._lock:
            return None if self._pose_stamp == 0.0 else time.monotonic() - self._pose_stamp

    def tf_error(self):
        with self._lock:
            return self._tf_error

    def status(self):
        with self._lock:
            return self._status

    def goal(self):
        with self._lock:
            return self._goal

    def set_status(self, value):
        with self._lock:
            self._status = value

    # ---------- 服务调用（阻塞，带超时）----------
    def call_patrol(self, action, timeout=15.0):
        """调用 /patrol/<action>（start/pause/resume/stop）。

        返回 (ok, message)。找不到服务时返回 (False, '...服务不可用')，
        GUI 直接显示原因，不需要用户去终端排查。
        """
        client = self._clients.get(action)
        if client is None:
            return False, f"未知动作 {action}"
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f"/patrol/{action} 服务不可用（导航/巡检是否已启动？）"
        future = client.call_async(self._trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return False, f"/patrol/{action} 调用超时"
        result = future.result()
        if result is None:
            return False, f"/patrol/{action} 无响应"
        # 服务返回成功时同步状态，让界面立刻反映，不必等下一帧订阅。
        if result.success:
            mapping = {"start": "RUNNING", "resume": "RUNNING",
                       "pause": "PAUSED", "stop": "IDLE"}
            if action in mapping:
                self.set_status(mapping[action])
            if action == "stop":
                with self._lock:
                    self._goal = ""
        return bool(result.success), result.message

    def shutdown(self):
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=3.0)
        try:
            self.node.destroy_node()
        except Exception:
            pass
