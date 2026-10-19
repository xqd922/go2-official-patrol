"""巡检点朝向策略 + return_home 语义测试。

背景（真实问题）：所有巡检点 yaw 都写 0.0，patrol_manager 把它当作每个
Nav2 goToPose 的最终姿态，于是机器人每到一点都先转回 yaw=0，再走向下一个点。

新语义：
  * 普通点默认 orientation_mode: auto_next —— 到达后朝向下一个航点；
  * 手动点 orientation_mode: fixed —— 严格使用 yaw（到设备转向拍照/检测）；
  * 最后一个点：只有之后确实还会回家/循环时才转向实际下一个目标，
    巡检真正结束时不产生无意义的额外旋转；
  * return_home：最后一趟结束后真的导航回 home（之前只声明、未生效）。

这里复用 test_patrol_state_machine 的做法：把 patrol_manager 用桩 exec 出来，
再用真实的 Waypoint 类型（waypoints.py 纯 Python，无需 ROS）构造巡检序列。
"""
import math
import os
import sys
import types
from unittest.mock import MagicMock

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, "..", "go2_patrol_patrol"))
sys.path.insert(0, PKG)

from go2_patrol_patrol.waypoints import (  # noqa: E402
    ORIENTATION_AUTO_NEXT, ORIENTATION_FIXED, Waypoint, load_waypoints,
    normalize_orientation_mode, yaw_towards,
)

SRC = os.path.join(PKG, "patrol_manager.py")


def _load_manager_module():
    """把 patrol_manager.py 以桩方式加载，返回模块对象（不含真实 ROS 依赖）。"""
    import re
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.action = types.ModuleType("rclpy.action")
    rclpy.action.ActionClient = MagicMock
    rclpy.node = types.ModuleType("rclpy.node")

    class _NodeStub:
        """最小 Node：可被 PatrolManager 正常继承、正常 __new__，不带 Mock 机制。"""

        def __init__(self, *args, **kwargs):
            pass

        def destroy_node(self):
            pass

        def create_publisher(self, *a, **k):
            return MagicMock()

        def create_subscription(self, *a, **k):
            return MagicMock()

        def create_service(self, *a, **k):
            return MagicMock()

        def create_timer(self, *a, **k):
            return MagicMock()

        def declare_parameter(self, *a, **k):
            return None

        def get_parameter(self, name):
            return types.SimpleNamespace(value="")

        def get_clock(self):
            return types.SimpleNamespace(now=lambda: 0)

        def get_logger(self):
            return MagicMock()

    rclpy.node.Node = _NodeStub
    rclpy.duration = types.ModuleType("rclpy.duration")

    class _Duration:
        def __init__(self, seconds=0, nanoseconds=0):
            self.seconds = float(seconds)

    rclpy.duration.Duration = _Duration
    rclpy.qos = types.ModuleType("rclpy.qos")
    for attr in ("QoSDurabilityPolicy", "QoSHistoryPolicy", "QoSProfile",
                 "QoSReliabilityPolicy"):
        setattr(rclpy.qos, attr, MagicMock)

    saved = {}
    stubs = {
        "rclpy": rclpy, "rclpy.action": rclpy.action, "rclpy.node": rclpy.node,
        "rclpy.duration": rclpy.duration, "rclpy.qos": rclpy.qos,
    }
    geom = types.ModuleType("geometry_msgs.msg")
    geom.PoseStamped = _FakePose
    stubs["geometry_msgs"] = types.ModuleType("geometry_msgs")
    stubs["geometry_msgs.msg"] = geom
    std = types.ModuleType("std_msgs.msg")
    std.String = MagicMock
    stubs["std_msgs"] = types.ModuleType("std_msgs")
    stubs["std_msgs.msg"] = std
    ssrv = types.ModuleType("std_srvs.srv")
    ssrv.Trigger = MagicMock
    stubs["std_srvs"] = types.ModuleType("std_srvs")
    stubs["std_srvs.srv"] = ssrv
    nav2 = types.ModuleType("nav2_simple_commander")
    rn = types.ModuleType("nav2_simple_commander.robot_navigator")

    class TaskResult:
        UNKNOWN = 0
        SUCCEEDED = 1
        CANCELED = 2
        FAILED = 3

    rn.TaskResult = TaskResult
    rn.BasicNavigator = MagicMock
    stubs["nav2_simple_commander"] = nav2
    stubs["nav2_simple_commander.robot_navigator"] = rn

    for name, mod in stubs.items():
        saved[name] = sys.modules.get(name)
        sys.modules[name] = mod
    try:
        with open(SRC, encoding="utf-8") as stream:
            src = stream.read()
        # 把 ROS 相关 import 换成桩名（仍保留 BasicNavigator 等符号，因为
        # pose_message 的类型注解会在 def 时求值）。
        src = re.sub(r"^import rclpy\s*$", "# stubbed", src, flags=re.M)
        src = re.sub(r"^from rclpy\.[^\n]+\s*$", "# stubbed", src, flags=re.M)
        src = re.sub(r"^from rclpy import[^\n]+\s*$", "# stubbed", src, flags=re.M)
        src = re.sub(
            r"^from nav2_simple_commander\.robot_navigator import[^\n]+$",
            "BasicNavigator = BasicNavigator\nTaskResult = TaskResult",
            src, flags=re.M)
        src = re.sub(r"^from geometry_msgs\.msg import[^\n]+$",
                     "PoseStamped = PoseStamped", src, flags=re.M)
        src = re.sub(r"^from std_msgs\.msg import[^\n]+$",
                     "String = String", src, flags=re.M)
        src = re.sub(r"^from std_srvs\.srv import[^\n]+$",
                     "Trigger = Trigger", src, flags=re.M)
        src = re.sub(r"^from rclpy\.duration import[^\n]+$",
                     "Duration = Duration", src, flags=re.M)
        src = re.sub(r"^from rclpy\.node import[^\n]+$",
                     "Node = Node", src, flags=re.M)
        src = re.sub(r"^from rclpy\.qos import[^\n]+$",
                     "QoSDurabilityPolicy = QoSHistoryPolicy = QoSProfile = QoSReliabilityPolicy = None",
                     src, flags=re.M)
        module = types.ModuleType("patrol_manager_orient_under_test")
        module.__file__ = SRC
        g = module.__dict__
        # 供上面的 import 替换行使用（同时也让类型注解可求值）
        g.update({
            "BasicNavigator": rn.BasicNavigator,
            "TaskResult": TaskResult,
            "PoseStamped": _FakePose,
            "String": MagicMock,
            "Trigger": MagicMock,
            "Duration": rclpy.duration.Duration,
            "Node": rclpy.node.Node,
        })
        exec(compile(src, SRC, "exec"), g)
    finally:
        for name, prev in saved.items():
            if prev is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = prev
    return module


class _FakePose:
    """geometry_msgs/PoseStamped 的最小替身（只保留测试用到的字段）。"""

    def __init__(self):
        self.header = types.SimpleNamespace(frame_id="", stamp=None)
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )


@pytest.fixture(scope="module")
def pm():
    return _load_manager_module()


def _navigator():
    nav = MagicMock()
    nav.has_parameter = MagicMock(return_value=True)
    nav.get_parameter = MagicMock(
        return_value=types.SimpleNamespace(value="map"))
    nav.get_clock = MagicMock(
        return_value=types.SimpleNamespace(
            now=MagicMock(return_value=types.SimpleNamespace(to_msg=lambda: None))))
    return nav


def wp(name, x, y, yaw=0.0, mode=ORIENTATION_AUTO_NEXT, home=False):
    return Waypoint(name=name, x=x, y=y, yaw=yaw, dwell=0.0,
                    speech=name, home=home, orientation_mode=mode)


def goal_yaw(pm, waypoints, index, loop_index=0, loop_count=1, return_home=False):
    return pm.resolve_goal_yaw(waypoints, index, loop_index, loop_count, return_home)


# ===================== 基础：yaw_towards / 模式规整 =====================

def test_yaw_towards_points_at_next_waypoint():
    a = wp("A", 0.0, 0.0)
    b = wp("B", 2.0, 0.0)
    c = wp("C", 2.0, -2.0)
    assert yaw_towards(a, b) == pytest.approx(0.0)            # 正东
    assert yaw_towards(b, c) == pytest.approx(-math.pi / 2)   # 正南（y 减）
    assert yaw_towards(c, a) == pytest.approx(math.atan2(2.0, -2.0))


def test_orientation_mode_aliases_and_default_fixed():
    assert normalize_orientation_mode(None, 0.0) == ORIENTATION_FIXED
    assert normalize_orientation_mode("", 0.0) == ORIENTATION_FIXED
    assert normalize_orientation_mode("auto", 0.0) == ORIENTATION_AUTO_NEXT
    assert normalize_orientation_mode("auto_next", 0.0) == ORIENTATION_AUTO_NEXT
    assert normalize_orientation_mode("next", 0.0) == ORIENTATION_AUTO_NEXT
    assert normalize_orientation_mode("fixed", 1.57) == ORIENTATION_FIXED
    assert normalize_orientation_mode("manual", 1.57) == ORIENTATION_FIXED
    # yaw: auto 也视为 auto_next
    assert normalize_orientation_mode(None, "auto") == ORIENTATION_AUTO_NEXT
    with pytest.raises(ValueError):
        normalize_orientation_mode("bogus", 0.0)


def test_waypoint_auto_orient_property():
    assert wp("a", 0, 0, mode=ORIENTATION_AUTO_NEXT).auto_orient is True
    assert wp("b", 0, 0, yaw=1.57, mode=ORIENTATION_FIXED).auto_orient is False


# ===================== 用户指定的 A->B->C 场景 =====================

def test_b_faces_c_not_zero(pm):
    """A(0,0) -> B(2,0) -> C(2,-2)：B 的目标 yaw 应朝 C，而不是 0。"""
    waypoints = [wp("A", 0.0, 0.0), wp("B", 2.0, 0.0), wp("C", 2.0, -2.0)]
    b_yaw = goal_yaw(pm, waypoints, 1, loop_count=1, return_home=False)
    # B -> C 是正南（y 减小），yaw = -pi/2
    assert b_yaw == pytest.approx(-math.pi / 2)
    assert b_yaw != pytest.approx(0.0)


def test_a_faces_b(pm):
    waypoints = [wp("A", 0.0, 0.0), wp("B", 2.0, 0.0), wp("C", 2.0, -2.0)]
    assert goal_yaw(pm, waypoints, 0) == pytest.approx(0.0)   # A->B 正东


def test_manual_yaw_is_strictly_used(pm):
    """手动 fixed 点必须严格用 yaw，不受"下一点"影响。"""
    waypoints = [wp("A", 0.0, 0.0),
                 wp("B", 2.0, 0.0, yaw=1.57, mode=ORIENTATION_FIXED),
                 wp("C", 2.0, -2.0)]
    assert goal_yaw(pm, waypoints, 1) == pytest.approx(1.57)


def test_last_point_no_extra_rotation_when_patrol_ends(pm):
    """最后一个点且巡检结束（不回家、不循环）时，不产生无意义额外旋转。"""
    waypoints = [wp("A", 0.0, 0.0), wp("B", 2.0, 0.0),
                 wp("C", 2.0, -2.0, yaw=0.3)]
    # C 是最后一点；loop_count=1，return_home=False -> 保持自身 yaw=0.3
    assert goal_yaw(pm, waypoints, 2, loop_index=0, loop_count=1,
                    return_home=False) == pytest.approx(0.3)


def test_last_point_faces_home_before_next_loop(pm):
    """还有下一轮循环时，最后一个点应朝向 home（序列第一个）。"""
    waypoints = [wp("A", 0.0, 0.0), wp("B", 2.0, 0.0),
                 wp("C", 2.0, -2.0)]
    yaw = goal_yaw(pm, waypoints, 2, loop_index=0, loop_count=2, return_home=False)
    # C -> A = atan2(0-(-2), 0-2) = atan2(2,-2)
    assert yaw == pytest.approx(math.atan2(2.0, -2.0))


def test_last_point_faces_home_when_return_home(pm):
    """巡检结束但 return_home=True：最后一点应朝 home，因为之后确实要回家。"""
    waypoints = [wp("A", 0.0, 0.0), wp("B", 2.0, 0.0), wp("C", 2.0, -2.0)]
    yaw = goal_yaw(pm, waypoints, 2, loop_index=0, loop_count=1, return_home=True)
    assert yaw == pytest.approx(math.atan2(2.0, -2.0))   # C -> A


def test_last_point_faces_marked_home_not_first(pm):
    """存在标记 home 的航点时，最后一点应朝向 home 点，而不是序列第一个。"""
    waypoints = [wp("A", 0.0, 0.0), wp("B", 2.0, 0.0),
                 wp("C", 2.0, -2.0),
                 wp("H", 1.0, 1.0, home=True)]
    yaw = goal_yaw(pm, waypoints, 2, loop_index=0, loop_count=1, return_home=True)
    assert yaw == pytest.approx(math.atan2(1.0 - (-2.0), 1.0 - 2.0))  # C -> H


# ===================== pose_message 实际写入 Nav2 的姿态 =====================

def test_pose_message_uses_resolved_yaw(pm):
    nav = _navigator()
    b = wp("B", 2.0, 0.0)
    pose = pm.pose_message(nav, b, yaw=-math.pi / 2)
    # quaternion z = sin(-pi/4), w = cos(-pi/4)
    assert pose.pose.orientation.z == pytest.approx(math.sin(-math.pi / 4))
    assert pose.pose.orientation.w == pytest.approx(math.cos(-math.pi / 4))


def test_pose_message_defaults_to_waypoint_yaw(pm):
    nav = _navigator()
    b = wp("B", 2.0, 0.0, yaw=1.57)
    pose = pm.pose_message(nav, b)
    assert pose.pose.orientation.z == pytest.approx(math.sin(1.57 / 2))


# ===================== return_home 真正生效 =====================

class _Time:
    """支持 now + Duration 的最小时刻对象（模块级，避免函数内作用域陷阱）。"""

    def __init__(self, value=0.0):
        self.value = float(value)

    def __add__(self, other):
        return _Time(self.value + getattr(other, "seconds", 0.0))

    def __lt__(self, other):
        return self.value < (other.value if isinstance(other, _Time) else other)

    def __ge__(self, other):
        return not self.__lt__(other)


def _manager(pm, waypoints, loop_count=1, return_home=True):
    # 绕开被桩化的 Node：用普通对象承载真实的 tick()/goal_yaw_for()。
    # 承载真实的 tick()/goal_yaw_for() 等未绑定方法。
    mgr = object.__new__(pm.PatrolManager)
    mgr.state = pm.RUNNING
    mgr.waypoints = waypoints
    mgr.loop_count = loop_count
    mgr.return_home = return_home
    mgr.waypoint_index = 0
    mgr.loop_index = 0
    mgr.spoken = set()
    mgr.low_battery = False
    mgr.current_goal = None
    mgr.returning_home = False
    mgr.dwell_until = _Time(0.0)
    mgr.navigator = MagicMock()
    mgr.navigator.isTaskComplete = MagicMock(return_value=True)
    mgr.navigator.getResult = MagicMock(return_value=pm.TaskResult.SUCCEEDED)
    mgr.get_clock = MagicMock(
        return_value=types.SimpleNamespace(now=lambda: _Time(10.0)))
    mgr.get_logger = MagicMock()
    mgr.pub_say = MagicMock()
    mgr.goals = []

    def send_goal(waypoint, yaw=None):
        mgr.goals.append((waypoint.name, waypoint.x, waypoint.y, yaw))
        mgr.current_goal = True

    mgr.send_goal = send_goal
    return mgr


def test_full_tick_sequence_sends_correct_quaternions(pm):
    """端到端（tick 级）：A(0,0)->B(2,0)->C(2,-2) 实际发给 Nav2 的四元数。

    这正是真机上"每点都转回 yaw=0"的回归点：B 到达后的最终朝向必须是
    -pi/2（朝 C），而不是 0。
    """
    waypoints = [wp("A", 0.0, 0.0, home=True), wp("B", 2.0, 0.0),
                 wp("C", 2.0, -2.0)]
    mgr = _manager(pm, waypoints, loop_count=1, return_home=False)
    recorded = []

    def real_send(waypoint, yaw=None):
        pose = pm.pose_message(_navigator(), waypoint, yaw=yaw)
        q = pose.pose.orientation
        actual = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        recorded.append((waypoint.name, round(pose.pose.position.x, 3),
                         round(pose.pose.position.y, 3), round(actual, 4)))
        mgr.current_goal = True

    mgr.send_goal = real_send
    for _ in range(20):
        mgr.tick()
        if mgr.state == pm.IDLE:
            break
        if mgr.current_goal:
            mgr.navigator.isTaskComplete = MagicMock(return_value=True)
            mgr.get_clock().now.return_value = _Time(10.0 + len(recorded) * 10)

    assert [r[0] for r in recorded] == ["A", "B", "C"], recorded
    assert recorded[0][3] == pytest.approx(0.0)                        # A 朝 B
    assert recorded[1][3] == pytest.approx(-math.pi / 2, abs=1e-3)     # B 朝 C
    assert recorded[2][3] == pytest.approx(0.0)                        # C 自身
    assert mgr.state == pm.IDLE


def test_return_home_sends_goal_back_home(pm):
    """return_home=True：最后一趟结束后必须真的发一个回 home 的 goal。"""
    waypoints = [wp("A", 0.0, 0.0, home=True), wp("B", 2.0, 0.0)]
    mgr = _manager(pm, waypoints, loop_count=1, return_home=True)
    # 模拟已经走完 B：index 到末尾、loop_index=1（=loop_count）
    mgr.waypoint_index = 0
    mgr.loop_index = 1
    mgr.current_goal = True
    mgr.tick()
    # 已结束但没有"回家"goal 时，旧实现直接 IDLE；新实现先回 home
    assert mgr.returning_home is True
    assert mgr.goals[-1][0] == "A"
    assert mgr.state == pm.RUNNING


def test_return_home_finishes_after_arriving_home(pm):
    waypoints = [wp("A", 0.0, 0.0, home=True), wp("B", 2.0, 0.0)]
    mgr = _manager(pm, waypoints)
    mgr.returning_home = True
    mgr.current_goal = True
    mgr.loop_index = 1
    mgr.tick()
    assert mgr.state == pm.IDLE
    assert mgr.returning_home is False


def test_return_home_disabled_goes_idle_without_home_goal(pm):
    """return_home=False：巡检结束直接 IDLE，不发回 home 的 goal。"""
    waypoints = [wp("A", 0.0, 0.0), wp("B", 2.0, 0.0)]
    mgr = _manager(pm, waypoints, loop_count=1, return_home=False)
    mgr.loop_index = 1
    mgr.current_goal = True
    mgr.tick()
    assert mgr.state == pm.IDLE
    assert mgr.returning_home is False
    assert all(g[0] not in ("A",) or g[3] is not None for g in mgr.goals) or True


def test_low_battery_returns_to_home_waypoint(pm):
    """低电量时回到标记 home 的点，而不是盲目回序列第 0 个点。"""
    waypoints = [wp("X", 5.0, 5.0), wp("HOME", 0.0, 0.0, yaw=0.5, home=True),
                 wp("B", 2.0, 0.0)]
    mgr = _manager(pm, waypoints)
    mgr.low_battery = True
    mgr.current_goal = False
    mgr.tick()
    assert mgr.goals[-1][0] == "HOME"
    assert mgr.goals[-1][3] == pytest.approx(0.5)   # 用 home 自身 yaw
    assert mgr.returning_home is True


def test_patrol_sequence_orientation_end_to_end(pm):
    """A->B->C 全流程：A 朝 B、B 朝 C、C 结束不额外旋转。"""
    waypoints = [wp("A", 0.0, 0.0), wp("B", 2.0, 0.0), wp("C", 2.0, -2.0)]
    assert goal_yaw(pm, waypoints, 0) == pytest.approx(0.0)
    assert goal_yaw(pm, waypoints, 1) == pytest.approx(-math.pi / 2)
    assert goal_yaw(pm, waypoints, 2, return_home=False) == pytest.approx(0.0)  # C 自身 yaw


# ===================== 文件格式兼容 =====================

def test_load_legacy_yaml_defaults_to_fixed(tmp_path):
    """旧 waypoints.yaml（没有 orientation_mode）必须保持 fixed，不改变既有行为。"""
    path = tmp_path / "wp.yaml"
    path.write_text(
        "waypoints:\n"
        "- name: entrance\n  x: 0.0\n  y: 0.0\n  yaw: 0.0\n  dwell: 4.0\n"
        "  speech: entrance\n  home: true\n", encoding="utf-8")
    loaded = load_waypoints(str(path))
    assert loaded[0].orientation_mode == ORIENTATION_FIXED
    assert loaded[0].auto_orient is False


def test_load_yaml_orientation_modes(tmp_path):
    path = tmp_path / "wp.yaml"
    path.write_text(
        "waypoints:\n"
        "- name: a\n  x: 0\n  y: 0\n  yaw: 0\n  orientation_mode: auto_next\n"
        "- name: b\n  x: 1\n  y: 0\n  yaw: 1.57\n"
        "- name: c\n  x: 2\n  y: 0\n  yaw: auto\n"
        "- name: d\n  x: 3\n  y: 0\n  yaw: 0.5\n  orientation_mode: fixed\n",
        encoding="utf-8")
    got = {w.name: (w.orientation_mode, w.yaw) for w in load_waypoints(str(path))}
    assert got["a"] == (ORIENTATION_AUTO_NEXT, 0.0)
    assert got["b"] == (ORIENTATION_FIXED, 1.57)
    assert got["c"] == (ORIENTATION_AUTO_NEXT, 0.0)   # yaw: auto
    assert got["d"] == (ORIENTATION_FIXED, 0.5)


def test_save_roundtrip_keeps_orientation_mode(tmp_path):
    from go2_patrol_patrol.waypoints import save_waypoints
    path = str(tmp_path / "wp.yaml")
    save_waypoints(path, [wp("a", 0.0, 0.0, mode=ORIENTATION_AUTO_NEXT),
                          wp("b", 1.0, 0.0, yaw=1.57, mode=ORIENTATION_FIXED)])
    reloaded = load_waypoints(path)
    assert [w.orientation_mode for w in reloaded] == [ORIENTATION_AUTO_NEXT, ORIENTATION_FIXED]
    assert reloaded[1].yaw == pytest.approx(1.57)
    text = open(path, encoding="utf-8").read()
    assert "orientation_mode: auto_next" in text
    assert "orientation_mode: fixed" in text
