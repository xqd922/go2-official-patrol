"""fake_go2 运动学单测（离线，不启动 ROS）。

只验证纯粹的二维独轮车积分逻辑：给定 /cmd_vel 与 dt，
x/y/yaw 是否按预期更新。这里不 import rclpy——
通过直接调用未绑定方法的方式复用 FakeGo2 的算法，
避免测试依赖 ROS 2 运行时。
"""
import math

import fake_go2


class _Time:
    """rclpy.time.Time 的最小替身：支持相减并暴露 .nanoseconds。"""

    def __init__(self, seconds):
        self._s = seconds

    def __sub__(self, other):
        return _Duration(self._s - other._s)


class _Duration:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)


class _Recorder:
    """最小替身：只提供 integrate() 需要的字段与 get_clock()。"""

    def __init__(self, x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.vx, self.vy, self.wz = vx, vy, wz
        # 与 FakeGo2.__init__ 一致：首次 integrate 只建立时间基准。
        self.last_cmd_time = None
        self._t = 0.0

    def get_clock(self):
        return self

    def now(self):
        # 复刻 rclpy.time.Time：需要支持减法，且结果带 .nanoseconds。
        return _Time(self._t)

    def _set_t(self, t):
        self._t = t


def _step(state, vx, vy, wz, dt):
    """推进状态 dt 秒，复用 FakeGo2.integrate 的真实实现。"""
    state.vx, state.vy, state.wz = vx, vy, wz
    fake_go2.FakeGo2.integrate(state)


def _new(t=0.0):
    s = _Recorder()
    s._t = t
    # 第一次调用只记录时间基准（last_cmd_time 从 None 变为当前时刻）
    fake_go2.FakeGo2.integrate(s)
    return s


def test_initial_state_is_at_origin():
    s = _new()
    assert (s.x, s.y, s.yaw) == (0.0, 0.0, 0.0)


def test_straight_line_motion_along_x():
    """/cmd_vel 只给前进速度：x 增加，y 与 yaw 不变。"""
    s = _new()
    for _ in range(10):
        s._t += 0.1
        _step(s, 0.5, 0.0, 0.0, 0.1)
    assert abs(s.x - 0.5) < 1e-6
    assert abs(s.y) < 1e-6
    assert abs(s.yaw) < 1e-6


def test_pure_rotation_changes_yaw_only():
    """只给角速度：yaw 增加，位置不变。"""
    s = _new()
    for _ in range(10):
        s._t += 0.1
        _step(s, 0.0, 0.0, 0.5, 0.1)
    assert abs(s.yaw - 0.5) < 1e-6
    assert abs(s.x) < 1e-6
    assert abs(s.y) < 1e-6


def test_motion_follows_current_heading():
    """先转 90°，再前进，应沿 +y 方向移动。"""
    s = _new()
    s._t += 0.1
    _step(s, 0.0, 0.0, math.pi / 0.2, 0.1)   # yaw -> pi/2
    assert abs(s.yaw - math.pi / 2) < 1e-6
    s._t += 0.1
    _step(s, 0.5, 0.0, 0.0, 0.1)
    assert abs(s.x) < 1e-6
    assert abs(s.y - 0.05) < 1e-6


def test_reaches_two_meters_toward_equipment_zone():
    """离线巡检场景：从 (0,0) 朝 +x 前进 2m。"""
    s = _new()
    for _ in range(40):
        s._t += 0.1
        _step(s, 0.5, 0.0, 0.0, 0.1)
    assert abs(s.x - 2.0) < 1e-6
    assert abs(s.y) < 1e-6


def test_abnormal_dt_is_ignored():
    """dt<=0 或 dt>0.5 时不应移动（防止定时器抖动导致瞬移）。"""
    s = _new()
    s._t += 0.0                 # dt = 0
    _step(s, 0.5, 0.0, 0.0, 0.0)
    assert (s.x, s.y, s.yaw) == (0.0, 0.0, 0.0)
    s._t += 5.0                 # dt = 5 > 0.5
    _step(s, 0.5, 0.0, 0.0, 5.0)
    assert (s.x, s.y, s.yaw) == (0.0, 0.0, 0.0)


def test_lateral_command_accumulates():
    """linear.y 为非零时也应按模型累积（为将来全向移动保留）。"""
    s = _new()
    s._t += 0.1
    _step(s, 0.0, 0.5, 0.0, 0.1)
    assert abs(s.x) < 1e-6
    assert abs(s.y - 0.05) < 1e-6
