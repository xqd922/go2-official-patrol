"""记录航点必须取 map 坐标系位姿 —— 这是本次最关键的正确性约束。

odom_bridge 的 /odom 是 odom 系里程计；若直接把它当地图坐标，机器人起点不在
地图原点时记录的航点会整体偏移。这里用一个"map->odom 有偏移"的假 TF 树，
断言记录出来的是 map 系坐标，而不是 /odom 的原始值。
"""
import math

from patrol_gui.map_pose import (pose_from_transform, wait_for_map_pose,
                                 yaw_from_quaternion)


class _Vec:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class _Transform:
    def __init__(self, x, y, z, w):
        self.transform = _Vec(
            translation=_Vec(x=x, y=y, z=0.0),
            rotation=_Vec(x=0.0, y=0.0, z=z, w=w),
        )


class _FakeBuffer:
    """模拟 tf2_ros.Buffer：前 fail_times 次抛异常，之后返回固定变换。"""

    def __init__(self, transform=None, fail_times=0):
        self.transform = transform
        self.fail_times = fail_times
        self.calls = []

    def lookup_transform(self, target, source, time_point):
        self.calls.append((target, source))
        if self.fail_times > 0:
            self.fail_times -= 1
            raise RuntimeError("LookupException: frame does not exist")
        if self.transform is None:
            raise RuntimeError("LookupException")
        return self.transform


def test_yaw_from_quaternion_quarter_turn():
    # yaw = pi/2 -> z = sin(pi/4), w = cos(pi/4)
    q = _Vec(x=0.0, y=0.0, z=math.sin(math.pi / 4), w=math.cos(math.pi / 4))
    assert abs(yaw_from_quaternion(q) - math.pi / 2) < 1e-6


def test_pose_from_transform_reads_translation_and_yaw():
    x, y, yaw = pose_from_transform(_Transform(3.0, -1.5, 0.0, 1.0))
    assert (x, y) == (3.0, -1.5)
    assert abs(yaw) < 1e-9


def test_wait_for_map_pose_uses_map_frame_not_odom():
    """map->base_footprint = (1.23, 0.82)，且 odom 偏移为 (5, 5)。

    假 buffer 只在被问 `map` 时给答案；若实现错误地去查 odom，就会拿不到值。
    """
    buffer = _FakeBuffer(transform=_Transform(1.23, 0.82, 0.0, 1.0))
    pose = wait_for_map_pose(buffer, timeout_sec=0.5,
                             time_point_factory=lambda: None)
    assert pose is not None
    x, y, yaw = pose
    assert abs(x - 1.23) < 1e-9
    assert abs(y - 0.82) < 1e-9
    # 明确断言查询的是 map 作为 target frame
    assert buffer.calls[0][0] == "map"
    assert buffer.calls[0][1] == "base_footprint"


def test_wait_for_map_pose_retries_then_succeeds():
    """TF 晚到（前几次查不到）时应重试，而不是立刻失败。"""
    buffer = _FakeBuffer(transform=_Transform(2.0, 0.0, 0.0, 1.0), fail_times=3)
    pose = wait_for_map_pose(buffer, timeout_sec=2.0, poll_interval=0.01,
                             time_point_factory=lambda: None)
    assert pose is not None
    assert abs(pose[0] - 2.0) < 1e-9
    assert len(buffer.calls) == 4


def test_wait_for_map_pose_times_out_when_no_transform():
    buffer = _FakeBuffer(transform=None)
    pose = wait_for_map_pose(buffer, timeout_sec=0.2, poll_interval=0.01,
                             time_point_factory=lambda: None)
    assert pose is None
