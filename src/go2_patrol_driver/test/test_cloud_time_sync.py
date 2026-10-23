"""点云时间戳对齐与空点过滤的纯函数（不依赖 rclpy，直接可测）。"""
import math
import struct

import pytest

from go2_patrol_driver.cloud_time_sync import (
    compact_points,
    keep_mask,
    next_offset,
    seconds_to_stamp,
    stamp_to_seconds,
)


class FakeStamp:
    def __init__(self, sec=0, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


def test_stamp_roundtrip_preserves_microseconds():
    stamp = FakeStamp()
    seconds_to_stamp(stamp, 1792552316.354)
    assert stamp_to_seconds(stamp) == pytest.approx(1792552316.354, abs=1e-6)


def test_stamp_never_goes_negative():
    stamp = FakeStamp()
    seconds_to_stamp(stamp, -1.0)
    assert (stamp.sec, stamp.nanosec) == (0, 0)


def test_first_frame_is_aligned_immediately():
    """起机第一帧就该对上：295s 的偏差不能靠几十帧慢慢追。"""
    offset, hard = next_offset(None, 295.0, 0.2, 5.0)
    assert hard is True
    assert offset == pytest.approx(295.0)


def test_small_jitter_is_smoothed_not_jumped():
    """单帧抖动只吃掉 20%，避免把噪声写进 TF。"""
    offset, hard = next_offset(295.0, 295.01, 0.2, 5.0)
    assert hard is False
    assert offset == pytest.approx(295.002)


def test_large_jump_resyncs_immediately():
    """狗重启后时钟跳变，必须立刻对齐。"""
    offset, hard = next_offset(295.0, 0.5, 0.2, 5.0)
    assert hard is True
    assert offset == pytest.approx(0.5)


def test_offset_converges_toward_measurement():
    offset = 0.0
    for _ in range(30):
        offset, _ = next_offset(offset, 2.0, 0.2, 5.0)
    assert math.isclose(offset, 2.0, abs_tol=0.01)


class FakeField:
    def __init__(self, name, offset, datatype=7):
        self.name = name
        self.offset = offset
        self.datatype = datatype
        self.count = 1


class FakeCloud:
    """按 Go2 的真实布局造一帧：x/y/z/intensity，point_step=32。"""

    def __init__(self, points, point_step=32):
        self.point_step = point_step
        self.fields = [
            FakeField("x", 0),
            FakeField("y", 4),
            FakeField("z", 8),
            FakeField("intensity", 16),
        ]
        chunks = []
        for x, y, z, intensity in points:
            point = struct.pack("<ffff", x, y, z, intensity)
            chunks.append(point + b"\x00" * (point_step - len(point)))
        self.data = b"".join(chunks)
        self.width = len(points)
        self.height = 1
        self.row_step = self.width * point_step


def test_zero_filler_is_dropped_and_real_returns_kept():
    """Go2 每帧前 10000 点是 (0,0,0) 填充，必须全部删掉。"""
    filler = [(0.0, 0.0, 0.0, 0.0)] * 10000
    real = [(1.5, 0.2, -0.25, 80.0), (0.0, 2.0, -0.25, 60.0)]
    cloud = FakeCloud(filler + real)

    data, kept = compact_points(cloud)

    assert kept == 2
    assert len(data) == 2 * cloud.point_step
    # 保留下来的确实是那两条真实回波，且逐字节未改
    assert data[0:16] == struct.pack("<ffff", 1.5, 0.2, -0.25, 80.0)
    assert data[32:48] == struct.pack("<ffff", 0.0, 2.0, -0.25, 60.0)


def test_point_lying_on_one_axis_is_not_treated_as_filler():
    """正前方/正侧方的真实回波有一个轴为 0，不能误删。"""
    cloud = FakeCloud([(0.0, 0.0, 0.0, 0.0), (2.0, 0.0, 0.0, 10.0), (0.0, -1.2, 0.0, 10.0)])

    _, kept = compact_points(cloud)

    assert kept == 2


def test_near_zero_epsilon_still_drops_driver_dust():
    """驱动偶尔写出 1e-8 这种尘埃值，同样算空点。"""
    cloud = FakeCloud([(1e-8, -1e-9, 0.0, 0.0), (0.9, 0.1, -0.2, 30.0)])

    _, kept = compact_points(cloud)

    assert kept == 1


def test_mask_length_matches_point_count_not_byte_count():
    """掩码按点数给，不能按字节数给——否则会切出半截点。"""
    cloud = FakeCloud([(1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), (2.0, 0.0, 0.0, 0.0)])

    keep = keep_mask(cloud)

    assert keep is not None
    assert keep.tolist() == [True, False, True]
    assert len(keep) == 3


def test_missing_xyz_fields_leave_cloud_untouched():
    """字段不全时返回 None，调用方原样转发，不能把整帧清零。"""
    cloud = FakeCloud([(1.0, 2.0, 3.0, 0.0)])
    cloud.fields = [FakeField("intensity", 16)]

    assert compact_points(cloud) is None
