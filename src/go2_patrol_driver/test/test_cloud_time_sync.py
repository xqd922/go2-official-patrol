"""点云时间戳对齐的纯函数（不依赖 rclpy，直接可测）。"""
import math

import pytest

from go2_patrol_driver.cloud_time_sync import (
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
