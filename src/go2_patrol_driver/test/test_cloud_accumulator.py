"""点云累积的纯函数（不依赖 ROS 运行时，直接可测）。"""
import math

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from go2_patrol_driver.cloud_accumulator import build_cloud, read_xyz, transform_points


class FakeField:
    def __init__(self, name, offset, datatype=7):
        self.name = name
        self.offset = offset
        self.datatype = datatype
        self.count = 1


class FakeCloud:
    def __init__(self, points, point_step=32):
        self.point_step = point_step
        self.fields = [
            FakeField("x", 0),
            FakeField("y", 4),
            FakeField("z", 8),
            FakeField("intensity", 16),
        ]
        raw = np.zeros((len(points), point_step), dtype=np.uint8)
        for row, (x, y, z) in zip(raw, points):
            row[0:4].view(np.float32)[0] = x
            row[4:8].view(np.float32)[0] = y
            row[8:12].view(np.float32)[0] = z
        self.data = raw.tobytes()
        self.width = len(points)
        self.height = 1
        self.row_step = self.width * point_step


class FakeVector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)


class FakeQuaternion:
    def __init__(self, yaw=0.0):
        self.x = 0.0
        self.y = 0.0
        self.z = math.sin(yaw / 2.0)
        self.w = math.cos(yaw / 2.0)


class FakeTransform:
    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self.translation = FakeVector(x, y, z)
        self.rotation = FakeQuaternion(yaw)


def test_read_xyz_respects_point_step_padding():
    """point_step=32 但字段只到 offset 16，必须按步长跳过 padding。"""
    cloud = FakeCloud([(1.0, 2.0, 3.0), (-4.0, 5.5, -6.25)])

    points = read_xyz(cloud)

    assert points is not None
    assert points.shape == (2, 3)
    assert points[0].tolist() == pytest.approx([1.0, 2.0, 3.0])
    assert points[1].tolist() == pytest.approx([-4.0, 5.5, -6.25])


def test_read_xyz_returns_none_when_fields_missing():
    cloud = FakeCloud([(1.0, 2.0, 3.0)])
    cloud.fields = [FakeField("intensity", 16)]

    assert read_xyz(cloud) is None


def test_transform_points_applies_yaw_and_translation():
    """90 度 yaw：前方 1 m 的点应该转到左侧 1 m，再加平移。"""
    points = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    transform = FakeTransform(x=0.5, y=-0.25, yaw=math.pi / 2)

    out = transform_points(points, transform)

    assert out[0].tolist() == pytest.approx([0.5, 0.75, 0.0], abs=1e-6)


def test_transform_points_ignores_roll_pitch_but_keeps_z_translation():
    """平地场景只用 yaw；z 平移要保留，换带高度的点云时不至于静默丢信息。"""
    points = np.array([[0.0, 1.0, 0.2]], dtype=np.float32)
    transform = FakeTransform(z=0.13)

    out = transform_points(points, transform)

    assert out[0].tolist() == pytest.approx([0.0, 1.0, 0.33], abs=1e-6)


def test_build_cloud_roundtrips_through_read_xyz():
    """发布出去的 PointCloud2 必须能被下游按同一套规则读回来。"""
    points = np.array([[1.5, -0.5, 0.25], [0.0, 2.0, -0.3]], dtype=np.float32)

    cloud = build_cloud(points, stamp=Time(), frame_id="base_footprint")
    decoded = read_xyz(cloud)

    assert cloud.width == 2
    assert cloud.height == 1
    assert cloud.point_step == 16
    assert cloud.is_dense is True
    assert cloud.header.frame_id == "base_footprint"
    assert decoded is not None
    assert np.allclose(decoded, points, atol=1e-6)


def test_build_cloud_handles_empty_input():
    """窗口里一帧都没有时不能崩，宽度应为 0。"""
    cloud = build_cloud(np.zeros((0, 3), dtype=np.float32), stamp=Time(), frame_id="base_footprint")

    assert cloud.width == 0
    assert len(cloud.data) == 0
