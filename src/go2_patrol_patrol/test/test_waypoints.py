import tempfile
from pathlib import Path

from go2_patrol_patrol.waypoints import load_waypoints


def test_load_waypoints_roundtrip():
    content = """
waypoints:
  - name: entrance
    x: 1.0
    y: 2.0
    yaw: 0.5
    dwell: 3.0
    speech: entrance
    home: true
"""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as stream:
        stream.write(content)
        path = stream.name
    waypoints = load_waypoints(path)
    Path(path).unlink()
    assert len(waypoints) == 1
    assert waypoints[0].name == "entrance"
    assert waypoints[0].x == 1.0
    assert waypoints[0].speech == "entrance"
    assert waypoints[0].home is True


def test_load_waypoints_reads_per_point_yaw_mode():
    content = """
waypoints:
  - name: corner
    x: 1.0
    y: 2.0
    yaw: 0.5
  - name: equipment
    x: 3.0
    y: 4.0
    yaw: 0.0
    yaw_mode: fixed
"""
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as stream:
        stream.write(content)
        path = stream.name
    waypoints = load_waypoints(path)
    Path(path).unlink()
    # 没写 yaw_mode 的点留空，节点用站点级 yaw_mode 兜底
    assert waypoints[0].yaw_mode == ""
    assert waypoints[1].yaw_mode == "fixed"
