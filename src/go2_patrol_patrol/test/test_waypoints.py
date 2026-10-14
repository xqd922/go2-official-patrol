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
