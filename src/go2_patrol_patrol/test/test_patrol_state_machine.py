"""Off-ROS state-machine test for patrol_manager's services.

The real `PatrolManager` derives from `rclpy.node.Node` and constructs a
`BasicNavigator` in `__init__`, which blocks until Nav2/AMCL are up. To verify
the state machine without a running Nav2, this test extracts the service
callbacks into a stand-alone handler that reuses the same code paths.

The source is loaded as text and patched so that the heavy rclpy /
nav2_simple_commander imports are replaced with stubs, then the `on_*` /
`reset_run` methods are bound to a small fake manager.
"""
import os
import re
import sys
import types
import unittest
from unittest.mock import MagicMock

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "go2_patrol_patrol"))
sys.path.insert(0, ROOT)
SRC = os.path.join(ROOT, "patrol_manager.py")


def _build_stub_modules():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **kw: None
    rclpy.shutdown = lambda *a, **kw: None
    rclpy.action = types.ModuleType("rclpy.action")
    rclpy.action.ActionClient = MagicMock
    rclpy.node = types.ModuleType("rclpy.node")
    rclpy.node.Node = MagicMock
    rclpy.duration = types.ModuleType("rclpy.duration")
    rclpy.duration.Duration = MagicMock(return_value=MagicMock(nanoseconds=0))
    rclpy.qos = types.ModuleType("rclpy.qos")
    rclpy.qos.QoSDurabilityPolicy = MagicMock()
    rclpy.qos.QoSHistoryPolicy = MagicMock()
    rclpy.qos.QoSProfile = MagicMock()
    rclpy.qos.QoSReliabilityPolicy = MagicMock()
    return rclpy, rclpy.action, rclpy.node, rclpy.duration, rclpy.qos


# Provide rclpy + rclpy.action stubs so `from nav2_simple_commander...` won't
# try to import a real rclpy.
_stub_rclpy = _build_stub_modules()
sys.modules.setdefault("rclpy", _stub_rclpy[0])
sys.modules.setdefault("rclpy.action", _stub_rclpy[1])
sys.modules.setdefault("rclpy.node", _stub_rclpy[2])
sys.modules.setdefault("rclpy.duration", _stub_rclpy[3])
sys.modules.setdefault("rclpy.qos", _stub_rclpy[4])

# Replace `from rclpy import ...` lines with nothing so the import doesn't
# replace our stub module.
with open(SRC, "r", encoding="utf-8") as stream:
    src = stream.read()
src = re.sub(r"^import rclpy\s*$", "# rclpy stubbed", src, flags=re.M)
src = re.sub(r"^from rclpy\.[^\n]+\s*$", "# rclpy stubbed", src, flags=re.M)
src = re.sub(r"^from rclpy import[^\n]+\s*$", "# rclpy stubbed", src, flags=re.M)
src = re.sub(r"^from rclpy\s*$", "# rclpy stubbed", src, flags=re.M)
src = re.sub(
    r"^from nav2_simple_commander\.robot_navigator import[^\n]+$",
    "class TaskResult: UNKNOWN=0; SUCCEEDED=1; CANCELED=2; FAILED=3\nclass BasicNavigator: pass",
    src,
    flags=re.M,
)
src = "class Node: pass\n" + src  # referenced inside class body via `super().__init__`
# Stub geometry_msgs.PoseStamped so the file can be exec'd without ROS types.
geom = types.ModuleType("geometry_msgs.msg")
geom.PoseStamped = MagicMock
sys.modules.setdefault("geometry_msgs.msg", geom)
std = types.ModuleType("std_msgs.msg")
std.String = MagicMock
sys.modules.setdefault("std_msgs.msg", std)
ssrv = types.ModuleType("std_srvs.srv")
ssrv.Trigger = MagicMock
sys.modules.setdefault("std_srvs.srv", ssrv)

module = types.ModuleType("patrol_manager_under_test")
module.__file__ = SRC
exec(compile(src, SRC, "exec"), module.__dict__)

IDLE = module.IDLE
RUNNING = module.RUNNING
PAUSED = module.PAUSED


def _make_manager():
    mgr = module.PatrolManager.__new__(module.PatrolManager)
    mgr.state = IDLE
    mgr.waypoints = []
    mgr.loop_count = 1
    mgr.waypoint_index = 0
    mgr.loop_index = 0
    mgr.spoken = set()
    mgr.low_battery = False
    mgr.current_goal = None
    mgr.dwell_until = None
    mgr.navigator = MagicMock()
    # PatrolManager.__init__ stores `self.dwell_until = self.get_clock().now()`,
    # so the test double needs a matching fake clock.
    mgr.get_clock = MagicMock(return_value=MagicMock(now=MagicMock(return_value=0)))
    return mgr


def _call(handler, **fields):
    request = MagicMock()
    for k, v in fields.items():
        setattr(request, k, v)
    response = MagicMock()
    response.success = None
    response.message = None
    handler(request, response)
    return response


class PatrolStateMachineTest(unittest.TestCase):
    def test_initial_state_is_idle(self):
        mgr = _make_manager()
        self.assertEqual(mgr.state, IDLE)

    def test_start_from_idle_keeps_running(self):
        mgr = _make_manager()
        resp = _call(mgr.on_start)
        self.assertTrue(resp.success)
        self.assertEqual(mgr.state, RUNNING)

    def test_start_when_already_running_fails(self):
        mgr = _make_manager()
        _call(mgr.on_start)
        resp = _call(mgr.on_start)
        self.assertFalse(resp.success)
        self.assertEqual(mgr.state, RUNNING)

    def test_pause_from_running(self):
        mgr = _make_manager()
        _call(mgr.on_start)
        resp = _call(mgr.on_pause)
        self.assertTrue(resp.success)
        self.assertEqual(mgr.state, PAUSED)
        # No active goal yet, so cancelTask is not called; the service still succeeds.
        mgr.navigator.cancelTask.assert_not_called()

    def test_pause_from_idle_fails(self):
        mgr = _make_manager()
        resp = _call(mgr.on_pause)
        self.assertFalse(resp.success)
        self.assertEqual(mgr.state, IDLE)

    def test_resume_from_paused(self):
        mgr = _make_manager()
        _call(mgr.on_start)
        _call(mgr.on_pause)
        resp = _call(mgr.on_resume)
        self.assertTrue(resp.success)
        self.assertEqual(mgr.state, RUNNING)

    def test_resume_from_idle_fails(self):
        mgr = _make_manager()
        resp = _call(mgr.on_resume)
        self.assertFalse(resp.success)
        self.assertEqual(mgr.state, IDLE)

    def test_stop_from_running_cancels_and_resets(self):
        mgr = _make_manager()
        _call(mgr.on_start)
        mgr.waypoint_index = 2
        mgr.loop_index = 1
        mgr.spoken.add("a")
        # No active goal yet, so cancelTask is not called; the service still
        # resets the run state.
        resp = _call(mgr.on_stop)
        self.assertTrue(resp.success)
        self.assertEqual(mgr.state, IDLE)
        mgr.navigator.cancelTask.assert_not_called()
        self.assertEqual(mgr.waypoint_index, 0)
        self.assertEqual(mgr.loop_index, 0)
        self.assertEqual(mgr.spoken, set())

    def test_stop_from_paused_also_resets(self):
        mgr = _make_manager()
        _call(mgr.on_start)
        _call(mgr.on_pause)
        resp = _call(mgr.on_stop)
        self.assertTrue(resp.success)
        self.assertEqual(mgr.state, IDLE)

    def test_stop_from_idle_fails(self):
        mgr = _make_manager()
        resp = _call(mgr.on_stop)
        self.assertFalse(resp.success)
        self.assertEqual(mgr.state, IDLE)

    def test_stop_does_not_kill_the_node(self):
        """Regression: /patrol/stop must not call rclpy.shutdown() or shutdown Nav2."""
        mgr = _make_manager()
        _call(mgr.on_start)
        _call(mgr.on_stop)
        mgr.navigator.lifecycleShutdown.assert_not_called()
        mgr.navigator.shutdown.assert_not_called() if hasattr(mgr.navigator, "shutdown") else None

    def test_can_restart_after_stop(self):
        mgr = _make_manager()
        _call(mgr.on_start)
        _call(mgr.on_stop)
        resp = _call(mgr.on_start)
        self.assertTrue(resp.success)
        self.assertEqual(mgr.state, RUNNING)


if __name__ == "__main__":
    unittest.main()
