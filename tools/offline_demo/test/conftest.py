"""测试夹具：让 fake_go2 能在没有 ROS 运行时的环境下被导入。

fake_go2 顶层 import 了 rclpy / geometry_msgs / unitree_go。这里在
**收集阶段**注入最小桩，并在本目录测试全部结束后立即还原 sys.modules，
避免污染 src/ 下的其它测试（它们需要真实的 ROS 消息类型）。
"""
import os
import sys
import types
from unittest.mock import MagicMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

_STUBBED = ("rclpy", "rclpy.node", "geometry_msgs", "geometry_msgs.msg",
            "sensor_msgs", "sensor_msgs.msg", "unitree_go", "unitree_go.msg")
_saved = {name: sys.modules.get(name) for name in _STUBBED}
_had_fake_go2 = "fake_go2" in sys.modules


class _Node:
    """可被子类化的最小 Node 桩（MagicMock 不能当基类用）。"""

    def __init__(self, *args, **kwargs):
        pass


def _install_stubs():
    def stub(name, attrs):
        module = types.ModuleType(name)
        for attr in attrs:
            setattr(module, attr, MagicMock)
        sys.modules[name] = module
        return module

    stub("rclpy", ["init", "spin", "shutdown"])
    stub("rclpy.node", []).Node = _Node
    stub("geometry_msgs", [])
    stub("geometry_msgs.msg", ["Twist", "PoseStamped", "Quaternion",
                               "TransformStamped", "PoseWithCovarianceStamped"])
    stub("sensor_msgs", [])
    stub("sensor_msgs.msg", ["PointCloud2", "PointField", "LaserScan", "Imu",
                             "BatteryState"])
    stub("unitree_go", [])
    stub("unitree_go.msg", ["LowState", "SportModeState"])


def _restore_stubs():
    for name, original in _saved.items():
        if original is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = original
    if not _had_fake_go2:
        sys.modules.pop("fake_go2", None)


_install_stubs()


def pytest_collection_modifyitems(config, items):
    """确保还原发生在“本目录测试收集完成之后、其它目录收集之前”。

    pytest 的收集顺序是逐目录进行的，因此这里按目录分组处理：
    只保留与本目录相关的测试，并在结束后立刻还原。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    ours = [i for i in items if os.path.abspath(str(i.fspath)).startswith(here)]
    if not ours:
        _restore_stubs()
