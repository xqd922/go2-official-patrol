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


def pytest_collection_finish(session):
    """**全部**收集完成后立刻还原 sys.modules。

    历史坑：之前在 pytest_collection_modifyitems 里按目录判断还原，只有在
    “本目录没有用例”时才还原，于是同一进程里同时收集 tools/offline_demo/test
    与 tools/patrol_gui/test 时，geometry_msgs 的桩会一直留在 sys.modules，
    让后面依赖真实 ROS 消息类型（visualization_msgs -> geometry_msgs.Pose）
    的用例全部 ImportError。

    收集阶段需要桩（让 fake_go2 能 import），而**运行阶段**不再需要：运行
    阶段用的桩模块本身已经绑定到已 import 的对象上。所以这里在收集一结束
    就把真实的 ROS 模块还原回去，保证运行期各测试目录互不污染。
    """
    _restore_stubs()
