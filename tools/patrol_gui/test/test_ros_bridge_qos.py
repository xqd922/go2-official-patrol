"""RosBridge 的 /map 订阅必须是 latched（TRANSIENT_LOCAL）。

对应真实事故：GUI 点【启动导航】后一直显示"地图未加载"，其实 map_server
已经加载好地图了。原因是 map_server 的 /map 是 transient_local（只发一次），
而 GUI 用默认 volatile QoS 订阅，永远收不到那条已发消息。

历史坑：tools/offline_demo/test 与 src/go2_patrol_patrol/test 都会往
sys.modules 里 setdefault 一份 rclpy / rclpy.qos 桩，同一 pytest 进程里
先收集它们就会污染真实的 rclpy.qos，使运行期 `from rclpy.qos import ...`
拿到 MagicMock。因此这里：
  1) 用**源码检查**做不依赖 ROS 的硬断言（永远执行）；
  2) 只有在 rclpy.qos 确实是真模块时才额外做一次运行期断言。
"""
import inspect
import types

from patrol_gui import ros_bridge


def _real_qos_module():
    """返回真实的 rclpy.qos 模块；若是桩或缺失则返回 None。"""
    import sys
    try:
        import rclpy.qos as qos  # noqa: F401
    except Exception:
        return None
    # MagicMock 桩不是 ModuleType；setdefault 塞进去的桩是 ModuleType，
    # 但属性是 MagicMock。用"类是否真存在"来判定。
    if not isinstance(qos, types.ModuleType):
        return None
    if not isinstance(getattr(qos, "DurabilityPolicy", None), type):
        return None
    return qos


def test_latched_qos_source_uses_transient_local():
    """不依赖 ROS 的硬断言：latched_qos 必须用 TRANSIENT_LOCAL + RELIABLE。"""
    src = inspect.getsource(ros_bridge.latched_qos)
    assert "TRANSIENT_LOCAL" in src
    assert "RELIABLE" in src
    assert "KEEP_LAST" in src


def test_latched_qos_runtime_matches_map_server():
    """有真实 rclpy.qos 时，额外验证构造出来的 QoS 取值。"""
    qos = _real_qos_module()
    if qos is None:
        import pytest
        pytest.skip("无真实 rclpy.qos（被桩污染或未 source ROS）")

    got = ros_bridge.latched_qos()
    assert got.durability == qos.DurabilityPolicy.TRANSIENT_LOCAL
    assert got.reliability == qos.ReliabilityPolicy.RELIABLE
    assert got.depth == 1
