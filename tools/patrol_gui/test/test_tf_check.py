"""TF 链检查：缺哪一段必须明确报出来，不能静默。

对应 gui.py 的 `_check_tf_chain()`：导航起来后如果 map->odom 或
odom->base_footprint 缺失，界面要明确告诉用户缺哪段，而不是显示假的 (0,0)。
"""
import pytest

from patrol_gui import tf_check


class _FakeBuffer:
    """假 TF buffer：只有 lookup 成功的边才在 available 里。

    故意模仿真实 `tf2_ros.Buffer`：**拒绝 time_point=None**（抛 TypeError）。
    真实事故就是 check_chain 传了 None 给 tf2_ros，导致每一段都被判缺失、
    界面永远显示"TF 链缺失"，而假 buffer 若宽容接受 None 就抓不到这个 bug。
    """

    def __init__(self, available):
        self.available = set(available)
        self.calls = []

    def lookup_transform(self, parent, child, time_point=None):
        self.calls.append((parent, child, time_point))
        if time_point is None:
            # 与 tf2_ros 一致：None 不是合法时间，必须抛 TypeError。
            raise TypeError("time must have sec and nanosec, or nanoseconds.")
        if (parent, child) not in self.available:
            raise RuntimeError(f'"{parent}" -> "{child}" 不存在')
        return object()


def test_check_chain_never_passes_none_to_lookup():
    """回归：绝不把 None 当时间传给 lookup_transform（tf2_ros 会 TypeError）。

    这条用例专门防"真实 TF 正常但永远报缺失"的事故：如果 check_chain 又改回
    传 None，这里会直接失败。
    """
    buffer = _FakeBuffer([("map", "odom"), ("odom", "base_footprint"),
                          ("base_footprint", "base_link")])
    result = tf_check.check_chain(buffer)
    assert result.ok is True, result.describe()
    assert all(tp is not None for _p, _c, tp in buffer.calls)


def test_chain_pairs_builds_node_list():
    pairs = [("map", "odom"), ("odom", "base_footprint")]
    assert tf_check.chain_pairs(pairs) == ["map", "odom", "base_footprint"]
    assert tf_check.chain_pairs([]) == []


def test_full_chain_is_ok():
    buffer = _FakeBuffer([("map", "odom"), ("odom", "base_footprint"),
                          ("base_footprint", "base_link")])
    result = tf_check.check_chain(buffer)
    assert result.ok is True
    assert result.missing == []
    assert "map -> odom -> base_footprint -> base_link" in result.describe()


def test_missing_map_to_odom_is_reported_precisely():
    """离线/真机最常见的坑：map->odom 没有发布者（AMCL 没起来 / 静态 TF 没起）。"""
    buffer = _FakeBuffer([("odom", "base_footprint"), ("base_footprint", "base_link")])
    result = tf_check.check_chain(buffer)
    assert result.ok is False
    assert result.missing == [("map", "odom")]
    assert "map->odom" in result.describe()


def test_missing_odom_to_base_footprint_is_reported_precisely():
    """odom_bridge 挂了 / 没发布时的报错。"""
    buffer = _FakeBuffer([("map", "odom"), ("base_footprint", "base_link")])
    result = tf_check.check_chain(buffer)
    assert result.ok is False
    assert result.missing == [("odom", "base_footprint")]
    assert "odom->base_footprint" in result.describe()


def test_multiple_missing_segments_all_listed():
    buffer = _FakeBuffer([])
    result = tf_check.check_chain(buffer)
    assert result.ok is False
    assert len(result.missing) == 3
    text = result.describe()
    assert "map->odom" in text and "odom->base_footprint" in text


def test_check_chain_ok_with_real_tf2_semantics():
    """只要有可用 TF buffer 且各段存在，check_chain 必须报 ok（不因时间参数误判）。"""
    buffer = _FakeBuffer([("map", "odom"), ("odom", "base_footprint"),
                          ("base_footprint", "base_link"), ("base_link", "lidar")])
    result = tf_check.check_chain(buffer, chain=("map", "odom", "base_footprint",
                                                 "base_link", "lidar"))
    assert result.ok is True
    assert "map -> odom -> base_footprint -> base_link -> lidar" in result.describe()


def test_custom_chain_and_error_field():
    buffer = _FakeBuffer([("a", "b")])
    result = tf_check.check_chain(buffer, chain=("a", "b"))
    assert result.ok is True

    forced = tf_check.TfCheckResult(False, [], [], error="缓冲区不可用")
    assert forced.describe() == "缓冲区不可用"
