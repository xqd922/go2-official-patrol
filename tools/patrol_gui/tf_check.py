"""TF 链健康检查：明确报出**缺哪一段**，而不是静默。

导航模式期望的链（每段都应恰好一个发布者）：

    map -> odom -> base_footprint -> base_link -> lidar

离线（offline_patrol.launch.py）的发布者分工：
    map->odom           static_transform_publisher (map_to_odom)
    odom->base_footprint odom_bridge
    base_footprint->base_link  static_transform_publisher
    base_link->lidar           perception.launch.py 里的静态 TF

无 ROS 依赖即可 import（测试里注入假 buffer），真正的查询逻辑集中在
check_chain()，便于单元测试覆盖"缺哪段"的各种组合。
"""

DEFAULT_CHAIN = ("map", "odom", "base_footprint", "base_link")


class TfCheckResult:
    def __init__(self, ok, missing, present, error=None):
        self.ok = ok
        self.missing = list(missing)     # [(parent, child), ...]
        self.present = list(present)     # [(parent, child), ...]
        self.error = error               # 非 None 时为直接错误说明

    def describe(self):
        """给界面/日志看的一句话说明。"""
        if self.error:
            return self.error
        if self.ok:
            return f"TF 链完整：{' -> '.join(chain_pairs(self.present))}"
        missing = "、".join(f"{p}->{c}" for p, c in self.missing)
        return f"TF 链缺失：{missing}"


def chain_pairs(present):
    """把 [(a,b), (b,c)] 拼成 [a, b, c] 便于展示。"""
    if not present:
        return []
    nodes = [present[0][0]]
    for _parent, child in present:
        nodes.append(child)
    return nodes


def _latest_time():
    """返回"取最新可用变换"的时间参数。

    真实事故（必读）：`tf2_ros.Buffer.lookup_transform()` **不接受 None**。
    只有裸 `tf2.Buffer` 才把 None 当"最新"。传 None 给 tf2_ros 会抛
    `TypeError: time must have sec and nanosec, or nanoseconds.`，于是每一段
    都被判成"缺失"，界面永远显示"TF 链缺失：map->odom、odom->base_footprint、
    base_footprint->base_link"，哪怕 TF 明明正常。这里统一用 rclpy 的
    `Time()`（= 0 = 最新可用变换），与 ros_bridge / tf_markers 的取法完全一致。
    没有 rclpy（纯离线逻辑单测）时返回 None，让假 buffer 自行处理。
    """
    try:
        from rclpy.time import Time
    except Exception:
        return None
    return Time()


def check_chain(buffer, chain=DEFAULT_CHAIN, time_point=None):
    """逐段查 TF；返回 TfCheckResult（缺哪段就报哪段）。"""
    pairs = list(zip(chain, chain[1:]))
    if time_point is None:
        time_point = _latest_time()
    present = []
    missing = []
    for parent, child in pairs:
        try:
            buffer.lookup_transform(parent, child, time_point)
        except Exception:
            missing.append((parent, child))
        else:
            present.append((parent, child))
    return TfCheckResult(ok=not missing, missing=missing, present=present)
