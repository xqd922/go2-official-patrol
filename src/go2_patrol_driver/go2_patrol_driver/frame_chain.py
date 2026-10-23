"""TF 补链策略（纯逻辑，不依赖 ROS，可直接单测）。

为什么单独放一个模块：`base_footprint → base_link` 这一环判断错了只有两种后果，
而且都要上机才会暴露——真机 TF 树是断的（导航整条链起不来），或者和机器狗自己发的
TF 打架（Nav2 报 TF 冲突）。这种判断必须能单测。
"""

MODE_AUTO = "auto"
MODE_ALWAYS = "always"
MODE_NEVER = "never"

VALID_MODES = (MODE_AUTO, MODE_ALWAYS, MODE_NEVER)

# 记住最近多少次自己发出去的 stamp。
#
# 认领"这条变换是不是我发的"**不能**用时间窗：本节点按 publish_period（默认 1s）
# 重发，上一帧 stamp 离 now 不到 1s，而同机的外部发布者时间戳同样≈now，
# 任何"相差小于 X 秒就算自己"的容差都会把别人的帧一起吞掉——于是
# external_seen 永远为假、auto 模式永远不回退，反而去和机器狗抢发同一段 TF。
# 自己发出去的 stamp 是精确已知的，直接按值匹配即可（回环收到的就是我们设的那个值）。
OURS_MEMORY = 64


def normalize_mode(value) -> str:
    """把 launch 参数 / 环境变量收敛成合法模式；认不出来按 auto 处理（最保守）。"""
    mode = str(value or "").strip().lower()
    return mode if mode in VALID_MODES else MODE_AUTO


def is_ours(stamp_ns, recent_stamps) -> bool:
    """收到的那条变换是不是本节点自己发出去的那一份（按 stamp 精确匹配）。"""
    if stamp_ns is None:
        return False
    return int(stamp_ns) in recent_stamps


def normalize_frame(name) -> str:
    """帧名归一：去掉前导 '/' 与首尾空白。

    tf2 自己解析时会吃掉前导斜杠，但**线上消息里并不保证规范化**。
    逐字比对时若对面发的是 `/base_footprint`，我们就会认不出
    "这段链已经有人发了"，auto 模式继续补链、和对方抢发同一段 TF
    （Nav2 报 TF 冲突）。比对帧名一律先过这里。
    """
    return (name or "").strip().strip("/")


def is_target_link(frame_id, child_frame_id, parent_frame, child_frame) -> bool:
    """这条变换说的就是我们要负责的那一环吗。

    必须**父帧与子帧都比**：只比子帧的话，别人发的 `odom → base_link`
    （或任何 parent 不是 base_footprint 的 base_link 变换）会被当成
    "这段链已经有人负责"，auto 模式从此不补链，而 TF 树其实还是断的。
    """
    return normalize_frame(child_frame_id) == normalize_frame(
        child_frame
    ) and normalize_frame(frame_id) == normalize_frame(parent_frame)


def should_publish(mode: str, external_seen: bool, elapsed_s: float, probe_seconds: float) -> bool:
    """这一拍要不要发补链 TF。

    auto：探测窗口内没人发才补；已经看到别人在发就永远不插手。
    """
    if mode == MODE_NEVER:
        return False
    if mode == MODE_ALWAYS:
        return True
    if external_seen:
        return False
    return float(elapsed_s) >= float(probe_seconds)


def publish_due(last_published_ns, now_ns, publish_period_s: float) -> bool:
    """补链 TF 要不要在这一拍重发（静态 TF 是 latched 的，重发只是给晚加入的节点兜底）。"""
    if last_published_ns is None:
        return True
    return (int(now_ns) - int(last_published_ns)) >= int(float(publish_period_s) * 1e9)
