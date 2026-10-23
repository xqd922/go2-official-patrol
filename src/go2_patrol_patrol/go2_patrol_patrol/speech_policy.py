"""播报队列策略（纯逻辑，不依赖 ROS，可直接单测）。

原实现是"正在播 / 冷却期内直接丢掉新请求"：转弯提醒和到点语音撞在一起时，
到点那句就静默消失了，而离线验收只断言"收到过播报"，这种漏播测不出来。
现在改成单槽等待位：占用中的请求排进去，播完立刻补播；过期（>pending_ttl）的
丢掉，但一定打日志——不再有静默丢失。
"""

PLAY = "play"
QUEUE = "queue"
DROP = "drop"

DEFAULT_PENDING_TTL_S = 2.0


def decide(requested: str, playing: str, busy: bool, cooldown_active: bool) -> str:
    """当前这条请求该立刻播、排进等待位、还是丢弃。"""
    if not str(requested or "").strip():
        return DROP
    if requested == playing:
        return DROP          # 正在播的就是这一句，不用排队重播
    if busy or cooldown_active:
        return QUEUE
    return PLAY


def pending_is_fresh(requested_at_ns, now_ns, ttl_s: float = DEFAULT_PENDING_TTL_S) -> bool:
    """等待位里那句还值不值得补播。

    到点语音晚播两秒以上会让人觉得"这话是在说上一个点"，不如丢掉并留日志。
    """
    if requested_at_ns is None:
        return False
    return (int(now_ns) - int(requested_at_ns)) <= int(float(ttl_s) * 1e9)
