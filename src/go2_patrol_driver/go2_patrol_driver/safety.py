"""前置急停的纯逻辑（不依赖 ROS，可直接单测）。

抽出来的原因：原实现里"没有扫描"和"扫描很新但前方很远"都落到同一个 inf 上，
于是点云断流时前置急停会静默失效——/cmd_vel 还在来，狗照走，日志里一个字都没有。
这里把"扫描新不新"和"该减速/停车/急停"拆成两个可测判断。
"""

ACTION_GO = "go"
ACTION_SLOW = "slow"
ACTION_STOP = "stop"
ACTION_EMERGENCY = "emergency"
ACTION_NO_SCAN = "no_scan"


def is_fresh(last_time_ns, now_ns, timeout_s) -> bool:
    """某个周期性输入是不是"足够新"。

    从没收到过（last_time_ns 为 None）算不新鲜——这正是最危险的情况：
    数据断流（点云 / DDS / 导航栈退出）后保护逻辑静默失效，而指令照来。
    """
    if last_time_ns is None:
        return False
    return (int(now_ns) - int(last_time_ns)) <= int(float(timeout_s) * 1e9)


def scan_is_fresh(last_scan_time_ns, now_ns, timeout_s) -> bool:
    """扫描新鲜度（is_fresh 的语义别名，让急停调用点读起来直白）。"""
    return is_fresh(last_scan_time_ns, now_ns, timeout_s)


def front_action(
    minimum: float,
    emergency_distance: float,
    stop_distance: float,
    slow_distance: float,
    fresh: bool = True,
    require_scan: bool = True,
) -> str:
    """按前方最近距离给出这一拍的动作。

    minimum 为 inf 表示前方没有可用回波（不是"路很宽"，要配合 fresh 一起看）。
    """
    if require_scan and not fresh:
        return ACTION_NO_SCAN
    if minimum <= emergency_distance:
        return ACTION_EMERGENCY
    if minimum <= stop_distance:
        return ACTION_STOP
    if minimum < slow_distance:
        return ACTION_SLOW
    return ACTION_GO
