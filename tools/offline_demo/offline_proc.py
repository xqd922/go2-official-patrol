#!/usr/bin/env python3
"""OFFLINE E2E 的进程/进程组基础设施（无 ROS 依赖，便于单测）。

集中放两类容易出错、又必须被测试覆盖的逻辑：
  1. /proc 读取与 domain 处理：ROS_DOMAIN_ID 读不到时返回 None，domain 集合
     里会同时出现 None 与字符串。sorted({None, "42"}) 在 Python 3 会抛
     TypeError，因此显示一律走 format_domains()（把 None 映射成 "<unknown>"），
     而 stale 判定走 stale_domains()（用**原始**集合，绝不被格式化掩盖）。
  2. 进程组管理：launch 跑在独立会话里，清理时对整组发信号，先 SIGTERM、
     有限等待、仍存活的再 SIGKILL。避免只 terminate launch 父进程导致
     ros2 节点变成 PPID=1 的孤儿残留。
"""
import os
import signal
import time

UNKNOWN_DOMAIN = "<unknown>"


def read_cmdline(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as stream:
            return stream.read().replace(b"\0", b" ").decode(errors="replace")
    except OSError:
        return ""


def read_domain(pid):
    """读进程的 ROS_DOMAIN_ID；读不到（无权限 / 已退出）返回 None。"""
    try:
        with open(f"/proc/{pid}/environ", "rb") as stream:
            for item in stream.read().split(b"\0"):
                if item.startswith(b"ROS_DOMAIN_ID="):
                    return item.split(b"=", 1)[1].decode()
    except OSError:
        pass
    return None


def format_domains(domains):
    """把 domain 集合安全格式化成可排序、可打印的字符串列表。

    None -> "<unknown>"，其余转 str，因此 None 与 "42" 混存也不会抛
    TypeError。仅用于日志显示。
    """
    return sorted(UNKNOWN_DOMAIN if d is None else str(d) for d in domains)


def stale_domains(domains, expected=("42",)):
    """返回含"非本次 domain"取值的条目 {key: 原始集合}。

    用原始集合比较，不经过格式化：None（读不到 environ 的可疑进程）也算残留。
    空集合不算残留（进程不存在）。
    """
    expected = set(expected)
    return {k: v for k, v in domains.items() if v and v != expected}


def domain_matches(pid, domain):
    """该进程是否属于指定 ROS_DOMAIN_ID（用于"只清本 domain 残留"）。

    读不到 environ 返回 False —— 宁可漏清也不误杀。绝不把 None 当成匹配。

    domain 统一按**字符串**比较：read_domain() 读出来的是 str，而调用方很容易
    写成 `_cleanup_domain(42)`（int）。`"42" == 42` 在 Python 里恒为 False，
    曾导致"按 domain 清残留"的安全网**从未生效**（真实机 12:26 那批
    static_transform_publisher / voice_player 孤儿就是这么漏掉的）。
    """
    value = read_domain(pid)
    return value is not None and value == str(domain)


def find_processes(signatures, exclude_substrings=()):
    """按 cmdline 子串匹配进程。

    返回 [(pid, key, cmdline, domain)]；exclude_substrings 里的子串命中则跳过
    （例如排除 E2E 自身的 cmdline）。signatures 是 {key: 子串}。
    """
    found = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        cmd = read_cmdline(entry)
        if not cmd or any(s in cmd for s in exclude_substrings):
            continue
        for key, sig in signatures.items():
            if sig in cmd:
                found.append((int(entry), key, cmd, read_domain(entry)))
                break
    return found


def filter_started_since(entries, since_ticks):
    """只保留 starttime >= since_ticks 的条目（本次运行期间新起的进程）。

    entries 是 [(pid, key, cmd, domain)]（如 find_processes() 的返回值）。
    since_ticks 为 None 时原样返回（拿不到基准就不过滤，宁可多报也不漏报）。
    取不到 starttime 的进程（已退出/无权限）按"本次新起"处理，避免漏掉真实泄漏。
    """
    if since_ticks is None:
        return list(entries)
    kept = []
    for entry in entries:
        started = proc_starttime(entry[0])
        if started is None or started >= since_ticks:
            kept.append(entry)
    return kept


def signal_group(pgid, sig):
    """对进程组发信号；组已不存在时静默返回。"""
    if pgid is None:
        return
    try:
        os.killpg(pgid, sig)
    except (ProcessLookupError, PermissionError, OSError):
        pass


def group_alive(pgid):
    """进程组里是否还有**存活**（非僵尸）进程。

    用 /proc 扫描而不是 killpg(pgid, 0)：进程组组长退出后、尚未被父进程
    wait() 回收前是僵尸(Z)态，仍算"组成员"，killpg 会一直返回成功，于是清理
    被误判为"没清干净"，白白多等一轮 SIGKILL。这里显式排除僵尸态。
    """
    if pgid is None:
        return False
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/stat") as stream:
                fields = stream.read().rsplit(")", 1)[1].split()
            state = fields[0]
            pgrp = int(fields[2])
        except (OSError, IndexError, ValueError):
            continue
        if pgrp == pgid and state != "Z":
            return True
    return False


def pid_running(pid):
    """进程是否存在且非僵尸。"""
    try:
        with open(f"/proc/{pid}/stat") as stream:
            state = stream.read().rsplit(")", 1)[1].split()[0]
    except (OSError, IndexError):
        return False
    return state != "Z"


def proc_starttime(pid):
    """返回进程的 starttime（自系统启动起的 clock ticks）；取不到返回 None。

    用于区分"本次 E2E 新起的进程"和"上一轮遗留的孤儿"：残留检测只看本次
    运行期间才出现的进程，避免把历史孤儿误算成本次泄漏。
    """
    try:
        with open(f"/proc/{pid}/stat") as stream:
            fields = stream.read().rsplit(")", 1)[1].split()
        return int(fields[19])
    except (OSError, IndexError, ValueError):
        return None


def wait_pgid_ready(pid, timeout=5.0):
    """等子进程完成 os.setsid()，此时它的 pgid 等于自身 pid。

    拿到确定且隔离的 pgid 后，清理才能精确地只作用于本次 launch 的进程组。
    进程已退出返回 None。
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            if os.getpgid(pid) == pid:
                return pid
        except ProcessLookupError:
            return None
        time.sleep(0.05)
    return None


def terminate_group(pgid, grace=15.0, kill_grace=5.0, log=print):
    """先 SIGTERM 整组 -> 有限等待 -> 仍存活的 SIGKILL 整组。

    返回 True 表示最终进程组已清空。
    """
    if pgid is None:
        return True
    signal_group(pgid, signal.SIGTERM)
    deadline = time.time() + grace
    while time.time() < deadline:
        if not group_alive(pgid):
            return True
        time.sleep(0.2)
    log(f"[driver] 进程组 {pgid} 在 {grace:.0f}s 内未退出，升级为 SIGKILL")
    signal_group(pgid, signal.SIGKILL)
    deadline = time.time() + kill_grace
    while time.time() < deadline:
        if not group_alive(pgid):
            return True
        time.sleep(0.2)
    return not group_alive(pgid)
