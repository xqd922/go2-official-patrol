"""GUI 自己的 ROS 2 子进程生命周期管理（无 ROS 依赖，便于单测）。

设计目标（对应"GUI 关闭后必须可靠清理、且不误伤"）：

  1. GUI 启动的每个 `ros2 launch` 都跑在**独立会话/进程组**里
     （`os.setsid()`），清理时对整组发信号即可带走 launch 派生的所有节点；
     因为换了会话，调用 GUI 的 shell/终端在另一个会话，绝不会被误杀。
  2. 只对自己启动的进程组发信号；从不 `killpg(0)`、从不按进程名全局 `pkill`。
     另外提供 ROS_DOMAIN_ID 维度的兜底清理，只清理**指定 domain** 的链路进程，
     不会碰到别的 domain 的 ROS 2 任务。
  3. 先 SIGTERM 整组 -> 有限等待 -> 仍存活的再 SIGKILL。

与 tools/offline_demo/offline_proc.py 的关系：那边是 OFFLINE E2E 专用的
清理/统计实现，这里不 import 它，避免 GUI 与 E2E 互相牵制；两处都遵守同一套
"独立进程组 + domain 限定 + SIGTERM→SIGKILL"规则。
"""
import os
import signal
import time


def read_cmdline(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as stream:
            return stream.read().replace(b"\0", b" ").decode(errors="replace")
    except OSError:
        return ""


def read_domain(pid):
    """读进程的 ROS_DOMAIN_ID；读不到（无权限/已退出）返回 None。"""
    try:
        with open(f"/proc/{pid}/environ", "rb") as stream:
            for item in stream.read().split(b"\0"):
                if item.startswith(b"ROS_DOMAIN_ID="):
                    return item.split(b"=", 1)[1].decode()
    except OSError:
        pass
    return None


def pid_running(pid):
    """进程存在且非僵尸。"""
    try:
        with open(f"/proc/{pid}/stat") as stream:
            state = stream.read().rsplit(")", 1)[1].split()[0]
    except (OSError, IndexError):
        return False
    return state != "Z"


def proc_starttime(pid):
    """进程 starttime（自启动起的 clock ticks）；取不到返回 None。"""
    try:
        with open(f"/proc/{pid}/stat") as stream:
            fields = stream.read().rsplit(")", 1)[1].split()
        return int(fields[19])
    except (OSError, IndexError, ValueError):
        return None


def wait_pgid_ready(pid, timeout=5.0):
    """等子进程完成 os.setsid()，此时它的 pgid == 自身 pid。已退出返回 None。"""
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        try:
            if os.getpgid(pid) == pid:
                return pid
        except ProcessLookupError:
            return None
        time.sleep(0.05)
    return None


def group_alive(pgid):
    """进程组里是否还有存活（非僵尸）进程。"""
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


def signal_group(pgid, sig):
    if pgid is None:
        return
    try:
        os.killpg(pgid, sig)
    except (ProcessLookupError, PermissionError, OSError):
        pass


def terminate_group(pgid, grace=15.0, kill_grace=5.0, log=print):
    """SIGTERM 整组 -> 有限等待 -> 仍存活的 SIGKILL 整组。返回是否已清空。"""
    if pgid is None:
        return True
    signal_group(pgid, signal.SIGTERM)
    deadline = time.monotonic() + float(grace)
    while time.monotonic() < deadline:
        if not group_alive(pgid):
            return True
        time.sleep(0.2)
    log(f"[gui] 进程组 {pgid} 在 {grace:.0f}s 内未退出，升级为 SIGKILL")
    signal_group(pgid, signal.SIGKILL)
    deadline = time.monotonic() + float(kill_grace)
    while time.monotonic() < deadline:
        if not group_alive(pgid):
            return True
        time.sleep(0.2)
    return not group_alive(pgid)


def find_processes(signatures, exclude_substrings=()):
    """按 cmdline 子串匹配进程，返回 [(pid, key, cmdline, domain)]。"""
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


def domain_matches(pid, domain):
    """该进程是否属于指定 ROS_DOMAIN_ID。读不到 environ 一律 False（宁可漏清）。"""
    value = read_domain(pid)
    return value is not None and value == str(domain)


def kill_domain_processes(domain, signatures, exclude_substrings=(),
                          grace=5.0, log=print):
    """只清理 `ROS_DOMAIN_ID == domain` 且 cmdline 命中 signatures 的进程。

    用于 GUI 退出时的兜底：万一某个 ros2 launch 没走独立进程组（例如用户手动
    在别处起过同一套离线链路），也不会残留在本 domain 里。绝不匹配调用方
    shell（shell 的 cmdline 不含这些签名），也绝不触碰其它 domain。
    """
    me = os.getpid()
    targets = []
    for pid, key, cmd, _dom in find_processes(signatures,
                                              exclude_substrings=exclude_substrings):
        if pid == me:
            continue
        if domain_matches(pid, domain):
            targets.append((pid, key, cmd))
    if not targets:
        return 0
    for pid, key, cmd in targets:
        log(f"[gui] 清理 domain={domain} 残留 pid={pid} ({key})")
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    deadline = time.monotonic() + float(grace)
    while time.monotonic() < deadline:
        if not any(pid_running(pid) for pid, _, _ in targets):
            return len(targets)
        time.sleep(0.2)
    for pid, _, _ in targets:
        if pid_running(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
    time.sleep(0.5)
    return len(targets)
