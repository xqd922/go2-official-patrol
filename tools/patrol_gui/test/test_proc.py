"""proc.py：进程组管理与 domain 限定清理的正确性。

重点断言"不误伤"：
  * 只对指定 ROS_DOMAIN_ID 的进程动手；
  * 读不到 environ 的进程一律跳过；
  * 调用方自己（GUI 进程）绝不在清理名单里；
  * 独立进程组用 killpg 收，而不是全局按名杀。
"""
import signal

from patrol_gui import proc


def test_read_domain_returns_none_for_missing_process():
    # 一个几乎不可能存在的 pid：读不到 environ 必须是 None，而不是抛异常。
    assert proc.read_domain(2 ** 22 + 12345) is None


def test_pid_running_false_for_missing_process():
    assert proc.pid_running(2 ** 22 + 12345) is False


def test_group_alive_false_for_unused_pgid():
    # 一个没人用的 pgid（很大且不会分配）应当报 False。
    assert proc.group_alive(2 ** 22 + 54321) is False


def test_signal_group_ignores_missing_group():
    # 对不存在的进程组发信号必须静默，不得抛异常。
    proc.signal_group(2 ** 22 + 99999, signal.SIGTERM)


def test_terminate_group_none_is_noop():
    assert proc.terminate_group(None) is True


class _FakeProc:
    def __init__(self, pid, key, cmd, domain):
        self.pid = pid
        self.key = key
        self.cmd = cmd
        self.domain = domain


def test_kill_domain_processes_only_touches_matching_domain(monkeypatch):
    """两个同名链路进程分属不同 domain：只清当前 domain 的那个。"""
    target = _FakeProc(101, "fake_go2", "python3 fake_go2.py", "42")
    other = _FakeProc(202, "fake_go2", "python3 fake_go2.py", "7")
    rows = [
        (target.pid, target.key, target.cmd, target.domain),
        (other.pid, other.key, other.cmd, other.domain),
    ]
    monkeypatch.setattr(proc, "find_processes", lambda *a, **k: rows)
    # domain_matches 会重新读 /proc/<pid>/environ，这里按 pid 伪造。
    monkeypatch.setattr(proc, "read_domain",
                        lambda pid: {101: "42", 202: "7"}.get(pid))
    monkeypatch.setattr(proc, "pid_running", lambda pid: pid == target.pid)
    killed = {}
    monkeypatch.setattr(proc.os, "kill",
                        lambda pid, sig: killed.setdefault(pid, []).append(sig))

    count = proc.kill_domain_processes("42", {"fake_go2": "fake_go2.py"},
                                       grace=0.1, log=lambda *_: None)
    assert count == 1
    assert target.pid in killed
    assert other.pid not in killed           # 别的 domain 绝不动
    assert signal.SIGTERM in killed[target.pid]


def test_kill_domain_processes_skips_unreadable_domain(monkeypatch):
    """domain 读不到（None）时必须跳过，而不是当成当前 domain 杀掉。"""
    unknown = (303, "odom_bridge", "python3 odom_bridge", None)
    monkeypatch.setattr(proc, "find_processes", lambda *a, **k: [unknown])
    monkeypatch.setattr(proc, "pid_running", lambda pid: True)
    killed = []
    monkeypatch.setattr(proc.os, "kill", lambda pid, sig: killed.append(pid))
    count = proc.kill_domain_processes("42", {"odom_bridge": "odom_bridge"},
                                       grace=0.1, log=lambda *_: None)
    assert count == 0
    assert killed == []


def test_kill_domain_processes_never_kills_self(monkeypatch):
    """即使签名命中，也绝不把 GUI 自己（os.getpid()）算进去。"""
    me = proc.os.getpid()
    monkeypatch.setattr(proc, "find_processes",
                        lambda *a, **k: [(me, "fake_go2", "python3 fake_go2.py", "42")])
    monkeypatch.setattr(proc, "pid_running", lambda pid: True)
    killed = []
    monkeypatch.setattr(proc.os, "kill", lambda pid, sig: killed.append(pid))
    count = proc.kill_domain_processes("42", {"fake_go2": "fake_go2.py"},
                                       grace=0.1, log=lambda *_: None)
    assert count == 0
    assert killed == []


def test_domain_matches_is_string_typed(monkeypatch):
    """domain 一律按字符串比较：'42' 命中，int 42 也要命中，None 不命中。"""
    monkeypatch.setattr(proc, "read_domain", lambda pid: "42")
    assert proc.domain_matches(1, "42") is True
    assert proc.domain_matches(1, 42) is True
    monkeypatch.setattr(proc, "read_domain", lambda pid: None)
    assert proc.domain_matches(1, "42") is False
