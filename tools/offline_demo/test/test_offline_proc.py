"""offline_proc 基础设施单测（离线，不启动 ROS）。

重点覆盖上一轮真实机暴露的两个问题：
  1. domain 集合里同时出现 None 与字符串时，sorted() 会抛
     TypeError: '<' not supported between instances of 'NoneType' and 'str'。
     format_domains() 必须能安全显示；stale_domains() 必须仍按原始集合判断，
     不能因为格式化把跨-domain 残留掩盖掉。
  2. 进程组清理：terminate_group() 先 SIGTERM、有限等待、仍存活再 SIGKILL。
"""
import os
import subprocess
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import offline_proc  # noqa: E402


# ---------- domain 格式化 / stale 判定 ----------

def test_format_domains_handles_none_and_str():
    # 直接复现真实机崩溃点：{None, "42"} 直接 sorted 会 TypeError。
    with pytest.raises(TypeError):
        sorted({None, "42"})
    # format_domains 必须不抛，且把 None 显示成 <unknown>。
    # 排序按字符串：'4'(0x34) < '<'(0x3c)，故 "42" 在前。
    assert offline_proc.format_domains({None, "42"}) == ["42", "<unknown>"]


def test_format_domains_is_sorted_and_stringly():
    assert offline_proc.format_domains({"7", "42", None}) == ["42", "7", "<unknown>"]
    assert offline_proc.format_domains(set()) == []


def test_stale_domains_ignores_formatting():
    # 全是本次 domain：无残留。
    assert offline_proc.stale_domains({"fake_go2": {"42"}}, expected=("42",)) == {}
    # 空集合（进程不存在）：不算残留。
    assert offline_proc.stale_domains({"fake_go2": set()}, expected=("42",)) == {}
    # 出现别的 domain：算残留。
    stale = offline_proc.stale_domains({"fake_go2": {"7"}}, expected=("42",))
    assert set(stale) == {"fake_go2"}
    # None（读不到 environ）：必须算残留，不能被格式化掩盖。
    stale = offline_proc.stale_domains({"fake_go2": {None}}, expected=("42",))
    assert set(stale) == {"fake_go2"}
    # None 与 "42" 混存：仍算残留（含非本次取值）。
    stale = offline_proc.stale_domains({"fake_go2": {None, "42"}}, expected=("42",))
    assert set(stale) == {"fake_go2"}


# ---------- /proc 读取 ----------

def test_read_cmdline_self_contains_python():
    cmd = offline_proc.read_cmdline(os.getpid())
    assert cmd  # 至少非空
    assert "python" in cmd.lower() or "pytest" in cmd.lower()


def test_read_cmdline_missing_pid_returns_empty():
    assert offline_proc.read_cmdline(999999999) == ""


def test_read_domain_self():
    # 本测试进程的 domain 取决于运行环境；只要求"能读到时是字符串，读不到是 None"。
    value = offline_proc.read_domain(os.getpid())
    assert value is None or isinstance(value, str)


def test_domain_matches_never_matches_none():
    # 读不到 environ 的进程绝不能算匹配，避免误杀其它任务。
    assert offline_proc.domain_matches(999999999, "42") is False


def test_domain_matches_self_only_for_own_domain():
    own = offline_proc.read_domain(os.getpid())
    if own is None:
        # 读不到 environ：任何 domain 都不匹配（宁可漏清也不误杀）。
        assert offline_proc.domain_matches(os.getpid(), "42") is False
    else:
        assert offline_proc.domain_matches(os.getpid(), own) is True
        assert offline_proc.domain_matches(os.getpid(), own + "_other") is False


def test_find_processes_excludes_substrings():
    # 用当前进程 cmdline 里一定出现的子串来构造签名。
    sigs = {"me": "python"}
    found = offline_proc.find_processes(sigs, exclude_substrings=("python",))
    assert all("python" not in cmd for _, _, cmd, _ in found)


# ---------- 进程组管理 ----------

def test_terminate_group_none_pgid_is_noop():
    assert offline_proc.terminate_group(None) is True


def test_group_alive_none_pgid():
    assert offline_proc.group_alive(None) is False


def test_wait_pgid_ready_returns_none_for_dead_pid():
    assert offline_proc.wait_pgid_ready(999999999, timeout=0.3) is None


def test_e2e_scripts_never_killpg_own_group():
    """安全不变量：E2E/清理脚本只能对"隔离出来的 launch 进程组"发信号，
    绝不能 killpg(0) / killpg(os.getpgrp())，否则会连带杀掉调用它的 shell。"""
    here = os.path.dirname(os.path.abspath(__file__))
    for name in ("run_offline_e2e.py", "offline_proc.py", "e2e_offline_patrol.py"):
        text = open(os.path.join(here, "..", name), encoding="utf-8").read()
        assert "killpg(0" not in text
        assert "killpg(os.getpgrp" not in text
        assert "killpg(os.getpid" not in text


@pytest.mark.skipif(os.name != "posix", reason="POSIX 进程组语义")
def test_terminate_group_kills_whole_group_including_children():
    """父进程 setsid 后 fork 一个长睡眠子进程；SIGTERM 整组必须两个都带走。"""
    parent = subprocess.Popen(
        [sys.executable, "-c",
         "import os, subprocess, sys, time\n"
         "os.setsid()\n"
         "child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(600)'])\n"
         "print(child.pid, flush=True)\n"
         "time.sleep(600)\n"],
        stdout=subprocess.PIPE, text=True,
    )
    try:
        child_pid = int(parent.stdout.readline().strip())
        pgid = offline_proc.wait_pgid_ready(parent.pid, timeout=5.0)
        assert pgid == parent.pid
        assert offline_proc.group_alive(pgid)
        assert offline_proc.pid_running(child_pid)

        ok = offline_proc.terminate_group(pgid, grace=5.0, kill_grace=5.0)
        assert ok is True
        # 父与子都应被清掉，且不再属于该进程组。
        # 注意：父进程由本测试持有，未 wait() 前可能是僵尸(Z)，用 pid_running
        # 判"存活"而非 kill(pid,0)（后者对僵尸也返回成功）。
        assert not offline_proc.pid_running(child_pid)
        assert not offline_proc.group_alive(pgid)
    finally:
        try:
            parent.wait(timeout=5)
        except subprocess.TimeoutExpired:
            parent.kill()
            parent.wait(timeout=5)


# ---------- 新增：domain 比较的 int/str 兼容 ----------

def test_domain_matches_accepts_int_and_str_consistently():
    """调用方写 _cleanup_domain(42)（int）时，必须与写 "42"（str）等价。

    真实机 bug：read_domain() 返回 str，而调用方传 int 42，
    `"42" == 42` 恒 False，导致"按 domain 清残留"的安全网从未生效。
    """
    me = os.getpid()
    own = offline_proc.read_domain(me)
    if own is None:
        pytest.skip("读不到本进程 environ，无法验证匹配语义")
    assert offline_proc.domain_matches(me, own) is True          # str
    if own.isdigit():
        assert offline_proc.domain_matches(me, int(own)) is True  # int 也必须匹配
    assert offline_proc.domain_matches(me, own + "_x") is False
    assert offline_proc.domain_matches(999999999, 42) is False
    assert offline_proc.domain_matches(999999999, "42") is False


def test_domain_matches_never_treats_none_as_match():
    # 读不到 domain 的可疑进程绝不能被当成匹配（否则会误杀别的任务）。
    assert offline_proc.domain_matches(999999999, None) is False


# ---------- 新增：残留判定只算"本次运行新起"的进程 ----------

def test_filter_started_since_excludes_historical_entries():
    me = os.getpid()
    now = offline_proc.proc_starttime(me)
    assert now is not None
    entries = [(me, "fake_go2", "cmd", "42")]
    # 基准 == 自己的 starttime：算"本次新起"，保留。
    assert offline_proc.filter_started_since(entries, now) == entries
    # 基准更晚：说明该进程早于本次运行，属历史孤儿，必须剔除。
    assert offline_proc.filter_started_since(entries, now + 1000) == []


def test_filter_started_since_none_baseline_keeps_all():
    # 拿不到基准时不过滤（宁可多报也不漏报）。
    entries = [(1, "a", "c", "42"), (2, "b", "c", "42")]
    assert offline_proc.filter_started_since(entries, None) == entries


def test_filter_started_since_keeps_vanished_pids():
    # 已退出/无权限（starttime 取不到）按"本次新起"处理，避免漏掉真实泄漏。
    entries = [(999999999, "gone", "cmd", "42")]
    assert offline_proc.filter_started_since(entries, 12345) == entries


def test_proc_starttime_self_is_positive_int():
    st = offline_proc.proc_starttime(os.getpid())
    assert isinstance(st, int) and st > 0


def test_proc_starttime_missing_pid_is_none():
    assert offline_proc.proc_starttime(999999999) is None
