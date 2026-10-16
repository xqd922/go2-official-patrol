"""cleanup.py：GUI 退出时的 domain 兜底清理边界。

硬性要求：
  * ROS_DOMAIN_ID 未设置时**什么都不做**（无法界定范围就不动手）；
  * 只清当前 domain，绝不触碰其它 domain；
  * 只按项目链路签名匹配，不会误杀调用它的 shell/终端。
"""
from patrol_gui import cleanup


def test_cleanup_skips_when_domain_unset(monkeypatch):
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    called = []
    monkeypatch.setattr(cleanup.proc, "kill_domain_processes",
                        lambda *a, **k: called.append(a))
    messages = []
    count = cleanup.cleanup_domain(logger=messages.append)
    assert count == 0
    assert called == []                       # 没设 domain 就不许扫描/杀进程
    assert any("跳过" in m for m in messages)


def test_cleanup_targets_only_current_domain(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "42")
    seen = {}

    def fake_kill(domain, signatures, exclude_substrings=(), grace=5.0, log=print):
        seen["domain"] = domain
        seen["signatures"] = signatures
        seen["exclude"] = exclude_substrings
        return 2

    monkeypatch.setattr(cleanup.proc, "kill_domain_processes", fake_kill)
    count = cleanup.cleanup_domain(logger=lambda *_: None)
    assert count == 2
    assert seen["domain"] == "42"
    # 必须覆盖 OFFLINE 与 REAL 两条链路的节点，否则兜底清不干净
    assert "fake_go2" in seen["signatures"]
    assert "patrol_manager" in seen["signatures"]
    assert "amcl" in seen["signatures"]
    # GUI 自身 cmdline 必须在排除名单里，避免把自己算成残留
    assert any("patrol_gui" in marker for marker in seen["exclude"])


def test_cleanup_explicit_domain_overrides_env(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "42")
    seen = {}
    monkeypatch.setattr(cleanup.proc, "kill_domain_processes",
                        lambda domain, *a, **k: seen.setdefault("domain", domain) or 0)
    cleanup.cleanup_domain(logger=lambda *_: None, domain="7")
    assert seen["domain"] == "7"
