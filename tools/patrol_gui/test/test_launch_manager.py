"""LaunchManager 的行为：独立进程组、互斥、整组清理、save_map 调用。

用假的 Popen / subprocess.run，不真的起 ROS。
"""
import os

import pytest

from patrol_gui import launch_manager as lm


class _FakePopen:
    """最小 Popen 替身：记录命令与环境，poll() 可控。"""

    instances = []

    def __init__(self, args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.pid = 40000 + len(_FakePopen.instances)
        self._returncode = None
        _FakePopen.instances.append(self)

    def poll(self):
        return self._returncode

    def wait(self, timeout=None):
        self._returncode = 0
        return 0

    def finish(self):
        self._returncode = 0


@pytest.fixture
def manager(tmp_path, monkeypatch):
    _FakePopen.instances = []
    # wait_pgid_ready 会去查真实 /proc；测试里直接让它返回 pid。
    monkeypatch.setattr(lm.proc, "wait_pgid_ready", lambda pid, timeout=5.0: pid)
    monkeypatch.setattr(lm.proc, "terminate_group",
                        lambda pgid, grace=15.0, log=print: True)
    return lm.LaunchManager(str(tmp_path), popen_factory=_FakePopen,
                            logger=lambda *_: None)


def test_start_mapping_uses_existing_launch(manager):
    manager.start_mapping()
    args = _FakePopen.instances[-1].args
    # 必须复用项目现有 mapping.launch.py，而不是自造一套
    assert args[:3] == ["ros2", "launch", "go2_patrol_bringup"]
    assert args[3] == "mapping.launch.py"
    assert manager.is_running(lm.MAPPING)


def test_start_navigation_passes_map_argument(manager, tmp_path):
    map_path = tmp_path / "maps" / "office.yaml"
    map_path.parent.mkdir(parents=True)
    map_path.write_text("image: office.pgm\n")
    manager.start_navigation(str(map_path))
    args = _FakePopen.instances[-1].args
    assert args[3] == "patrol.launch.py"
    assert args[4] == f"map:={map_path}"


def test_start_navigation_requires_existing_map(manager, tmp_path):
    with pytest.raises(RuntimeError):
        manager.start_navigation(str(tmp_path / "missing.yaml"))
    with pytest.raises(RuntimeError):
        manager.start_navigation("")


def test_new_session_isolates_process_group(manager):
    """必须用 start_new_session 起子进程，否则 killpg 会波及 GUI/终端。"""
    manager.start_mapping()
    assert _FakePopen.instances[-1].kwargs.get("start_new_session") is True


def test_running_modes_reports_active(manager):
    assert manager.running_modes() == []
    manager.start_mapping()
    assert manager.running_modes() == [lm.MAPPING]
    _FakePopen.instances[-1].finish()
    assert manager.running_modes() == []


def test_stop_is_idempotent(manager):
    manager.start_mapping()
    assert manager.stop(lm.MAPPING) is True
    assert manager.stop(lm.MAPPING) is False
    assert manager.running_modes() == []


def test_cannot_start_same_mode_twice(manager):
    manager.start_mapping()
    with pytest.raises(RuntimeError):
        manager.start_mapping()


def test_stop_all_stops_every_mode(manager):
    manager.start_mapping()
    # 直接伪造第二个模式，验证 stop_all 覆盖所有已记录模式
    manager._groups[lm.NAVIGATION] = 12345
    manager._procs[lm.NAVIGATION] = _FakePopen(["x"])
    manager.stop_all()
    assert manager._groups == {}


def test_save_map_invokes_project_script(tmp_path):
    calls = {}

    class _Completed:
        returncode = 0
        stdout = "[save_map] 已保存：x.yaml 与 x.pgm"
        stderr = ""

    def fake_run(cmd, **kwargs):
        calls["cmd"] = cmd
        calls["kwargs"] = kwargs
        return _Completed()

    ws = tmp_path
    (ws / "scripts").mkdir()
    (ws / "scripts" / "save_map.sh").write_text("#!/bin/bash\n")
    ok, output = lm.save_map(str(ws), "office_floor", run=fake_run,
                             logger=lambda *_: None)
    assert ok is True
    assert "已保存" in output
    # 必须调用项目现有脚本，而不是自己实现保存
    assert calls["cmd"][1].endswith("scripts/save_map.sh")
    assert calls["cmd"][2] == "office_floor"


def test_save_map_reports_failure(tmp_path):
    class _Completed:
        returncode = 1
        stdout = ""
        stderr = "map_saver_cli 未找到"

    (tmp_path / "scripts").mkdir()
    (tmp_path / "scripts" / "save_map.sh").write_text("#!/bin/bash\n")
    ok, output = lm.save_map(str(tmp_path), "x", run=lambda *a, **k: _Completed(),
                             logger=lambda *_: None)
    assert ok is False
    assert "map_saver_cli" in output
