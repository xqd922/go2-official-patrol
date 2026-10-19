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
                        lambda pgid, grace=15.0, log=print, reason=None: True)
    return lm.LaunchManager(str(tmp_path), popen_factory=_FakePopen,
                            logger=lambda *_: None)


def test_start_mapping_uses_existing_launch(manager):
    manager.start_mapping()
    args = _FakePopen.instances[-1].args
    # 必须复用项目现有 mapping.launch.py，而不是自造一套
    assert args[:3] == ["ros2", "launch", "go2_patrol_bringup"]
    assert args[3] == "mapping.launch.py"
    assert manager.is_running(lm.MAPPING)


def test_current_mode_defaults_to_real(monkeypatch):
    """没有任何离线标志时必须是 REAL，避免离线逻辑渗进真机。"""
    monkeypatch.delenv("GO2_OFFLINE_MODE", raising=False)
    monkeypatch.delenv("GO2_OFFLINE_LOCALHOST_ONLY", raising=False)
    assert lm.current_mode() == lm.REAL
    assert lm.navigation_launch_for(lm.REAL) == "patrol.launch.py"


def test_current_mode_detects_offline(monkeypatch):
    """setup_offline.sh 的两个标志任一存在都判为 OFFLINE。"""
    monkeypatch.delenv("GO2_OFFLINE_LOCALHOST_ONLY", raising=False)
    monkeypatch.setenv("GO2_OFFLINE_MODE", "1")
    assert lm.current_mode() == lm.OFFLINE
    assert lm.navigation_launch_for(lm.OFFLINE) == "offline_patrol.launch.py"

    monkeypatch.delenv("GO2_OFFLINE_MODE", raising=False)
    monkeypatch.setenv("GO2_OFFLINE_LOCALHOST_ONLY", "1")
    assert lm.current_mode() == lm.OFFLINE

    assert lm.navigation_launch_for(lm.OFFLINE) == "offline_patrol.launch.py"


def test_start_navigation_real_uses_patrol_launch(manager, tmp_path):
    """REAL 模式必须仍是 patrol.launch.py（AMCL 真机链路），不能被离线改坏。"""
    map_path = tmp_path / "maps" / "office.yaml"
    map_path.parent.mkdir(parents=True)
    map_path.write_text("image: office.pgm\n")
    manager.start_navigation(str(map_path), mode=lm.REAL)
    args = _FakePopen.instances[-1].args
    assert args[3] == "patrol.launch.py"
    assert args[4] == f"map:={map_path}"
    # REAL 不传 rviz 开关（由 patrol.launch.py 自己决定）
    assert not any(a.startswith("rviz:=") for a in args)


def test_start_navigation_offline_uses_offline_patrol_launch(manager, tmp_path):
    """OFFLINE 必须走 offline_patrol.launch.py：fake_go2 + 静态 map->odom，
    绝不去起 REAL 的 AMCL 链路。"""
    map_path = tmp_path / "src" / "go2_patrol_bringup" / "maps" / "offline_room.yaml"
    map_path.parent.mkdir(parents=True)
    map_path.write_text("image: offline_room.pgm\n")
    manager.start_navigation(str(map_path), mode=lm.OFFLINE)
    args = _FakePopen.instances[-1].args
    assert args[3] == "offline_patrol.launch.py"
    # 完整绝对路径必须传给 launch，不能退回默认地图
    assert f"map:={map_path}" == args[4]
    assert any(a.startswith("rviz:=") for a in args)


def test_start_navigation_map_argument_is_absolute(manager, tmp_path, monkeypatch):
    """GUI 选中的地图必须原样（绝对路径）传给 launch，不许写死 offline_room。"""
    map_path = tmp_path / "maps" / "lab.yaml"
    map_path.parent.mkdir(parents=True)
    map_path.write_text("image: lab.pgm\n")
    monkeypatch.chdir(tmp_path)
    manager.start_navigation("maps/lab.yaml", mode=lm.OFFLINE)
    args = _FakePopen.instances[-1].args
    assert args[4] == f"map:={map_path}"
    assert "lab.yaml" in args[4]
    assert "offline_room" not in args[4]


def test_start_navigation_logs_selected_and_nav_map(manager, tmp_path):
    """启动日志必须能一眼对照 GUI 选的地图和真正传给 Nav2 的地图。"""
    logged = []
    map_path = tmp_path / "maps" / "corridor.yaml"
    map_path.parent.mkdir(parents=True)
    map_path.write_text("image: corridor.pgm\n")
    manager.log = logged.append
    manager.start_navigation(str(map_path), mode=lm.OFFLINE)
    joined = "\n".join(str(x) for x in logged)
    assert f"GUI_SELECTED_MAP={map_path}" in joined
    assert f"NAV_MAP={map_path}" in joined


def test_start_navigation_follows_rviz_flag(manager, tmp_path):
    """GUI 关掉 RViz 开关时，离线 launch 也必须收到 rviz:=false。"""
    map_path = tmp_path / "maps" / "x.yaml"
    map_path.parent.mkdir(parents=True)
    map_path.write_text("image: x.pgm\n")
    manager.start_navigation(str(map_path), mode=lm.OFFLINE, rviz=False)
    args = _FakePopen.instances[-1].args
    assert "rviz:=false" in args


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
