"""GUI 生命周期：navigation 起来后必须一直活着，只有显式 stop/关窗才结束。

对应真实事故：离线 launch 整套消失，GUI 却仍显示"导航已启动"。这一组测试锁死
"不会因为定时刷新 / 地图刷新 / TF 检查失败 / marker 不可用 / 辅助进程(nav2_starter)
正常退出而误停 navigation"，并要求 launch 意外退出时界面能看到 returncode。

全部用假的 Popen / 假 bridge / 假 tkinter，不真起 ROS。
"""
import os
import sys
import types

import pytest

from patrol_gui import launch_manager as lm


# ---------------- 假 Popen ----------------

class _FakePopen:
    instances = []

    def __init__(self, args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.pid = 50000 + len(_FakePopen.instances)
        self._returncode = None
        self.terminated = False
        _FakePopen.instances.append(self)

    def poll(self):
        return self._returncode

    def wait(self, timeout=None):
        if self._returncode is None:
            self._returncode = 0
        return self._returncode

    def simulate_exit(self, rc):
        """模拟 launch 自身退出（例如被外部结束）。"""
        self._returncode = rc


@pytest.fixture
def manager(tmp_path, monkeypatch):
    _FakePopen.instances = []
    monkeypatch.setattr(lm.proc, "wait_pgid_ready", lambda pid, timeout=5.0: pid)
    recorded = {"terminate": []}

    def fake_terminate(pgid, grace=15.0, log=print, reason=None):
        recorded["terminate"].append((pgid, reason))
        return True

    monkeypatch.setattr(lm.proc, "terminate_group", fake_terminate)
    mgr = lm.LaunchManager(str(tmp_path), popen_factory=_FakePopen,
                           logger=lambda *_: None)
    mgr.recorded = recorded
    return mgr


def _map(tmp_path):
    p = tmp_path / "maps" / "offline_room.yaml"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text("image: offline_room.pgm\n")
    return str(p)


# ---------------- 1. start 后 Popen 保持 alive ----------------

def test_start_navigation_keeps_popen_alive(manager, tmp_path):
    manager.start_navigation(_map(tmp_path), mode=lm.OFFLINE)
    popen = _FakePopen.instances[-1]
    assert popen.poll() is None
    assert manager.is_running(lm.NAVIGATION) is True
    assert lm.NAVIGATION in manager.running_modes()


# ---------------- 2. nav2_starter 退出不影响 navigation ----------------
# （nav2_starter 是 launch 内的 ExecuteProcess，不属于 GUI 直接持有的 Popen；
#   这里用"另一个 mode 的 Popen 退出"与"垃圾回收一个临时 Popen"来证伪误停）

def test_helper_popen_exit_does_not_stop_navigation(manager, tmp_path):
    manager.start_navigation(_map(tmp_path), mode=lm.OFFLINE)
    helper = _FakePopen(["nav2_starter"])       # 模拟辅助进程（非 GUI 持有）
    helper.simulate_exit(0)
    assert manager.is_running(lm.NAVIGATION) is True
    assert manager.recorded["terminate"] == []   # 绝不能触发整组清理


# ---------------- 3. 定时刷新不会停 navigation ----------------

def test_periodic_refresh_does_not_stop_navigation(manager, tmp_path):
    manager.start_navigation(_map(tmp_path), mode=lm.OFFLINE)
    for _ in range(60):            # 模拟 30s（界面上 500ms 一帧）
        manager.running_modes()
        manager.unexpected_exit(lm.NAVIGATION)
    assert manager.is_running(lm.NAVIGATION) is True
    assert manager.recorded["terminate"] == []


# ---------------- 4. 刷新地图列表不会停 navigation ----------------

def test_map_listing_does_not_stop_navigation(manager, tmp_path, monkeypatch):
    _install_fake_tkinter(monkeypatch)
    gui = _import_gui()
    app = _make_gui(tmp_path, gui, manager)
    app.on_start_launch(gui.NAVIGATION)
    _drain(app)
    assert manager.is_running(lm.NAVIGATION) is True
    for _ in range(5):
        app.refresh_maps()
        app.refresh_waypoints()
    assert manager.is_running(lm.NAVIGATION) is True
    assert manager.recorded["terminate"] == []


# ---------------- 5. TF 检查失败不会停 navigation ----------------

def test_tf_check_failure_does_not_stop_navigation(manager, tmp_path, monkeypatch):
    _install_fake_tkinter(monkeypatch)
    gui = _import_gui()

    class _Bridge:
        _buffer = object()          # 非 None，但 lookup 会失败
        def map_info(self):
            return {"width": 121, "height": 161, "resolution": 0.05,
                    "origin_x": "-3.03", "origin_y": "-4.02",
                    "frame_id": "map", "age": 0.1}
        def pose(self): return None
        def tf_error(self): return "缺少 map->odom"
        def status(self): return "IDLE"
        def goal(self): return ""

    app = _make_gui(tmp_path, gui, manager, bridge=_Bridge())
    # 让 TF 链检查必定失败
    monkeypatch.setattr(app, "_check_tf_chain",
                        lambda: __import__("patrol_gui.tf_check",
                                           fromlist=["TfCheckResult"]
                                           ).TfCheckResult(False, [("map", "odom")], []))
    app.on_start_launch(gui.NAVIGATION)
    _drain(app, seconds=3.0)
    assert manager.is_running(lm.NAVIGATION) is True
    assert manager.recorded["terminate"] == []


# ---------------- 6. marker 不可用不会停 navigation ----------------

def test_marker_unavailable_does_not_stop_navigation(manager, tmp_path, monkeypatch):
    _install_fake_tkinter(monkeypatch)
    gui = _import_gui()

    class _Bridge:
        _buffer = None             # 没有 TF buffer -> marker 不可用
        def map_info(self): return None
        def pose(self): return None
        def tf_error(self): return "无 TF"
        def status(self): return "IDLE"
        def goal(self): return ""

    app = _make_gui(tmp_path, gui, manager, bridge=_Bridge())
    app._tick_status()
    assert manager.is_running(lm.NAVIGATION) is False  # 还没启动
    app.on_start_launch(gui.NAVIGATION)
    _drain(app, seconds=2.0)
    app._tick_status()
    assert manager.is_running(lm.NAVIGATION) is True
    assert manager.recorded["terminate"] == []


# ---------------- 7. 只有显式 stop / close 才结束 ----------------

def test_only_explicit_stop_ends_navigation(manager, tmp_path, monkeypatch):
    _install_fake_tkinter(monkeypatch)
    gui = _import_gui()
    app = _make_gui(tmp_path, gui, manager)
    app.on_start_launch(gui.NAVIGATION)
    _drain(app)
    assert manager.is_running(lm.NAVIGATION) is True

    # 显式停止
    app.on_stop_launch()
    _drain(app)
    assert manager.is_running(lm.NAVIGATION) is False
    assert any("user" in (r or "") for _, r in manager.recorded["terminate"])


def test_close_stops_owned_navigation_only(manager, tmp_path, monkeypatch):
    _install_fake_tkinter(monkeypatch)
    gui = _import_gui()
    app = _make_gui(tmp_path, gui, manager)
    app.on_start_launch(gui.NAVIGATION)
    _drain(app)
    assert manager.is_running(lm.NAVIGATION) is True
    app.on_close()                     # 假 messagebox 返回 True
    assert manager.is_running(lm.NAVIGATION) is False


# ---------------- 8. 意外退出要报告 returncode ----------------

def test_unexpected_exit_reported_with_returncode(manager, tmp_path):
    manager.start_navigation(_map(tmp_path), mode=lm.OFFLINE)
    popen = _FakePopen.instances[-1]
    popen.simulate_exit(-9)            # 被 SIGKILL
    msg = manager.unexpected_exit(lm.NAVIGATION)
    assert msg is not None
    assert "returncode=-9" in msg
    assert "SIGKILL" in msg


def test_user_stop_is_not_reported_as_unexpected(manager, tmp_path):
    manager.start_navigation(_map(tmp_path), mode=lm.OFFLINE)
    popen = _FakePopen.instances[-1]
    manager.stop(lm.NAVIGATION)        # 用户主动停止
    popen.simulate_exit(-15)
    assert manager.unexpected_exit(lm.NAVIGATION) is None


def test_gui_shows_unexpected_exit_in_hint(manager, tmp_path, monkeypatch):
    _install_fake_tkinter(monkeypatch)
    gui = _import_gui()
    app = _make_gui(tmp_path, gui, manager)
    app.on_start_launch(gui.NAVIGATION)
    _drain(app)
    _FakePopen.instances[-1].simulate_exit(1)
    app._tick_status()
    assert "意外退出" in app.var_hint.get()


# ---------------- 9. cleanup 不能误伤 shell / 其它 domain ----------------

def test_domain_cleanup_skips_other_domain_and_shell(monkeypatch):
    from patrol_gui import proc

    fake_procs = [
        (100, "fake_go2", "/usr/bin/python3 .../fake_go2.py --ros-args", "42"),
        (101, "fake_go2", "/usr/bin/python3 .../fake_go2.py --ros-args", "7"),   # 别的 domain
        (102, "fake_go2", "/bin/bash -c source setup_offline.sh; fake_go2", "42"),  # 调用方 shell
    ]
    # 让假 find_processes 也遵守 exclude_substrings（真实实现会按子串排除
    # 调用方 shell），否则测不到"不误伤 shell"这条契约。
    def fake_find(sigs, exclude_substrings=()):
        return [t for t in fake_procs
                if not any(x in t[2] for x in exclude_substrings)]
    monkeypatch.setattr(proc, "find_processes", fake_find)
    # domain 来自 /proc/<pid>/environ；这里用假 pid，必须一并打桩。
    monkeypatch.setattr(proc, "domain_matches",
                        lambda pid, domain: {100: "42", 101: "7", 102: "42"}.get(pid) == str(domain))
    killed = []
    monkeypatch.setattr(proc.os, "kill", lambda pid, sig: killed.append(pid))
    monkeypatch.setattr(proc, "pid_running", lambda pid: False)

    count = proc.kill_domain_processes("42", {"fake_go2": "fake_go2.py"},
                                       exclude_substrings=("setup_offline.sh",))
    # 只应清 domain=42 且不是调用方 shell 的那一个
    assert killed == [100]
    assert count == 1


def test_current_domain_none_does_nothing(monkeypatch):
    from patrol_gui import cleanup
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    called = []
    monkeypatch.setattr(cleanup.proc, "kill_domain_processes",
                        lambda *a, **k: called.append(1))
    assert cleanup.cleanup_domain(logger=lambda *_: None) == 0
    assert called == []


# ---------------- 假 tkinter / gui 装配辅助 ----------------

class _Var:
    def __init__(self, value=None, **kwargs):
        self._value = value
    def get(self): return self._value
    def set(self, value): self._value = value


class _Widget:
    """最小 tkinter 控件替身。

    注意：**不能**用 `__getattr__` 兜底返回 callable —— gui 里
    `self.tree.delete(*self.tree.get_children())` 会因此陷入死循环（`*widget`
    触发 `__iter__` 兜底）。Treeview 相关方法必须显式实现。
    """

    def __init__(self, *a, **k):
        self._items = {}; self._values = []; self._options = {}
        self._canvas_items = []

    def __setitem__(self, k, v): self._options[k] = v
    def __getitem__(self, k): return self._options.get(k)
    def pack(self, *a, **k): return self
    def grid(self, *a, **k): return self
    def pack_propagate(self, *a, **k): return self
    def grid_propagate(self, *a, **k): return self
    def columnconfigure(self, *a, **k): return self
    def rowconfigure(self, *a, **k): return self
    def configure(self, *a, **k): return self
    config = configure
    def bind(self, *a, **k): return self
    def title(self, *a, **k): return self
    def geometry(self, *a, **k): return self
    def protocol(self, *a, **k): return self
    def after(self, _ms, _cb=None, *a, **k): return "id"
    def destroy(self): return self
    def see(self, *a, **k): return self
    def heading(self, *a, **k): return self
    def column(self, *a, **k): return self
    def insert(self, _parent, _index, iid=None, values=None):
        self._items[iid] = values; self._values.append(iid); return iid
    def delete(self, *ids):
        for i in ids:
            self._items.pop(i, None)
            if i in self._values:
                self._values.remove(i)
    def get_children(self): return tuple(self._values)
    def selection(self): return tuple(self._values[:1])
    def selection_set(self, *items):
        self._values = [i for i in items if i]; return self
    def state(self, *a, **k): return self
    def winfo_width(self): return 720
    def winfo_height(self): return 520
    def create_image(self, *a, **k):
        self._canvas_items.append(("image", k)); return len(self._canvas_items)
    def create_oval(self, *a, **k):
        self._canvas_items.append(("oval", a, k)); return len(self._canvas_items)
    def create_rectangle(self, *a, **k):
        self._canvas_items.append(("rect", a, k)); return len(self._canvas_items)
    def create_line(self, *a, **k):
        self._canvas_items.append(("line", a, k)); return len(self._canvas_items)
    def create_text(self, *a, **k):
        self._canvas_items.append(("text", a, k)); return len(self._canvas_items)
    def bbox(self, *a, **k): return (0, 0, 10, 10)
    def tag_raise(self, *a, **k): return self


def _install_fake_tkinter(monkeypatch):
    tk = types.ModuleType("tkinter")
    ttk = types.ModuleType("tkinter.ttk")
    for name in ("Frame", "LabelFrame", "Label", "Button", "Entry",
                 "Checkbutton", "Combobox", "Treeview"):
        setattr(ttk, name, _Widget)
    tk.ttk = ttk
    tk.Text = _Widget
    tk.StringVar = _Var
    tk.BooleanVar = _Var
    tk.Tk = _Widget
    tk.Canvas = _Widget
    messagebox = types.ModuleType("tkinter.messagebox")
    for name in ("showinfo", "showwarning", "showerror", "askyesno", "askokcancel"):
        setattr(messagebox, name, lambda *a, **k: True)
    tk.messagebox = messagebox
    monkeypatch.setitem(sys.modules, "tkinter", tk)
    monkeypatch.setitem(sys.modules, "tkinter.ttk", ttk)
    monkeypatch.setitem(sys.modules, "tkinter.messagebox", messagebox)


def _import_gui():
    for name in list(sys.modules):
        if name.startswith("patrol_gui"):
            del sys.modules[name]
    import patrol_gui.gui as gui
    return gui


class _FakeBridge:
    def __init__(self):
        self._buffer = None
    def map_info(self): return None
    def pose(self): return None
    def tf_error(self): return None
    def status(self): return "IDLE"
    def goal(self): return ""
    def shutdown(self): pass


def _make_gui(tmp_path, gui, manager, bridge=None):
    maps = tmp_path / "src" / "go2_patrol_bringup" / "maps"
    maps.mkdir(parents=True, exist_ok=True)
    (maps / "offline_room.yaml").write_text("image: offline_room.pgm\n")
    (tmp_path / "src" / "go2_patrol_bringup" / "config").mkdir(
        parents=True, exist_ok=True)
    app = gui.PatrolGui(_Widget(), str(tmp_path),
                        bridge=bridge or _FakeBridge(),
                        launch_manager=manager, logger=lambda *_: None)
    app.nav_verify_timeout = 0.3
    app.nav_verify_interval = 0.05
    app.refresh_maps()
    app.var_map.set("offline_room.yaml")
    return app


def _drain(app, seconds=3.0):
    import time
    deadline = time.time() + seconds
    while time.time() < deadline:
        app._poll_results()
        if not app._busy and app._results.empty():
            break
        time.sleep(0.02)
    app._poll_results()
