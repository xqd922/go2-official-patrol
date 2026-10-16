"""用假的 tkinter 垫片实例化 PatrolGui，无显示器也能跑通全部界面装配。

沙箱/CI 里没有 X display（`tk.Tk()` 会抛 TclError），但界面装配代码本身
（_build_widgets / refresh_maps / refresh_waypoints / _tick_status / 各按钮
回调）才是最容易写错的部分。这里注入一个最小 tkinter 替身，把 PatrolGui
真正构造出来并点一遍按钮回调，从而在没有窗口的环境里抓出拼写/属性错误。
"""
import sys
import types

import pytest


class _Var:
    def __init__(self, value=None, **kwargs):
        self._value = value

    def get(self):
        return self._value

    def set(self, value):
        self._value = value


class _Widget:
    def __init__(self, *args, **kwargs):
        self._items = {}
        self._values = []
        self._options = {}

    # Combobox 等支持 widget["values"] = [...] 的下标访问
    def __setitem__(self, key, value):
        self._options[key] = value

    def __getitem__(self, key):
        return self._options.get(key)

    # 布局 / 生命周期：全部吞掉
    def pack(self, *a, **k):
        return self

    def grid(self, *a, **k):
        return self

    def configure(self, *a, **k):
        return self

    config = configure

    def bind(self, *a, **k):
        return self

    def title(self, *a, **k):
        return self

    def geometry(self, *a, **k):
        return self

    def protocol(self, *a, **k):
        return self

    def after(self, _ms, _cb=None, *a, **k):
        return "id"          # 不真正调度 _tick_status，避免无限递归

    def destroy(self):
        return self

    def see(self, *a, **k):
        return self

    # Treeview
    def heading(self, *a, **k):
        return self

    def column(self, *a, **k):
        return self

    def insert(self, _parent, _index, iid=None, values=None):
        self._items[iid] = values
        self._values.append(iid)
        return iid

    def delete(self, *ids):
        for i in ids:
            self._items.pop(i, None)
            if i in self._values:
                self._values.remove(i)

    def get_children(self):
        return tuple(self._values)

    def selection(self):
        return tuple(self._values[:1])

    # Text
    def state(self, *a, **k):
        return self


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
    messagebox = types.ModuleType("tkinter.messagebox")
    for name in ("showinfo", "showwarning", "showerror", "askyesno", "askokcancel"):
        setattr(messagebox, name, lambda *a, **k: True)
    tk.messagebox = messagebox

    monkeypatch.setitem(sys.modules, "tkinter", tk)
    monkeypatch.setitem(sys.modules, "tkinter.ttk", ttk)
    monkeypatch.setitem(sys.modules, "tkinter.messagebox", messagebox)
    return tk


@pytest.fixture
def gui_module(monkeypatch):
    _install_fake_tkinter(monkeypatch)
    # 重新导入，让 gui.py 顶层的 `import tkinter` 拿到垫片。
    for name in list(sys.modules):
        if name.startswith("patrol_gui"):
            del sys.modules[name]
    import patrol_gui.gui as gui
    return gui


class _FakeBridge:
    def __init__(self, pose=(1.23, 0.82, 0.0)):
        self._pose = pose
        self._status = "IDLE"
        self._goal = ""
        self.calls = []

    def pose(self):
        return self._pose

    def tf_error(self):
        return None

    def status(self):
        return self._status

    def goal(self):
        return self._goal

    def call_patrol(self, action, timeout=15.0):
        self.calls.append(action)
        return True, f"{action} ok"

    def shutdown(self):
        pass


class _FakeLauncher:
    def __init__(self, *_a, **_k):
        self.started = []
        self.stopped = 0

    def running_modes(self):
        return []

    def start_mapping(self):
        self.started.append("mapping")
        return 1

    def start_navigation(self, path):
        self.started.append(("navigation", path))
        return 2

    def stop_all(self, *_a, **_k):
        self.stopped += 1

    def log_path(self, mode):
        return f"/tmp/{mode}.log"


def _make(tmp_path, gui, bridge=None):
    maps = tmp_path / "src" / "go2_patrol_bringup" / "maps"
    maps.mkdir(parents=True)
    (maps / "office.yaml").write_text("image: office.pgm\n")
    (tmp_path / "src" / "go2_patrol_bringup" / "config").mkdir(parents=True)
    launcher = _FakeLauncher()
    app = gui.PatrolGui(_Widget(), str(tmp_path), bridge=bridge or _FakeBridge(),
                        launch_manager=launcher, logger=lambda *_: None)
    return app, launcher


def test_gui_builds_and_lists_maps(tmp_path, gui_module):
    app, _ = _make(tmp_path, gui_module)
    assert app.list_maps() == ["office.yaml"]
    # 默认应选中唯一/最新的地图
    assert app.var_map.get() == "office.yaml"
    assert app.selected_map_path().endswith("maps/office.yaml")


def test_gui_shows_pose_from_bridge(tmp_path, gui_module):
    app, _ = _make(tmp_path, gui_module, bridge=_FakeBridge((2.5, -1.0, 0.5)))
    app._tick_status()
    assert "x=2.50" in app.var_pose.get()
    assert "yaw=0.50" in app.var_pose.get()


def test_record_waypoint_uses_map_pose_and_updates_list(tmp_path, gui_module):
    app, _ = _make(tmp_path, gui_module, bridge=_FakeBridge((1.23, 0.82, 0.0)))
    app.var_wp_name.set("entrance")
    app.on_record_waypoint()
    # 后台线程 + 结果队列：直接驱动一次 poll
    app._results.join = None  # no-op
    import time
    for _ in range(50):
        app._poll_results()
        if app.tree.get_children():
            break
        time.sleep(0.02)
    names = [w.name for w in app.store.list()]
    assert names == ["entrance"]
    wp = app.store.list()[0]
    assert abs(wp.x - 1.23) < 1e-6 and abs(wp.y - 0.82) < 1e-6
    assert "已记录航点：1" in app.var_count.get()


def test_patrol_buttons_call_existing_services(tmp_path, gui_module):
    bridge = _FakeBridge()
    app, _ = _make(tmp_path, gui_module, bridge=bridge)
    app.store.upsert("entrance", 0.0, 0.0, 0.0)   # start 需要至少一个航点
    import time
    for action in ("start", "pause", "resume", "stop"):
        app.on_patrol(action)
        # GUI 串行化操作（_busy）：每点一次要等它跑完再点下一个。
        deadline = time.time() + 3
        while time.time() < deadline and app._busy:
            app._poll_results()
            time.sleep(0.02)
    assert bridge.calls == ["start", "pause", "resume", "stop"]


def test_start_navigation_requires_map_selection(tmp_path, gui_module):
    app, launcher = _make(tmp_path, gui_module)
    app.var_map.set("")            # 清掉默认选择
    app.on_start_launch(gui_module.NAVIGATION)
    import time
    time.sleep(0.1)
    app._poll_results()
    assert launcher.started == []  # 没选地图就不许启动


def test_delete_selected_waypoint(tmp_path, gui_module):
    app, _ = _make(tmp_path, gui_module)
    app.store.upsert("A", 0.0, 0.0, 0.0)
    app.store.upsert("B", 1.0, 0.0, 0.0)
    app.refresh_waypoints()
    app.tree.delete("A")           # 让 selection 只剩 B（假 selection 返回首个）
    app.on_delete_waypoint()
    import time
    deadline = time.time() + 3
    while time.time() < deadline and "B" in app.store.names():
        app._poll_results()
        time.sleep(0.02)
    assert app.store.names() == ["A"]
