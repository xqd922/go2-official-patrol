"""用假的 tkinter 垫片实例化 PatrolGui，无显示器也能跑通全部界面装配。

沙箱/CI 里没有 X display（`tk.Tk()` 会抛 TclError），但界面装配代码本身
（_build_widgets / refresh_maps / refresh_waypoints / _tick_status / 各按钮
回调）才是最容易写错的部分。这里注入一个最小 tkinter 替身，把 PatrolGui
真正构造出来并点一遍按钮回调，从而在没有窗口的环境里抓出拼写/属性错误。
"""
import os
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
        self._canvas_items = []

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

    def pack_propagate(self, *a, **k):
        return self

    def grid_propagate(self, *a, **k):
        return self

    def columnconfigure(self, *a, **k):
        return self

    def rowconfigure(self, *a, **k):
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

    def selection_set(self, *items):
        self._values = [i for i in items if i]
        return self

    # Text
    def state(self, *a, **k):
        return self

    # Canvas：记录绘制项，便于断言"地图/机器人/航点画出来了"
    def winfo_width(self):
        return 720

    def winfo_height(self):
        return 520

    def create_image(self, *a, **k):
        self._canvas_items.append(("image", k))
        return len(self._canvas_items)

    def create_oval(self, *a, **k):
        self._canvas_items.append(("oval", a, k))
        return len(self._canvas_items)

    def create_rectangle(self, *a, **k):
        self._canvas_items.append(("rect", a, k))
        return len(self._canvas_items)

    def create_line(self, *a, **k):
        self._canvas_items.append(("line", a, k))
        return len(self._canvas_items)

    def create_text(self, *a, **k):
        self._canvas_items.append(("text", a, k))
        return len(self._canvas_items)

    def bbox(self, *a, **k):
        return (0, 0, 10, 10)

    def tag_raise(self, *a, **k):
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
    tk.Canvas = _Widget
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
    def __init__(self, pose=(1.23, 0.82, 0.0), map_info=None, buffer=None):
        self._pose = pose
        self._status = "IDLE"
        self._goal = ""
        self.calls = []
        self._map_info = map_info
        self._buffer = buffer

    def map_info(self):
        return self._map_info

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
        self._running = set()

    def running_modes(self):
        return list(self._running)

    def start_mapping(self):
        self.started.append("mapping")
        self._running.add("mapping")
        return 1

    def start_navigation(self, path, mode=None, rviz=True):
        self.started.append(("navigation", path, mode, rviz))
        self._running.add("navigation")
        return 2

    def stop_all(self, *_a, **_k):
        self.stopped += 1
        self._running.clear()

    def log_path(self, mode):
        return f"/tmp/{mode}.log"


def _make(tmp_path, gui, bridge=None):
    maps = tmp_path / "src" / "go2_patrol_bringup" / "maps"
    maps.mkdir(parents=True, exist_ok=True)
    (maps / "office.yaml").write_text("image: office.pgm\n")
    (tmp_path / "src" / "go2_patrol_bringup" / "config").mkdir(
        parents=True, exist_ok=True)
    launcher = _FakeLauncher()
    app = gui.PatrolGui(_Widget(), str(tmp_path), bridge=bridge or _FakeBridge(),
                        launch_manager=launcher, logger=lambda *_: None)
    # 校验等待时长收紧，否则每个校验用例都要等满 60s / 8s。
    app.nav_verify_timeout = 0.3
    app.nav_verify_interval = 0.05
    app.tf_verify_grace = 0.3
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


# ===================== 启动导航：地图/TF 校验 =====================

def test_start_navigation_forwards_selected_map_path(tmp_path, gui_module):
    """核心：GUI 选中的地图绝对路径必须原样传给 launcher（不许写死 offline_room）。"""
    app, launcher = _ready_app(tmp_path, gui_module, map_name="lab")
    app.on_start_launch(gui_module.NAVIGATION)
    _drain(app)
    kind, path, mode, _rviz = launcher.started[-1]
    assert kind == "navigation"
    assert path.endswith("maps/lab.yaml")
    assert os.path.isabs(path)
    # 传给 launch 的路径要和界面选中的地图是同一个文件
    assert path == app.selected_map_path()


def test_nav_verify_reports_map_not_loaded(tmp_path, gui_module):
    """map_server 没起来时必须明确报“地图未加载”，不能因为进程活着就说已启动。"""
    bridge = _FakeBridge(map_info=None)     # 一直收不到 /map
    app, launcher = _ready_app(tmp_path, gui_module, bridge=bridge)
    app.on_start_launch(gui_module.NAVIGATION)
    _drain(app, seconds=5.0)
    # 断言后台校验回调跑完，并给出失败提示
    _drain(app, seconds=5.0)
    assert "地图未加载" in app.var_hint.get()


def test_nav_verify_reports_loaded_map_and_tf(tmp_path, gui_module):
    """地图加载成功 + TF 链完整时：/map 尺寸要出现，TF 状态显示为“正常”。"""
    class _Buffer:
        def lookup_transform(self, parent, child, time_point=None):
            return object()

    bridge = _FakeBridge(
        pose=(0.0, 0.0, 0.0),
        map_info={"width": 40, "height": 30, "resolution": 0.05,
                  "origin_x": "-1.0", "origin_y": "-1.0",
                  "frame_id": "map", "age": 0.1},
        buffer=_Buffer())
    app, _ = _ready_app(tmp_path, gui_module, bridge=bridge)
    app.on_start_launch(gui_module.NAVIGATION)
    _drain(app, seconds=5.0)
    app._tick_status()          # 后台 TF 状态刷新一次
    hint = app.var_hint.get()
    assert "40x30" in hint
    assert "TF：map->base_footprint 正常" in hint
    assert "TF 链缺失" not in hint


def test_nav_verify_warns_on_map_size_mismatch(tmp_path, gui_module):
    """/map 与选中地图尺寸不一致时要明确警告（防止“加载了别的图”被当成成功）。"""
    class _Buffer:
        def lookup_transform(self, parent, child, time_point=None):
            return object()

    bridge = _FakeBridge(
        map_info={"width": 999, "height": 999, "resolution": 0.05,
                  "origin_x": "0", "origin_y": "0",
                  "frame_id": "map", "age": 0.1},
        buffer=_Buffer())
    app, _ = _ready_app(tmp_path, gui_module, bridge=bridge)
    app.on_start_launch(gui_module.NAVIGATION)
    _drain(app, seconds=5.0)
    assert "⚠" in app.var_hint.get()


def test_nav_verify_missing_tf_shows_waiting_not_failure(tmp_path, gui_module):
    """TF 暂不可用（比如刚启动 buffer 还没填满）只能显示“等待 TF”，

    绝不能永久判定“TF 链缺失 / 链路故障”。这是本次修复的核心回归点。
    """
    class _Buffer:
        def lookup_transform(self, parent, child, time_point=None):
            if (parent, child) == ("map", "odom"):
                raise RuntimeError("no map->odom yet")
            return object()

    bridge = _FakeBridge(
        pose=None,              # ros_bridge 还没拿到 map->base_footprint
        map_info={"width": 40, "height": 30, "resolution": 0.05,
                  "origin_x": "-1.0", "origin_y": "-1.0",
                  "frame_id": "map", "age": 0.1},
        buffer=_Buffer())
    app, _ = _ready_app(tmp_path, gui_module, bridge=bridge)
    app.on_start_launch(gui_module.NAVIGATION)
    _drain(app, seconds=5.0)
    app._tick_status()
    hint = app.var_hint.get()
    # 地图仍要正常显示
    assert "40x30" in hint
    # TF 只能显示"等待"，不能是"缺失/故障"
    assert "等待" in hint and "base_footprint" in hint
    assert "TF 链缺失" not in hint
    assert "无法显示" not in hint


def test_tf_status_self_heals_from_waiting_to_ok(tmp_path, gui_module):
    """核心修复回归：TF 一开始不可用（刚启动）→ 之后可解析，提示自动转“正常”。

    旧行为：启动时检查一次失败就永久写死“TF 链缺失”，哪怕 TF 早已正常。
    新行为：后台每帧重查，一旦 map->base_footprint 可解析就自愈。
    """
    class _Buffer:
        def lookup_transform(self, parent, child, time_point=None):
            raise RuntimeError("buffer 还没填满")

    bridge = _FakeBridge(
        pose=None,
        map_info={"width": 40, "height": 30, "resolution": 0.05,
                  "origin_x": "-1.0", "origin_y": "-1.0",
                  "frame_id": "map", "age": 0.1},
        buffer=_Buffer())
    app, _ = _ready_app(tmp_path, gui_module, bridge=bridge)
    app.on_start_launch(gui_module.NAVIGATION)
    _drain(app, seconds=5.0)
    app._tick_status()
    # 第一阶段：TF 不可用 -> 显示“等待”，不是“缺失/故障”
    assert "等待" in app.var_hint.get()
    assert "TF 链缺失" not in app.var_hint.get()

    # 模拟 TF 稍后就绪：ros_bridge 的 pose() 从 None 变成真实位姿。
    bridge._pose = (1.50, 0.25, 0.30)
    app._tick_status()
    hint = app.var_hint.get()
    assert "TF：map->base_footprint 正常" in hint
    assert "1.50" in hint and "0.30" in hint
    assert "TF 链缺失" not in hint
    assert "等待" not in hint

    # 再模拟 TF 又暂时掉了：应回落到“等待”，仍不写死故障。
    bridge._pose = None
    app._tick_status()
    assert "等待" in app.var_hint.get()
    assert "TF 链缺失" not in app.var_hint.get()


def test_tf_status_keeps_updating_robot_pose_in_hint(tmp_path, gui_module):
    """TF 正常后，提示里的 x/y/yaw 必须持续跟着最新位姿走（实时更新）。"""
    class _Buffer:
        def lookup_transform(self, parent, child, time_point=None):
            return object()

    bridge = _FakeBridge(
        pose=(0.0, 0.0, 0.0),
        map_info={"width": 40, "height": 30, "resolution": 0.05,
                  "origin_x": "-1.0", "origin_y": "-1.0",
                  "frame_id": "map", "age": 0.1},
        buffer=_Buffer())
    app, _ = _ready_app(tmp_path, gui_module, bridge=bridge)
    app.on_start_launch(gui_module.NAVIGATION)
    _drain(app, seconds=5.0)
    app._tick_status()
    assert "x=0.00" in app.var_hint.get()
    # 机器人移动后，提示同步更新（从 (0,0) 走向 (2,0)）
    bridge._pose = (2.00, -0.10, 0.05)
    app._tick_status()
    hint = app.var_hint.get()
    assert "x=2.00" in hint and "y=-0.10" in hint
    # 顶部状态条的位姿也要同步
    assert "x=2.00" in app.var_pose.get()


def test_tf_status_not_clobbered_by_stop(tmp_path, gui_module):
    """停止导航后不再继续刷新导航校验提示（避免残留旧状态）。"""
    class _Buffer:
        def lookup_transform(self, parent, child, time_point=None):
            return object()

    bridge = _FakeBridge(
        pose=(0.0, 0.0, 0.0),
        map_info={"width": 40, "height": 30, "resolution": 0.05,
                  "origin_x": "-1.0", "origin_y": "-1.0",
                  "frame_id": "map", "age": 0.1},
        buffer=_Buffer())
    app, _ = _ready_app(tmp_path, gui_module, bridge=bridge)
    app.on_start_launch(gui_module.NAVIGATION)
    _drain(app, seconds=5.0)
    app._tick_status()
    assert app.var_hint.get()          # 有内容
    app.on_stop_launch()
    _drain(app, seconds=3.0)
    assert app.var_hint.get() == ""
    app._tick_status()                 # 再 tick 不应把旧提示写回来
    assert app.var_hint.get() == ""


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


# ===================== 地图可视化 + 点击定点 =====================

def _write_map(tmp_path, name="offline_room", width=40, height=30):
    """在 tmp 工作区里造一张最小可用地图（yaml + pgm）。"""
    maps = tmp_path / "src" / "go2_patrol_bringup" / "maps"
    maps.mkdir(parents=True, exist_ok=True)
    pgm = maps / f"{name}.pgm"
    with open(pgm, "wb") as stream:
        stream.write(b"P5\n%d %d\n255\n" % (width, height))
        stream.write(bytes([205]) * (width * height))
    (maps / f"{name}.yaml").write_text(
        f"image: {name}.pgm\n"
        "resolution: 0.05\n"
        "origin: [-1.0, -1.0, 0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n", encoding="utf-8")
    return f"{name}.yaml"


def _ready_app(tmp_path, gui, bridge=None, map_name="offline_room"):
    """造出 app 并加载地图。"""
    name = _write_map(tmp_path, map_name)
    app, launcher = _make(tmp_path, gui, bridge=bridge)
    app.refresh_maps()
    app.var_map.set(name)
    app.on_load_map()
    return app, launcher


def _drain(app, seconds=3.0):
    """等在后台线程里跑的 _run_async 完成**并执行完回调**。

    只等 _busy 变 False 不够：_busy 在 _poll_results 取出结果时就清了，
    此时回调可能还没跑完。这里改成等结果队列真的空掉。
    """
    import time
    deadline = time.time() + seconds
    while time.time() < deadline:
        app._poll_results()
        if not app._busy and app._results.empty():
            break
        time.sleep(0.02)
    app._poll_results()


def test_load_map_displays_and_reports_size(tmp_path, gui_module):
    app, _ = _ready_app(tmp_path, gui_module)
    assert app.map_view.has_map()
    grid = app.map_view.grid
    assert (grid.width, grid.height) == (40, 30)
    info = app.var_map_info.get()
    assert "已加载" in info
    assert "40x30" in info


def test_load_map_failure_shows_reason_not_crash(tmp_path, gui_module):
    """地图 yaml 损坏时必须明确报错，而不是静默或崩溃。"""
    app, _ = _make(tmp_path, gui_module)
    maps = tmp_path / "src" / "go2_patrol_bringup" / "maps"
    maps.mkdir(parents=True, exist_ok=True)
    (maps / "broken.yaml").write_text("resolution: 0.05\norigin: [0,0,0]\n",
                                      encoding="utf-8")   # 缺 image 字段
    app.refresh_maps()
    app.var_map.set("broken.yaml")
    app.on_load_map()                     # 不应抛异常
    assert "失败" in app.var_map_info.get()
    assert not app.map_view.has_map()


def test_map_click_converts_screen_to_map_coordinates(tmp_path, gui_module):
    """点击地图后，临时点必须是 map 坐标（不是像素、不是屏幕坐标）。"""
    app, _ = _ready_app(tmp_path, gui_module)
    view = app.map_view
    # 先点地图正中心
    cx = view.offset[0] + view.grid.width * view.scale / 2.0
    cy = view.offset[1] + view.grid.height * view.scale / 2.0
    view._on_button(type("E", (), {"x": cx, "y": cy})())
    point = view.pending()
    assert point is not None
    x, y = point
    # 地图 origin=(-1,-1) 尺寸 40x30 res=0.05 -> 中心约 (0.0, -0.25)
    assert -0.1 < x < 0.1
    assert abs(y - (-1.0 + 30 * 0.05 / 2)) < 0.05
    # 提示文字里应带上坐标
    assert "x=" in app.var_pending.get()


def test_map_click_outside_is_ignored(tmp_path, gui_module):
    """点到地图范围外应被忽略，避免生成离谱航点。"""
    app, _ = _ready_app(tmp_path, gui_module)
    view = app.map_view
    view._on_button(type("E", (), {"x": -5000, "y": -5000})())
    assert view.pending() is None


def test_add_map_waypoint_writes_clicked_coordinates(tmp_path, gui_module):
    """核心：地图点击 -> 输入名称 -> 写入 waypoints.yaml，坐标是点击处的 map 坐标。"""
    app, _ = _ready_app(tmp_path, gui_module)
    # 直接给一个已知 map 坐标的临时点
    app.map_view.set_pending((0.75, -0.50))
    app.var_wp_name.set("point_A")
    app.var_yaw.set("1.57")
    app.var_dwell.set("4.0")
    app.on_add_map_waypoint()
    _drain(app)

    waypoints = app.store.list()
    assert [w.name for w in waypoints] == ["point_A"]
    wp = waypoints[0]
    assert abs(wp.x - 0.75) < 1e-6
    assert abs(wp.y - (-0.50)) < 1e-6
    assert abs(wp.yaw - 1.57) < 1e-6
    assert abs(wp.dwell - 4.0) < 1e-6
    # 添加后临时点应清空
    assert app.map_view.pending() is None


def test_add_map_waypoint_requires_click_first(tmp_path, gui_module):
    app, _ = _ready_app(tmp_path, gui_module)
    app.var_wp_name.set("no_click")
    app.on_add_map_waypoint()
    _drain(app, seconds=1.0)
    assert app.store.list() == []      # 没点击就不该写入


def test_add_multiple_waypoints_not_limited_to_two(tmp_path, gui_module):
    """要求里明确：不能写死只有 A/B 两个点。"""
    app, _ = _ready_app(tmp_path, gui_module)
    for index, name in enumerate(["point_A", "point_B", "point_C", "point_D",
                                  "point_E"]):
        app.map_view.set_pending((float(index) * 0.1, 0.0))
        app.var_wp_name.set(name)
        app.on_add_map_waypoint()
        _drain(app)
    names = app.store.names()
    assert names == ["point_A", "point_B", "point_C", "point_D", "point_E"]
    # 地图上也应有 5 个标记
    assert len(app.map_view._waypoints) == 5


def test_waypoints_drawn_on_map_and_labeled_A_B_C(tmp_path, gui_module):
    """已有航点必须画在地图上，并按顺序编号 A/B/C…"""
    app, _ = _ready_app(tmp_path, gui_module)
    app.store.upsert("entrance", 0.0, 0.0, 0.0, home=True)
    app.store.upsert("equipment_zone", 0.5, 0.0, 0.0)
    app.refresh_waypoints()

    drawn = app.map_view._waypoints
    assert [w[0] for w in drawn] == ["entrance", "equipment_zone"]
    assert drawn[0][4] is True           # home 标记透传
    # 列表里 A/B 编号
    labels = [app.tree._items[n][0] for n in app.tree.get_children()]
    assert labels == ["A", "B"]


def test_selecting_list_highlights_map_marker(tmp_path, gui_module):
    app, _ = _ready_app(tmp_path, gui_module)
    app.store.upsert("entrance", 0.0, 0.0, 0.0)
    app.store.upsert("equipment_zone", 0.5, 0.0, 0.0)
    app.refresh_waypoints()
    # 模拟选中 equipment_zone
    app.tree._values = ["equipment_zone"]
    app._on_tree_select()
    assert app.map_view._selected == "equipment_zone"


def test_delete_waypoint_removes_map_marker(tmp_path, gui_module):
    app, _ = _ready_app(tmp_path, gui_module)
    app.store.upsert("entrance", 0.0, 0.0, 0.0)
    app.store.upsert("equipment_zone", 0.5, 0.0, 0.0)
    app.refresh_waypoints()
    assert len(app.map_view._waypoints) == 2

    app.tree._values = ["entrance"]
    app.on_delete_waypoint()
    _drain(app)

    assert app.store.names() == ["equipment_zone"]
    assert [w[0] for w in app.map_view._waypoints] == ["equipment_zone"]


def test_robot_marker_updates_and_handles_missing_tf(tmp_path, gui_module):
    """有 TF 时画机器人；没有 TF 时不崩、显示不可用。"""
    app, _ = _ready_app(tmp_path, gui_module, bridge=_FakeBridge((0.0, 0.0, 0.0)))
    app._update_robot_marker((0.0, 0.0, 0.0))
    assert app.map_view._pose == (0.0, 0.0, 0.0)
    # TF 不可用
    app._update_robot_marker(None)
    assert app.map_view._pose is None


def test_record_current_position_still_works(tmp_path, gui_module):
    """方式 A（记录当前位置）必须保留，并写入同一个 waypoints.yaml。

    普通点默认"自动朝向下一点"：yaw 输入框默认 auto，所以记录时不保存
    机器人当前朝向，而是标记 orientation_mode=auto_next。
    """
    app, _ = _ready_app(tmp_path, gui_module, bridge=_FakeBridge((1.23, 0.82, 0.5)))
    app.var_wp_name.set("from_robot")
    app.on_record_waypoint()
    _drain(app)
    waypoints = app.store.list()
    assert [w.name for w in waypoints] == ["from_robot"]
    assert abs(waypoints[0].x - 1.23) < 1e-6
    # 默认 auto：普通点不再写死 yaw
    assert waypoints[0].orientation_mode == "auto_next"


def test_record_current_position_with_manual_yaw(tmp_path, gui_module):
    """yaw 框填数值时，记录当前位置必须严格使用该 yaw（手动朝向）。"""
    app, _ = _ready_app(tmp_path, gui_module, bridge=_FakeBridge((1.23, 0.82, 0.5)))
    app.var_wp_name.set("device")
    app.var_yaw.set("1.57")
    app.on_record_waypoint()
    _drain(app)
    waypoints = app.store.list()
    assert [w.name for w in waypoints] == ["device"]
    assert abs(waypoints[0].yaw - 1.57) < 1e-6
    assert waypoints[0].orientation_mode == "fixed"
    assert waypoints[0].auto_orient is False


def test_map_click_waypoint_defaults_to_auto_next(tmp_path, gui_module):
    """地图点击新增的普通点默认 auto_next（不再写死 yaw=0）。"""
    app, _ = _ready_app(tmp_path, gui_module)
    app.map_view.set_pending((1.0, -1.0))
    app.var_wp_name.set("point_A")
    app.on_add_map_waypoint()
    _drain(app)
    waypoints = app.store.list()
    assert [w.name for w in waypoints] == ["point_A"]
    assert waypoints[0].orientation_mode == "auto_next"
    assert waypoints[0].auto_orient is True


def test_map_click_waypoint_with_manual_yaw(tmp_path, gui_module):
    """yaw 框填数值 -> fixed，严格保存该 yaw。"""
    app, _ = _ready_app(tmp_path, gui_module)
    app.map_view.set_pending((1.0, -1.0))
    app.var_wp_name.set("point_fixed")
    app.var_yaw.set("3.14")
    app.on_add_map_waypoint()
    _drain(app)
    waypoints = app.store.list()
    assert waypoints[0].orientation_mode == "fixed"
    assert abs(waypoints[0].yaw - 3.14) < 1e-6


def test_map_click_waypoint_invalid_yaw_rejected(tmp_path, gui_module):
    """yaw 既不是 auto 也不是数字时，不写入并给出提示。"""
    app, _ = _ready_app(tmp_path, gui_module)
    app.map_view.set_pending((1.0, -1.0))
    app.var_wp_name.set("bad")
    app.var_yaw.set("left")
    app.on_add_map_waypoint()
    _drain(app)
    assert app.store.names() == []


def test_waypoint_list_shows_orientation(tmp_path, gui_module):
    """列表里能看到 auto 点与非 auto 点的区分。"""
    app, _ = _ready_app(tmp_path, gui_module)
    app.store.upsert("p1", 0.0, 0.0, 0.0, orientation_mode="auto_next")
    app.store.upsert("p2", 1.0, 0.0, 1.57, orientation_mode="fixed")
    app.refresh_waypoints()
    values = {iid: app.tree._items[iid] for iid in app.tree.get_children()}
    # values = (label, name, x, y, yaw, orient)
    assert values["p1"][4] == "auto" and values["p1"][5] == "→下一点"
    assert values["p2"][4] == "1.57" and values["p2"][5] == "固定"


def test_both_methods_write_to_same_file(tmp_path, gui_module):
    """两种定点方式最终落在同一个 waypoints.yaml。"""
    app, _ = _ready_app(tmp_path, gui_module, bridge=_FakeBridge((3.0, 3.0, 0.0)))
    app.map_view.set_pending((0.1, 0.2))
    app.var_wp_name.set("by_click")
    app.on_add_map_waypoint()
    _drain(app)

    app.var_wp_name.set("by_robot")
    app.on_record_waypoint()
    _drain(app)

    assert app.store.names() == ["by_click", "by_robot"]
    assert app.waypoints_path.endswith("waypoints.yaml")
    # 点击得到的坐标来自地图点击，机器人方式来自 TF，两者互不干扰
    by_click = [w for w in app.store.list() if w.name == "by_click"][0]
    by_robot = [w for w in app.store.list() if w.name == "by_robot"][0]
    assert abs(by_click.x - 0.1) < 1e-6
    assert abs(by_robot.x - 3.0) < 1e-6
