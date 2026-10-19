"""保存地图的「真假成功」判定。

真实机出现过：map_saver 打印
    [ERROR] [map_saver]: Failed to spin map subscription
但 save_map.sh 仍以 0 退出，旧实现于是打印"地图已保存" —— 假成功。
这里确保：只有脚本成功**且** .yaml/.pgm 确实存在且非空，才算成功。
"""
import os

from patrol_gui.launch_manager import map_files_exist
from patrol_gui import gui as gui_module


def _ws_with_maps(tmp_path):
    maps = tmp_path / "src" / "go2_patrol_bringup" / "maps"
    maps.mkdir(parents=True, exist_ok=True)
    return maps


def test_map_files_exist_true_when_both_present(tmp_path):
    maps = _ws_with_maps(tmp_path)
    (maps / "room.yaml").write_text("image: room.pgm\n", encoding="utf-8")
    (maps / "room.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    ok, detail = map_files_exist(str(tmp_path), "room")
    assert ok is True
    assert "确认存在" in detail


def test_map_files_exist_false_when_missing_pgm(tmp_path):
    """脚本返回 0 但没写出 pgm —— 必须判为失败。"""
    maps = _ws_with_maps(tmp_path)
    (maps / "room.yaml").write_text("image: room.pgm\n", encoding="utf-8")
    ok, detail = map_files_exist(str(tmp_path), "room")
    assert ok is False
    assert "room.pgm" in detail


def test_map_files_exist_false_when_yaml_missing(tmp_path):
    maps = _ws_with_maps(tmp_path)
    (maps / "room.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    ok, detail = map_files_exist(str(tmp_path), "room")
    assert ok is False
    assert "room.yaml" in detail


def test_map_files_exist_false_when_empty(tmp_path):
    maps = _ws_with_maps(tmp_path)
    (maps / "room.yaml").write_text("", encoding="utf-8")
    (maps / "room.pgm").write_bytes(b"")
    ok, detail = map_files_exist(str(tmp_path), "room")
    assert ok is False


def test_map_files_exist_false_for_empty_name(tmp_path):
    ok, _ = map_files_exist(str(tmp_path), "")
    assert ok is False


# ---------- GUI 层的成功/失败文案 ----------

class _FakeApp:
    """只借 GUI 的 _after_save_map，避免引入整个 tkinter 界面。"""

    def __init__(self, ws_root):
        self.ws_root = ws_root
        self.maps_dir = os.path.join(ws_root, "src", "go2_patrol_bringup", "maps")
        self.messages = []
        self.logs = []
        self.loaded = []
        self.refreshed = 0
        self.var_save_info = _Var()

    def _log(self, message):
        self.logs.append(str(message))

    def refresh_maps(self):
        self.refreshed += 1

    def on_load_map(self):
        self.loaded.append(True)

    var_map = None


class _Var:
    def __init__(self):
        self.value = ""

    def set(self, value):
        self.value = value

    def get(self):
        return self.value


def _bind_gui_methods(app):
    """把 gui.PatrolGui 的真实方法绑到一个轻量对象上，专测判定逻辑。"""
    import types
    app.var_map = _Var()
    for name in ("_after_save_map",):
        setattr(app, name, types.MethodType(getattr(gui_module.PatrolGui, name), app))
    return app


def test_after_save_map_success_when_files_exist(tmp_path, monkeypatch):
    app = _bind_gui_methods(_FakeApp(str(tmp_path)))
    shown = {}
    monkeypatch.setattr(gui_module.messagebox, "showinfo",
                        lambda *a, **k: shown.setdefault("info", a))
    app._after_save_map(True, {"name": "room", "script_ok": True,
                               "exists": True, "detail": "ok", "output": "done"})
    assert "成功" in app.var_save_info.value
    assert "info" in shown
    assert app.refreshed == 1


def test_after_save_map_failure_when_script_ok_but_files_missing(tmp_path, monkeypatch):
    """最关键的一条：脚本 0 退出但文件没生成，绝不能报成功。"""
    app = _bind_gui_methods(_FakeApp(str(tmp_path)))
    errors = {}
    monkeypatch.setattr(gui_module.messagebox, "showerror",
                        lambda *a, **k: errors.setdefault("err", a))
    app._after_save_map(False, {"name": "room", "script_ok": True, "exists": False,
                                "detail": "未生成文件：room.pgm", "output": ""})
    assert "失败" in app.var_save_info.value
    assert "room.pgm" in app.var_save_info.value
    assert "err" in errors
    assert app.refreshed == 0
    assert app.loaded == []


def test_after_save_map_failure_surfaces_map_saver_error(tmp_path, monkeypatch):
    """脚本失败时要把底层 ERROR 行带给用户，而不是只说"失败"。"""
    app = _bind_gui_methods(_FakeApp(str(tmp_path)))
    monkeypatch.setattr(gui_module.messagebox, "showerror", lambda *a, **k: None)
    app._after_save_map(False, {
        "name": "room", "script_ok": False, "exists": False, "detail": "",
        "output": "some log\n[ERROR] [map_saver]: Failed to spin map subscription\n"})
    assert "失败" in app.var_save_info.value
    assert "Failed to spin map subscription" in app.var_save_info.value
