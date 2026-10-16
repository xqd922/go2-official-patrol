"""GUI 结构守卫（不打开窗口，因此不需要 X display）。

这里不实例化 tkinter 控件，只断言：
  * 模块可 import（tkinter 是 stdlib，无第三方前端依赖）；
  * 需求里要求的 10 个按钮文案确实出现在界面代码里；
  * 巡检按钮复用现有 /patrol/{start,pause,resume,stop}，没有自造状态机；
  * app.repo_root() 能正确定位仓库根。
"""
import inspect
import os

from patrol_gui import app, gui


REQUIRED_BUTTONS = [
    "开始建图",
    "保存地图",
    "启动导航",
    "记录当前位置为巡检点",
    "查看巡检点",
    "删除选中的巡检点",
    "开始巡检",
    "暂停巡检",
    "继续巡检",
    "停止巡检",
]


def test_gui_module_imports_without_display():
    assert hasattr(gui, "PatrolGui")


def test_all_required_buttons_present_in_source():
    source = inspect.getsource(gui)
    for label in REQUIRED_BUTTONS:
        assert label in source, f"缺少按钮：{label}"


def test_patrol_buttons_reuse_existing_services():
    """巡检控制必须走现有四个 /patrol 服务，不得自写状态机。"""
    source = inspect.getsource(gui)
    for action in ("start", "pause", "resume", "stop"):
        assert f'"{action}"' in source
    assert "call_patrol" in source


def test_gui_does_not_hardcode_offline_static_tf():
    """GUI 不得含 map->odom 逻辑：离线静态定位只能在 offline launch 里。"""
    source = inspect.getsource(gui)
    assert "static_transform_publisher" not in source
    assert "map_to_odom" not in source


def test_repo_root_points_at_workspace():
    root = app.repo_root()
    assert os.path.isdir(os.path.join(root, "src", "go2_patrol_bringup"))
    assert os.path.isfile(os.path.join(root, "scripts", "save_map.sh"))


def test_app_argparse_has_no_ros_flag():
    parser_source = inspect.getsource(app.main)
    assert "--no-ros" in parser_source
