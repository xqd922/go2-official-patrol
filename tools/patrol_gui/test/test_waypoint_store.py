"""WaypointStore：复用共享 waypoints 模块的增删查与原子写。

断言 GUI 与巡检节点对 waypoints.yaml 的理解一致（同一个 load/save 实现），
以及"同名重记 = 覆盖而非追加""删除只删选中的那个"。
"""
import os

import pytest
import yaml

from patrol_gui.waypoint_store import WaypointStore, normalize_name, validate_name


def test_normalize_name_collapses_whitespace():
    assert normalize_name("  entrance  ") == "entrance"
    assert normalize_name("equipment  zone") == "equipment_zone"
    assert normalize_name("a\tb") == "a_b"


def test_validate_name_rejects_empty_and_symbol_only():
    assert validate_name("entrance")[0] is True
    assert validate_name("   ")[0] is False
    assert validate_name("---")[0] is False
    assert validate_name("A1")[0] is True


def test_upsert_appends_then_overwrites_same_name(tmp_path):
    path = tmp_path / "waypoints.yaml"
    store = WaypointStore(str(path))

    store.upsert("entrance", 1.0, 2.0, 0.0, home=True)
    store.upsert("equipment_zone", 4.0, 5.0, 1.57)
    names = store.names()
    assert names == ["entrance", "equipment_zone"]

    # 同名重记：坐标更新，顺序与数量不变（不新增重复项）
    store.upsert("entrance", 9.0, 9.0, 3.14)
    assert store.names() == ["entrance", "equipment_zone"]
    entry = [w for w in store.list() if w.name == "entrance"][0]
    assert (entry.x, entry.y) == (9.0, 9.0)
    assert abs(entry.yaw - 3.14) < 1e-9


def test_written_file_is_loadable_by_shared_module(tmp_path):
    """写出的 yaml 必须能被巡检节点用的同一个 load_waypoints 读回来。"""
    from go2_patrol_patrol.waypoints import load_waypoints

    path = tmp_path / "waypoints.yaml"
    WaypointStore(str(path)).upsert("entrance", 1.23, 0.82, 0.0, dwell=4.0, home=True)

    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert raw["waypoints"][0]["name"] == "entrance"
    assert raw["waypoints"][0]["speech"] == "entrance"
    loaded = load_waypoints(str(path))
    assert loaded[0].name == "entrance"
    assert loaded[0].home is True


def test_remove_deletes_only_selected(tmp_path):
    path = tmp_path / "waypoints.yaml"
    store = WaypointStore(str(path))
    store.upsert("A", 0.0, 0.0, 0.0)
    store.upsert("B", 1.0, 0.0, 0.0)
    store.upsert("C", 2.0, 0.0, 0.0)

    remaining, removed = store.remove("B")
    assert removed is True
    assert [w.name for w in remaining] == ["A", "C"]
    assert store.names() == ["A", "C"]

    # 删不存在的名字：不改文件，返回 removed=False
    _, removed_again = store.remove("B")
    assert removed_again is False


def test_upsert_rejects_invalid_name(tmp_path):
    store = WaypointStore(str(tmp_path / "waypoints.yaml"))
    with pytest.raises(ValueError):
        store.upsert("   ", 0.0, 0.0, 0.0)


def test_list_missing_file_returns_empty(tmp_path):
    store = WaypointStore(str(tmp_path / "nope.yaml"))
    assert store.list() == []
    assert store.names() == []


def test_save_is_atomic_no_leftover_temp_files(tmp_path):
    """原子写：目录里不应残留临时文件。"""
    path = tmp_path / "waypoints.yaml"
    store = WaypointStore(str(path))
    for i in range(5):
        store.upsert(f"wp{i}", float(i), 0.0, 0.0)
    leftovers = [n for n in os.listdir(tmp_path) if n.startswith(".waypoints_")]
    assert leftovers == []
