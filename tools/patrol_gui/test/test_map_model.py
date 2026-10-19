"""地图坐标换算的单元测试 —— 本次升级里最容易算错的部分。

重点覆盖：
  * 图像 Y 轴翻转（左上角 vs 左下角）；
  * origin 偏移 + resolution 缩放是否被正确应用；
  * pixel_to_map / map_to_pixel 互为逆运算；
  * negate 取反对占用判定的影响；
  * yaml 缺字段 / origin yaw != 0 时报错而不是静默算错。
"""
import math
import os

import pytest

from patrol_gui.map_model import MapError, OccupancyGrid, load_map


def _make_grid(width=10, height=4, resolution=0.5,
               origin=(-2.0, -1.0, 0.0), value=255):
    return OccupancyGrid(width, height, resolution, origin,
                         bytes([value]) * (width * height))


def test_origin_is_bottom_left_not_top_left():
    """核心约定：origin 对应**左下角**像素 (0, height-1)，不是左上角 (0,0)。"""
    grid = _make_grid(width=10, height=4, resolution=0.5, origin=(-2.0, -1.0, 0.0))
    # 左下角像素中心 = origin + 半个像素
    x, y = grid.pixel_to_map(0, grid.height - 1)
    assert math.isclose(x, -2.0 + 0.25, abs_tol=1e-9)
    assert math.isclose(y, -1.0 + 0.25, abs_tol=1e-9)


def test_y_axis_is_flipped_between_image_and_map():
    """行号增大（图片往下）=> map 的 y 必须减小。这是最容易搞反的地方。"""
    grid = _make_grid(width=4, height=5, resolution=1.0, origin=(0.0, 0.0, 0.0))
    _, y_top = grid.pixel_to_map(0, 0)              # 图片最上面一行
    _, y_bottom = grid.pixel_to_map(0, grid.height - 1)   # 图片最下面一行
    assert y_top > y_bottom
    # 相邻两行正好差一个 resolution
    assert math.isclose(y_top - y_bottom, (grid.height - 1) * 1.0, abs_tol=1e-9)


def test_column_increases_with_x():
    grid = _make_grid(width=6, height=3, resolution=1.0, origin=(0.0, 0.0, 0.0))
    xs = [grid.pixel_to_map(col, 0)[0] for col in range(6)]
    assert xs == sorted(xs)


def test_pixel_map_round_trip():
    """任意像素点 -> map -> 像素，必须回到原地。"""
    grid = _make_grid(width=37, height=23, resolution=0.05,
                      origin=(-3.03, -4.02, 0.0))
    for col in (0, 1, 5, 18, 36):
        for row in (0, 2, 11, 22):
            x, y = grid.pixel_to_map(col, row)
            col2, row2 = grid.map_to_pixel(x, y)
            assert math.isclose(col2, col, abs_tol=1e-6)
            assert math.isclose(row2, row, abs_tol=1e-6)


def test_resolution_scaling():
    """resolution 决定米/像素，翻倍则同名像素间距翻倍。"""
    a = _make_grid(width=4, height=2, resolution=0.05, origin=(0.0, 0.0, 0.0))
    b = _make_grid(width=4, height=2, resolution=0.10, origin=(0.0, 0.0, 0.0))
    ax = a.pixel_to_map(3, 0)[0] - a.pixel_to_map(0, 0)[0]
    bx = b.pixel_to_map(3, 0)[0] - b.pixel_to_map(0, 0)[0]
    assert math.isclose(bx, 2 * ax, abs_tol=1e-9)


def test_world_size_and_bounds():
    grid = _make_grid(width=121, height=161, resolution=0.05,
                      origin=(-3.03, -4.02, 0.0))
    w, h = grid.world_size()
    assert math.isclose(w, 121 * 0.05, abs_tol=1e-9)
    assert math.isclose(h, 161 * 0.05, abs_tol=1e-9)
    min_x, min_y, max_x, max_y = grid.bounds()
    assert math.isclose(min_x, -3.03, abs_tol=1e-9)
    assert math.isclose(max_x, -3.03 + 121 * 0.05, abs_tol=1e-9)
    assert max_y > min_y


def test_contains_map_excludes_points_outside():
    grid = _make_grid(width=10, height=10, resolution=0.1, origin=(0.0, 0.0, 0.0))
    assert grid.contains_map(0.5, 0.5) is True
    assert grid.contains_map(-1.0, 0.5) is False
    assert grid.contains_map(0.5, 99.0) is False


def test_occupancy_classification_negate_off():
    """negate=0：0=黑=占用，255=白=空闲。"""
    grid = _make_grid(value=0, origin=(0, 0, 0))
    assert grid.classify(0) == "occupied"
    assert grid.classify(255) == "free"
    assert grid.classify(205) == "free"       # 离线房间的浅灰地面


def test_occupancy_classification_negate_on():
    """negate=1 时黑白语义反转。"""
    grid = OccupancyGrid(2, 2, 1.0, (0, 0, 0), bytes([0, 0, 0, 0]), negate=1)
    assert grid.classify(255) == "occupied"
    assert grid.classify(0) == "free"


def test_unknown_between_thresholds():
    grid = _make_grid(value=0, origin=(0, 0, 0))
    # 灰度 128 -> p≈0.498，落在 free(0.25) 与 occupied(0.65) 之间 -> unknown
    assert grid.classify(128) == "unknown"


def test_rejects_nonpositive_resolution():
    with pytest.raises(MapError):
        _make_grid(resolution=0)


def test_rejects_nonpositive_size():
    with pytest.raises(MapError):
        _make_grid(width=0)


# ---------- load_map：真实文件读写 ----------
def _write_pgm(path, width, height, rows):
    """rows: 每行一个 bytes，长度需等于 width。"""
    with open(path, "wb") as stream:
        stream.write(b"P5\n%d %d\n255\n" % (width, height))
        for row in rows:
            stream.write(bytes(row))


def test_load_map_reads_real_yaml_and_pgm(tmp_path):
    pgm = tmp_path / "room.pgm"
    _write_pgm(pgm, 4, 3, [
        [0, 0, 0, 0],
        [255, 255, 255, 255],
        [205, 205, 205, 205],
    ])
    yaml_path = tmp_path / "room.yaml"
    yaml_path.write_text(
        "image: room.pgm\n"
        "resolution: 0.05\n"
        "origin: [-1.5, -2.5, 0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n",
        encoding="utf-8")

    grid = load_map(str(yaml_path))
    assert (grid.width, grid.height) == (4, 3)
    assert math.isclose(grid.resolution, 0.05)
    assert math.isclose(grid.origin_x, -1.5)
    assert math.isclose(grid.origin_y, -2.5)
    # 第一行是黑色（占用），转换后应位于 map 的**上方**
    _, y_top = grid.pixel_to_map(0, 0)
    _, y_bottom = grid.pixel_to_map(0, 2)
    assert y_top > y_bottom
    assert grid.classify(grid.pixels[0]) == "occupied"
    assert grid.classify(grid.pixels[4]) == "free"


def test_load_map_resolves_image_relative_to_yaml(tmp_path):
    """image 用相对路径时，必须相对 yaml 所在目录，而不是当前工作目录。"""
    nested = tmp_path / "maps"
    nested.mkdir()
    _write_pgm(nested / "m.pgm", 2, 2, [[0, 0], [0, 0]])
    (nested / "m.yaml").write_text(
        "image: m.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n", encoding="utf-8")
    grid = load_map(str(nested / "m.yaml"))     # 从别处调用也不受影响
    assert grid.width == 2


def test_load_map_missing_file_raises(tmp_path):
    with pytest.raises(MapError):
        load_map(str(tmp_path / "nope.yaml"))


def test_load_map_missing_image_field_raises(tmp_path):
    yaml_path = tmp_path / "bad.yaml"
    yaml_path.write_text("resolution: 0.05\norigin: [0, 0, 0]\n", encoding="utf-8")
    with pytest.raises(MapError):
        load_map(str(yaml_path))


def test_load_map_missing_pgm_raises(tmp_path):
    yaml_path = tmp_path / "bad.yaml"
    yaml_path.write_text("image: ghost.pgm\nresolution: 0.05\norigin: [0,0,0]\n",
                         encoding="utf-8")
    with pytest.raises(MapError):
        load_map(str(yaml_path))


def test_load_map_rejects_rotated_origin(tmp_path):
    """origin yaw != 0 会让所有换算偏移，必须显式报错而不是静默算错。"""
    _write_pgm(tmp_path / "r.pgm", 2, 2, [[0, 0], [0, 0]])
    yaml_path = tmp_path / "r.yaml"
    yaml_path.write_text(
        "image: r.pgm\nresolution: 0.05\norigin: [0, 0, 1.57]\n", encoding="utf-8")
    with pytest.raises(MapError):
        load_map(str(yaml_path))


def test_load_map_of_project_offline_room():
    """对项目里真实存在的 offline_room 做一次端到端校验。"""
    here = os.path.dirname(os.path.abspath(__file__))
    yaml_path = os.path.abspath(os.path.join(
        here, "..", "..", "..", "src", "go2_patrol_bringup", "maps",
        "offline_room.yaml"))
    if not os.path.isfile(yaml_path):
        pytest.skip("offline_room.yaml 不存在")
    grid = load_map(yaml_path)
    # entrance(0,0) 与 equipment_zone(2,0) 都必须落在地图内
    assert grid.contains_map(0.0, 0.0), "entrance 应在地图内"
    assert grid.contains_map(2.0, 0.0), "equipment_zone 应在地图内"
    # 且这两点应是可通行的（free），否则记录出来的航点没法导航
    for x, y in ((0.0, 0.0), (2.0, 0.0)):
        col, row = grid.map_to_pixel(x, y)
        value = grid.pixels[int(round(row)) * grid.width + int(round(col))]
        assert grid.classify(value) == "free", f"({x},{y}) 不是 free"
