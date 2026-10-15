#!/usr/bin/env python3
"""由 geometry.py 的房间定义生成离线演示用静态地图（确定性，可重复生成）。

生成物（默认 tools/offline_demo/site-demo/，即离线演示的站点源目录）：
  map.pgm    占用栅格（0=墙，254=空地）
  map.yaml   Nav2 map_server 描述，origin 让最外圈墙心落在栅格中心

为什么不用真机建图产物：真机 SLAM 产物是带噪声的现场快照，无法复现、也无法在 CI 上跑；
这里的地图与假机器人点云出自同一份几何定义，改一处两边同时变，闭环结果才可信。

用法:
  python3 tools/offline_demo/make_demo_map.py                        # 默认房间（含木箱）
  python3 tools/offline_demo/make_demo_map.py --obstacles "0,2,0.6,0.6,0.6"
  python3 tools/offline_demo/make_demo_map.py --obstacles ""         # 清空空房间

注意：--obstacles 必须和 fake_go2_loop.py 用同一串取值，run_patrol_demo.sh 会代劳。
"""
from __future__ import annotations

import argparse
import os

from geometry import RESOLUTION, room_from_spec

# 默认直接写进离线演示站点目录：apply_site.sh 要求 map.yaml 与地图数据同目录，
# 这样 site-demo 本身就是一份完整的"站点源目录"，不需要再拷来拷去。
DEFAULT_OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "site-demo")
DEFAULT_NAME = "map"


def write_pgm(path: str, occupied: set, width: int, height: int) -> None:
    free = 254
    wall = 0
    with open(path, "wb") as handle:
        handle.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        for row in range(height):
            handle.write(bytes(wall if (col, row) in occupied else free for col in range(width)))


def write_map_yaml(path: str, image_name: str, resolution: float, origin) -> None:
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(
            f"# 由 tools/offline_demo/make_demo_map.py 生成，勿手工编辑\n"
            f"image: {image_name}\n"
            f"mode: trinary\n"
            f"resolution: {resolution}\n"
            f"origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.25\n"
        )


def build(out_dir: str, name: str, resolution: float, room) -> int:
    width, height = room.map_size(resolution)
    origin = room.map_origin(resolution)
    occupied = room.occupied_cells(resolution)
    os.makedirs(out_dir, exist_ok=True)

    image_name = f"{name}.pgm"
    write_pgm(os.path.join(out_dir, image_name), occupied, width, height)
    write_map_yaml(os.path.join(out_dir, f"{name}.yaml"), image_name, resolution, origin)

    print(f"[make_demo_map] 地图 = {os.path.join(out_dir, f'{name}.yaml')}")
    print(
        f"[make_demo_map] {width}x{height} 像素 @{resolution} m/px"
        f"（{width * resolution:.2f}m x {height * resolution:.2f}m），"
        f"origin=[{origin[0]:.3f}, {origin[1]:.3f}]，占用格 {len(occupied)}"
    )
    print(f"[make_demo_map] 房间 墙 x=±{room.half_x:g} y=±{room.half_y:g} 高 {room.height:g}")
    for box in room.obstacles:
        print(
            f"[make_demo_map] 障碍 {box.name}: 中心=({box.x:g}, {box.y:g}) "
            f"{box.size_x:g}x{box.size_y:g}x{box.height:g}"
        )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="生成离线演示静态地图")
    parser.add_argument("--out-dir", default=DEFAULT_OUT_DIR, help="输出目录")
    parser.add_argument("--name", default=DEFAULT_NAME, help="地图文件名前缀")
    parser.add_argument("--resolution", type=float, default=RESOLUTION, help="米/像素")
    parser.add_argument(
        "--obstacles",
        default=None,
        help='障碍规格 "x,y[,size_x[,size_y[,height]]];…"；默认用 geometry.py 里的木箱',
    )
    args = parser.parse_args()
    try:
        room = room_from_spec(args.obstacles)
    except ValueError as exc:
        parser.error(str(exc))
    return build(os.path.abspath(args.out_dir), args.name, args.resolution, room)


if __name__ == "__main__":
    raise SystemExit(main())
