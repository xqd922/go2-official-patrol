#!/usr/bin/env python3
"""离线校验站点资产：点位是否落在房间内、离墙/障碍是否够远、home 是否合规。

生产交付前用同一份逻辑复查现场点位，可以提前发现"点位贴墙、机器人到不了"
这类上机才会暴露的问题（真机上表现为 Nav2 反复规划失败或原地卡死）。

用法:
  python3 tools/offline_demo/check_site.py --site-dir tools/offline_demo/site-demo
  python3 tools/offline_demo/check_site.py --site-dir <站点> --obstacles "1.5,1.5,0.5,0.5,0.5"
"""
from __future__ import annotations

import argparse
import os
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from geometry import RESOLUTION, room_from_spec  # noqa: E402

DEFAULT_SITE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "site-demo")
WALL_MARGIN = 0.55  # 墙净空下限：robot_radius 0.35 + 0.20 余量
OBSTACLE_MARGIN = 0.70  # 障碍净空下限：robot_radius 0.35 + 0.35 余量（不是 inflation 0.45，
#                          按 0.35+0.45=0.80 去"修正"会把 demo 点位 equipment_zone 误判为警告）


def wall_clearance(room, x: float, y: float) -> float:
    return min(room.half_x - abs(x), room.half_y - abs(y))


def obstacle_clearance(room, x: float, y: float) -> float:
    return min((box.plan_distance(x, y) for box in room.obstacles), default=float("inf"))


def check(site_dir: str, room) -> int:
    failures: list[str] = []
    warnings: list[str] = []

    waypoints_path = os.path.join(site_dir, "waypoints.yaml")
    if not os.path.isfile(waypoints_path):
        print(f"[FAIL] 缺少 {waypoints_path}")
        return 1
    with open(waypoints_path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    waypoints = data.get("waypoints") or []
    if not waypoints:
        print("[FAIL] waypoints.yaml 里没有 waypoints 列表")
        return 1

    print(f"[check_site] 站点={site_dir}")
    print(f"[check_site] 房间 墙 x=±{room.half_x:g} y=±{room.half_y:g} 高 {room.height:g}m")
    for box in room.obstacles:
        print(f"[check_site] 障碍 {box.name} 中心=({box.x:g}, {box.y:g}) {box.size_x:g}x{box.size_y:g}x{box.height:g}")

    homes = [item for item in waypoints if item.get("home")]
    if not homes:
        failures.append("没有 home: true 的点位（低电量返航与收工回原点都会缺目标）")
    elif not waypoints[0].get("home"):
        warnings.append(
            f"home 不在第一个点位（第一个是 {waypoints[0].get('name')}）："
            "巡检仍会按列表顺序走到 home 这个点，返航阶段还会再去一次——不影响功能，"
            "但把 home 排第一读起来更清楚（patrol_sequence._resolve_home_index 取的是 home: true）"
        )
    if len(homes) > 1:
        failures.append(f"home 点位有 {len(homes)} 个，应只保留一个")

    for index, item in enumerate(waypoints):
        name = item.get("name", f"#{index}")
        for key in ("name", "x", "y"):
            if key not in item:
                failures.append(f"点位 {name} 缺少字段 {key}")
        if "x" not in item or "y" not in item:
            continue
        x, y = float(item["x"]), float(item["y"])
        wall = wall_clearance(room, x, y)
        obstacle = obstacle_clearance(room, x, y)
        if wall < 0:
            failures.append(f"点位 {name} ({x:.2f}, {y:.2f}) 在房间外，距墙 {wall:.2f}m")
        elif wall < WALL_MARGIN:
            warnings.append(f"点位 {name} ({x:.2f}, {y:.2f}) 距墙仅 {wall:.2f}m（建议 ≥{WALL_MARGIN}m）")
        if obstacle < 0:
            failures.append(f"点位 {name} ({x:.2f}, {y:.2f}) 落在障碍内部")
        elif obstacle < OBSTACLE_MARGIN:
            warnings.append(
                f"点位 {name} ({x:.2f}, {y:.2f}) 距障碍仅 {obstacle:.2f}m（建议 ≥{OBSTACLE_MARGIN}m）"
            )
        cell = room.world_to_cell(x, y, RESOLUTION)
        note = ""
        if cell in room.occupied_cells(RESOLUTION):
            failures.append(f"点位 {name} ({x:.2f}, {y:.2f}) 落在静态地图的占用格上")
            note = " ← 占用格"
        print(
            f"  [ OK ] {name:<16} ({x:+.2f}, {y:+.2f}) yaw={float(item.get('yaw', 0.0)):+.2f} "
            f"dwell={float(item.get('dwell', 0.0)):.1f}s 墙净空={wall:.2f}m 障碍净空={obstacle:.2f}m{note}"
        )

    audio_dir = os.path.join(site_dir, "audio")
    missing_audio = []
    if os.path.isdir(audio_dir):
        for item in waypoints:
            speech = str(item.get("speech") or item.get("name") or "").strip()
            if speech and not os.path.isfile(os.path.join(audio_dir, f"{speech}.wav")):
                missing_audio.append(speech)
    else:
        warnings.append("没有 audio/ 目录，到点只打印日志不发声")
    if missing_audio:
        warnings.append(f"缺少语音文件: {', '.join(missing_audio)}（到点只打印日志）")

    for message in warnings:
        print(f"  [WARN] {message}")
    for message in failures:
        print(f"  [FAIL] {message}")
    print(f"[check_site] 结果: 失败 {len(failures)} 项，警告 {len(warnings)} 项")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description="校验离线演示站点资产")
    parser.add_argument("--site-dir", default=DEFAULT_SITE)
    parser.add_argument("--obstacles", default=None, help="障碍规格，须与假机器狗一致")
    args = parser.parse_args()
    try:
        room = room_from_spec(args.obstacles)
    except ValueError as exc:
        parser.error(str(exc))
    return check(os.path.abspath(args.site_dir), room)


if __name__ == "__main__":
    raise SystemExit(main())
