#!/usr/bin/env python3
"""离线演示共用的世界几何（假传感器数据、静态地图、点位校验的唯一来源）。

坐标系 = 生产环境的 map 坐标系：
  房间中心在原点，机器狗上电位置在原点并朝向 +x，
  墙面在 x=±half_x、y=±half_y，地面 z=0，天花板 z=height。

假机器人扫描用的几何和 site-demo 静态地图由本模块同一份定义生成，
因此"扫出来的墙"与"地图上的墙"天然对齐，AMCL 不会因为几何不一致而发散
（这是自己造数据最容易踩的坑）。

本模块不依赖 ROS，可用纯 python3 运行，便于 pytest/CI 直接校验。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Iterator, Sequence

RESOLUTION = 0.05  # 与生产 nav2_params.yaml 的 costmap resolution 一致
HALF_X = 3.0
HALF_Y = 4.0
HEIGHT = 2.4
SENSOR_Z = 0.13  # base_link -> utlidar_lidar，与 perception.launch.py 的静态 TF 一致
EPS = 1e-6

# 层数从 0° 起：0° 那一层严格落在墙面的水平投影上，投影成 /scan 后与静态地图完全重合；
# 其余层只用于 RViz 里的三维观感，投影后落在同一批角度格里，不会污染 /scan。
DEFAULT_ELEVATIONS_DEG = (0.0, 5.0, 10.0, 15.0, 20.0, 25.0)


@dataclass(frozen=True)
class Box:
    """轴对齐长方体：底面在 z0，高 height，水平中心 (x, y)。"""

    x: float
    y: float
    size_x: float
    size_y: float
    height: float = 0.5
    z0: float = 0.0
    name: str = "box"

    @property
    def lo(self) -> tuple[float, float, float]:
        return (self.x - self.size_x / 2.0, self.y - self.size_y / 2.0, self.z0)

    @property
    def hi(self) -> tuple[float, float, float]:
        return (self.x + self.size_x / 2.0, self.y + self.size_y / 2.0, self.z0 + self.height)

    def plan_distance(self, x: float, y: float) -> float:
        """点到长方体水平投影的最近距离（点在投影内部时为 0）。"""
        lo_x, lo_y, _ = self.lo
        hi_x, hi_y, _ = self.hi
        dx = max(lo_x - x, 0.0, x - hi_x)
        dy = max(lo_y - y, 0.0, y - hi_y)
        return math.hypot(dx, dy)


def slab_intersection(
    origin: Sequence[float],
    direction: Sequence[float],
    lo: Sequence[float],
    hi: Sequence[float],
):
    """射线与轴对齐长方体的相交区间，返回 (t_enter, t_exit)；不相交返回 None。"""
    ox, oy, oz = origin
    dx, dy, dz = direction
    t_enter, t_exit = -math.inf, math.inf
    for o, d, a, b in ((ox, dx, lo[0], hi[0]), (oy, dy, lo[1], hi[1]), (oz, dz, lo[2], hi[2])):
        if abs(d) < EPS:
            if o < a or o > b:
                return None
            continue
        t1 = (a - o) / d
        t2 = (b - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        if t1 > t_enter:
            t_enter = t1
        if t2 < t_exit:
            t_exit = t2
        if t_enter > t_exit:
            return None
    return t_enter, t_exit


@dataclass(frozen=True)
class Room:
    """封闭房间 + 若干放置在地面上的长方体障碍。"""

    half_x: float = HALF_X
    half_y: float = HALF_Y
    height: float = HEIGHT
    obstacles: tuple[Box, ...] = (Box(1.5, 1.5, 0.5, 0.5, 0.5, 0.0, "crate"),)

    # ------------------------------------------------------------- 基础几何
    @property
    def lo(self) -> tuple[float, float, float]:
        return (-self.half_x, -self.half_y, 0.0)

    @property
    def hi(self) -> tuple[float, float, float]:
        return (self.half_x, self.half_y, self.height)

    def contains_2d(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (
            abs(x) <= self.half_x - margin + EPS
            and abs(y) <= self.half_y - margin + EPS
        )

    def clearance(self, x: float, y: float) -> float:
        """点到最近墙面 / 障碍的距离（机器人半径安全裕度就按它算）。"""
        wall = min(self.half_x - abs(x), self.half_y - abs(y))
        for box in self.obstacles:
            wall = min(wall, box.plan_distance(x, y))
        return wall

    def raycast(self, origin: Sequence[float], direction: Sequence[float]) -> float | None:
        """返回最近命中距离（米）；射线起点在房间内部，方向为单位向量。"""
        box = slab_intersection(origin, direction, self.lo, self.hi)
        if box is None:  # 起点必须在房间内，否则模型无意义
            return None
        best = box[1]
        if best <= EPS:
            return None
        for obstacle in self.obstacles:
            hit = slab_intersection(origin, direction, obstacle.lo, obstacle.hi)
            if hit is None:
                continue
            t_enter, t_exit = hit
            if t_enter > EPS:
                best = min(best, t_enter)
            elif t_exit > EPS:
                # 起点在障碍内部（机器人撞进去了）：贴到障碍外表面
                best = min(best, t_exit)
        return best

    def scan(
        self,
        origin: Sequence[float],
        azimuth_count: int = 360,
        elevations_deg: Sequence[float] = DEFAULT_ELEVATIONS_DEG,
        max_range: float = 20.0,
        noise: float = 0.01,
        dropout: float = 0.0,
        rng=None,
    ) -> list[tuple[float, float, float]]:
        """按 (方位角 × 俯仰角) 发射射线，返回传感器坐标系下的命中点。"""
        points: list[tuple[float, float, float]] = []
        sin_el = [math.sin(math.radians(e)) for e in elevations_deg]
        cos_el = [math.cos(math.radians(e)) for e in elevations_deg]
        for index in range(azimuth_count):
            azimuth = 2.0 * math.pi * index / azimuth_count
            ca, sa = math.cos(azimuth), math.sin(azimuth)
            for se, ce in zip(sin_el, cos_el):
                direction = (ca * ce, sa * ce, se)
                distance = self.raycast(origin, direction)
                if distance is None or distance > max_range:
                    continue
                if dropout and rng is not None and rng.random() < dropout:
                    continue
                if noise and rng is not None:
                    distance += rng.gauss(0.0, noise)
                    distance = max(0.05, min(distance, max_range))
                points.append((direction[0] * distance, direction[1] * distance, direction[2] * distance))
        return points

    # ------------------------------------------------------------- 栅格地图
    def map_origin(self, resolution: float = RESOLUTION) -> tuple[float, float]:
        """让最外圈墙心正好落在栅格中心上的 origin（与 map.yaml 的 origin 一致）。"""
        return (-self.half_x - resolution / 2.0, -self.half_y - resolution / 2.0)

    def map_size(self, resolution: float = RESOLUTION) -> tuple[int, int]:
        return (
            int(round(2.0 * self.half_x / resolution)) + 1,
            int(round(2.0 * self.half_y / resolution)) + 1,
        )

    def cell_center(self, col: int, row: int, resolution: float = RESOLUTION) -> tuple[float, float]:
        origin_x, origin_y = self.map_origin(resolution)
        width, height = self.map_size(resolution)
        return (
            origin_x + (col + 0.5) * resolution,
            origin_y + (height - row - 0.5) * resolution,
        )

    def world_to_cell(self, x: float, y: float, resolution: float = RESOLUTION) -> tuple[int, int]:
        origin_x, origin_y = self.map_origin(resolution)
        width, height = self.map_size(resolution)
        col = int(round((x - origin_x) / resolution - 0.5))
        row = int(round((y - origin_y) / resolution - 0.5))
        return col, height - 1 - row

    def _plane_index(self, value: float, resolution: float, axis: str) -> int:
        origin_x, origin_y = self.map_origin(resolution)
        if axis == "x":
            return int(round((value - origin_x) / resolution - 0.5))
        width, height = self.map_size(resolution)
        row_from_bottom = int(round((value - origin_y) / resolution - 0.5))
        return height - 1 - row_from_bottom

    def occupied_cells(self, resolution: float = RESOLUTION) -> set[tuple[int, int]]:
        """静态地图的占用格：四面墙各 1 格（严格对齐真实墙面）+ 障碍投影。"""
        width, height = self.map_size(resolution)
        occupied: set[tuple[int, int]] = set()
        left = self._plane_index(-self.half_x, resolution, "x")
        right = self._plane_index(self.half_x, resolution, "x")
        bottom = self._plane_index(-self.half_y, resolution, "y")
        top = self._plane_index(self.half_y, resolution, "y")
        for row in range(height):
            occupied.add((left, row))
            occupied.add((right, row))
        for col in range(width):
            occupied.add((col, bottom))
            occupied.add((col, top))
        for box in self.obstacles:
            lo_x, lo_y, _ = box.lo
            hi_x, hi_y, _ = box.hi
            col_lo = int(round((lo_x - self.map_origin(resolution)[0]) / resolution - 0.5))
            col_hi = int(round((hi_x - self.map_origin(resolution)[0]) / resolution - 0.5))
            row_lo = int(round((lo_y - self.map_origin(resolution)[1]) / resolution - 0.5))
            row_hi = int(round((hi_y - self.map_origin(resolution)[1]) / resolution - 0.5))
            for col in range(col_lo, col_hi + 1):
                for row_from_bottom in range(row_lo, row_hi + 1):
                    occupied.add((col, height - 1 - row_from_bottom))
        return occupied


def default_room(obstacles: Iterable[Box] | None = None) -> Room:
    """默认测试房间：6m x 8m，墙在 ±3 / ±4，可传自定义障碍。"""
    if obstacles is None:
        return Room()
    return Room(obstacles=tuple(obstacles))


def parse_obstacles(text: str) -> tuple[Box, ...]:
    """解析障碍规格："x,y[,size_x[,size_y[,height]]];x,y;…"（分号分隔多个）。

    地图生成与假机器人必须用同一串规格，否则"扫描出来的障碍"和"地图上的障碍"
    会不一致，AMCL 与代价地图都会跟着出错，故解析逻辑只放在这里一份。
    """
    boxes: list[Box] = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        try:
            values = [float(value) for value in chunk.replace(",", " ").split()]
        except ValueError as exc:
            raise ValueError(f"障碍规格无法解析: {chunk!r}") from exc
        if len(values) < 2:
            raise ValueError(f"障碍至少要有 x,y: {chunk!r}")
        x, y = values[0], values[1]
        size_x = values[2] if len(values) > 2 else 0.5
        size_y = values[3] if len(values) > 3 else size_x
        height = values[4] if len(values) > 4 else 0.5
        boxes.append(Box(x, y, size_x, size_y, height, 0.0, f"obstacle_{x:g}_{y:g}"))
    return tuple(boxes)


def room_from_spec(spec: str | None) -> Room:
    """None/空串 → 默认房间；否则按规格替换障碍集合。"""
    if not spec:
        return Room()
    return Room(obstacles=parse_obstacles(spec))
