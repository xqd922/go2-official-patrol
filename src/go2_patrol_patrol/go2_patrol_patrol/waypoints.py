import math
from dataclasses import dataclass

import yaml


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float
    dwell: float = 4.0
    speech: str = ""
    home: bool = False
    # 到点朝向策略；留空 = 用节点的 yaw_mode 参数：
    #   fixed  用本文件的 yaw（默认，适合"要正对设备拍照"的点）
    #   travel 朝进点方向（到点不用转身，路口推荐）
    #   next   朝下一段方向（起步不用转身）
    yaw_mode: str = ""


def quaternion_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_waypoints(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return [Waypoint(**item) for item in data.get("waypoints", [])]
