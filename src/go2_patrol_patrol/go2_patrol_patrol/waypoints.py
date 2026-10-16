import math
import os
import tempfile
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


def quaternion_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_waypoints(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return [Waypoint(**item) for item in data.get("waypoints", [])]


def waypoint_to_dict(waypoint: Waypoint) -> dict:
    """Waypoint -> 可直接写进 waypoints.yaml 的字典。

    坐标保留 3 位小数（毫米级），与手工维护的 waypoints.yaml 风格一致；
    speech 为空时退回 name，保证巡检时一定有可播报的内容。
    """
    return {
        "name": waypoint.name,
        "x": round(float(waypoint.x), 3),
        "y": round(float(waypoint.y), 3),
        "yaw": round(float(waypoint.yaw), 3),
        "dwell": float(waypoint.dwell),
        "speech": waypoint.speech or waypoint.name,
        "home": bool(waypoint.home),
    }


def save_waypoints(path, waypoints) -> None:
    """把航点列表写回 yaml（保持顺序），原子替换避免写一半损坏文件。

    先写同目录临时文件再 os.replace：即使进程在写入中途被杀，原文件也保持
    完整，不会出现"建图/巡检中途航点文件被写坏"的情况。
    """
    data = {"waypoints": [waypoint_to_dict(w) for w in waypoints]}
    text = yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
    directory = os.path.dirname(os.path.abspath(path)) or "."
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".waypoints_", suffix=".yaml", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
    except BaseException:
        # 失败时清掉临时文件，避免在 maps/config 目录里堆积垃圾。
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def upsert_waypoint(path, waypoint: Waypoint):
    """按 name 覆盖已有航点，不存在则追加；返回更新后的完整列表。

    同名覆盖而不是追加，是为了支持"遥控到新位置后重新记录同一点"这一常见操作。
    """
    waypoints = load_waypoints(path) if os.path.exists(path) else []
    replaced = False
    result = []
    for existing in waypoints:
        if existing.name == waypoint.name:
            result.append(waypoint)
            replaced = True
        else:
            result.append(existing)
    if not replaced:
        result.append(waypoint)
    save_waypoints(path, result)
    return result


def remove_waypoint(path, name: str):
    """删除指定 name 的航点；返回 (剩余列表, 是否真的删掉了)。"""
    waypoints = load_waypoints(path) if os.path.exists(path) else []
    remaining = [w for w in waypoints if w.name != name]
    removed = len(remaining) != len(waypoints)
    if removed:
        save_waypoints(path, remaining)
    return remaining, removed
