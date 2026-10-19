import math
import os
import tempfile
from dataclasses import dataclass

import yaml


# 到达巡检点时的朝向策略（Waypoint.orientation_mode）：
#   auto_next : 到达该点时自动朝向"下一个航点"（普通巡检点的默认行为）。
#               yaw 字段被忽略/保留原值，仅作为记录；实际目标朝向由
#               patrol_manager 依据下一点坐标算出。
#   fixed     : 严格使用 yaw 字段（例如到设备后转向固定方向拍照/检测）。
ORIENTATION_AUTO_NEXT = "auto_next"
ORIENTATION_FIXED = "fixed"
ORIENTATION_MODES = (ORIENTATION_AUTO_NEXT, ORIENTATION_FIXED)


def normalize_orientation_mode(value, yaw=None):
    """把 yaml / GUI 里的 orientation_mode 规整成标准值。

    兼容旧配置：没有 orientation_mode 的一律视为 fixed（老 waypoints.yaml 的
    yaw 一直精确生效，不能因为升级就悄悄改变已有巡检点的行为）。
    yaw 写成字符串 "auto" 也视为 auto_next，方便手写 yaml。
    """
    if isinstance(yaw, str) and yaw.strip().lower() == "auto":
        return ORIENTATION_AUTO_NEXT
    if value is None or (isinstance(value, str) and not value.strip()):
        return ORIENTATION_FIXED
    text = str(value).strip().lower()
    # 容忍常见写法：auto / auto_next / next / fixed / manual
    aliases = {
        "auto": ORIENTATION_AUTO_NEXT,
        "auto_next": ORIENTATION_AUTO_NEXT,
        "next": ORIENTATION_AUTO_NEXT,
        "automatic": ORIENTATION_AUTO_NEXT,
        "fixed": ORIENTATION_FIXED,
        "manual": ORIENTATION_FIXED,
        "yaw": ORIENTATION_FIXED,
    }
    if text in aliases:
        return aliases[text]
    raise ValueError(
        f"orientation_mode 只能是 {ORIENTATION_MODES}（或 yaw: auto），当前为 {value!r}")


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float
    dwell: float = 4.0
    speech: str = ""
    home: bool = False
    orientation_mode: str = ORIENTATION_FIXED

    @property
    def auto_orient(self) -> bool:
        """到达该点时是否自动朝向下一个航点。"""
        return normalize_orientation_mode(self.orientation_mode, self.yaw) \
            == ORIENTATION_AUTO_NEXT


def yaw_towards(from_waypoint, to_waypoint) -> float:
    """从 from 指向 to 的朝向（弧度，map 系，逆时针为正）。"""
    return math.atan2(float(to_waypoint.y) - float(from_waypoint.y),
                      float(to_waypoint.x) - float(from_waypoint.x))


def quaternion_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _waypoint_from_item(item):
    """单条 yaml -> Waypoint（兼容 yaw: auto 与 orientation_mode 两种写法）。"""
    payload = dict(item)
    raw_yaw = payload.get("yaw", 0.0)
    mode = normalize_orientation_mode(payload.get("orientation_mode"), raw_yaw)
    if isinstance(raw_yaw, str) and raw_yaw.strip().lower() == "auto":
        # yaw: auto 不是数字，实测位姿里需要一个占位数值（实际不使用）。
        raw_yaw = 0.0
    payload["yaw"] = float(raw_yaw)
    payload["orientation_mode"] = mode
    # 兼容旧 yaml：speech / home 缺省
    payload.setdefault("speech", payload.get("name", ""))
    payload.setdefault("home", False)
    return Waypoint(**payload)


def load_waypoints(path):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return [_waypoint_from_item(item) for item in data.get("waypoints", [])]


def waypoint_to_dict(waypoint: Waypoint) -> dict:
    """Waypoint -> 可直接写进 waypoints.yaml 的字典。

    坐标保留 3 位小数（毫米级），与手工维护的 waypoints.yaml 风格一致；
    speech 为空时退回 name，保证巡检时一定有可播报的内容。
    """
    mode = normalize_orientation_mode(waypoint.orientation_mode, waypoint.yaw)
    return {
        "name": waypoint.name,
        "x": round(float(waypoint.x), 3),
        "y": round(float(waypoint.y), 3),
        "yaw": round(float(waypoint.yaw), 3),
        "dwell": float(waypoint.dwell),
        "speech": waypoint.speech or waypoint.name,
        "home": bool(waypoint.home),
        # 明确写出朝向策略，文件自解释；fixed 时 yaw 精确生效。
        "orientation_mode": mode,
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
