"""GUI 的航点读写封装：直接复用 go2_patrol_patrol.waypoints 的既有逻辑。

不另写一套 yaml 解析/序列化，避免 GUI 与巡检节点对文件格式理解不一致。
唯一补充：写入前把 name 规整化（去空白），并做最基本的合法性校验，
让错误在界面上就暴露，而不是等到巡检时才炸。
"""
import os
import sys


def _waypoints_module():
    """导入共享的 waypoints 模块。

    优先用 ROS 安装好的包；在没有 source ROS 的纯测试环境里，回退到源码树。
    """
    try:
        from go2_patrol_patrol import waypoints as module
        return module
    except ImportError:
        # 注意：要加入 sys.path 的是 **包的父目录**（src/go2_patrol_patrol），
        # 不是包目录本身，否则 `import go2_patrol_patrol` 仍然失败。
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..",
                                            "src", "go2_patrol_patrol"))
        if root not in sys.path:
            sys.path.insert(0, root)
        from go2_patrol_patrol import waypoints as module
        return module


def normalize_name(name):
    """航点名规整：去首尾空白；内部空白转下划线，便于做 speech 与文件名。"""
    cleaned = "_".join(str(name).strip().split())
    return cleaned


def validate_name(name):
    """返回 (ok, 错误说明)。name 必须非空、且不能是纯符号。"""
    normalized = normalize_name(name)
    if not normalized:
        return False, "巡检点名称不能为空"
    if not any(ch.isalnum() for ch in normalized):
        return False, "巡检点名称至少包含一个字母或数字"
    return True, ""


class WaypointStore:
    """对 waypoints.yaml 的增删查，全部走共享模块的原子写。"""

    def __init__(self, path):
        self.path = path
        self._module = _waypoints_module()

    def list(self):
        if not os.path.exists(self.path):
            return []
        return self._module.load_waypoints(self.path)

    def names(self):
        return [w.name for w in self.list()]

    def upsert(self, name, x, y, yaw, dwell=4.0, speech=None, home=False):
        ok, error = validate_name(name)
        if not ok:
            raise ValueError(error)
        normalized = normalize_name(name)
        waypoint = self._module.Waypoint(
            name=normalized,
            x=float(x),
            y=float(y),
            yaw=float(yaw),
            dwell=float(dwell),
            speech=speech or normalized,
            home=bool(home),
        )
        return self._module.upsert_waypoint(self.path, waypoint)

    def remove(self, name):
        return self._module.remove_waypoint(self.path, name)
