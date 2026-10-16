"""`map` 坐标系位姿查询 —— 复用 go2_patrol_patrol.map_pose 的同一实现。

为什么不在 GUI 里另写一份：记录航点用错坐标系（把 /odom 当地图坐标）是
本工具最需要避免的错误，实现必须只有一份。真正的逻辑在
`src/go2_patrol_patrol/go2_patrol_patrol/map_pose.py`，CLI 的 record_waypoint
与 GUI 共用它，避免两边理解不一致。

在没有 source ROS 的纯测试环境里，回退到源码树导入（与 waypoint_store 同法）。
"""
import os
import sys


def _module():
    try:
        from go2_patrol_patrol import map_pose as module
        return module
    except ImportError:
        # sys.path 里要放包的**父目录**（src/go2_patrol_patrol），不是包目录本身。
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..",
                                            "src", "go2_patrol_patrol"))
        if root not in sys.path:
            sys.path.insert(0, root)
        from go2_patrol_patrol import map_pose as module
        return module


_impl = _module()

DEFAULT_MAP_FRAME = _impl.DEFAULT_MAP_FRAME
DEFAULT_BASE_FRAME = _impl.DEFAULT_BASE_FRAME
yaw_from_quaternion = _impl.yaw_from_quaternion
pose_from_transform = _impl.pose_from_transform
wait_for_map_pose = _impl.wait_for_map_pose
lookup_map_pose = _impl.lookup_map_pose
