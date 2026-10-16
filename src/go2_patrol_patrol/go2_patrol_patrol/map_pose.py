"""在 `map` 坐标系下取机器人当前位姿（航点记录的正确姿势）。

为什么不能直接读 `/odom`：

  odom_bridge 发布的 `/odom` 是 **odom 系** 的里程计，`odom -> base_footprint`
  由它广播。odom 系只是"上电时刻的机器人位置"，会随漂移与地图错位；真正的
  地图坐标在 `map` 系里。`map -> odom` 这一段由定位模块提供：

    * REAL  模式：AMCL 发布 `map -> odom`（会不断修正）
    * OFFLINE 模式：静态 `map -> odom`
    * 建图时：slam_toolbox 发布 `map -> odom`

  所以"记录航点"必须查 `map -> base_footprint` 的 TF，而不是把 `/odom` 的
  x/y 当成地图坐标 —— 否则 odom 起点不在原点时，记下来的航点会整体偏移。

本模块只依赖标准库；ROS 相关 import 全部放在函数内部，方便在无 rclpy 的
环境里做单元测试（用假的 buffer 对象即可）。
"""
import math
import time

DEFAULT_MAP_FRAME = "map"
DEFAULT_BASE_FRAME = "base_footprint"


def yaw_from_quaternion(q) -> float:
    """四元数 -> yaw（绕 z 轴，弧度）。兼容 rclpy 返回的 numpy.float32。"""
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_from_transform(transform_stamped):
    """TransformStamped -> (x, y, yaw)。"""
    translation = transform_stamped.transform.translation
    rotation = transform_stamped.transform.rotation
    return float(translation.x), float(translation.y), yaw_from_quaternion(rotation)


def wait_for_map_pose(buffer, map_frame=DEFAULT_MAP_FRAME,
                      base_frame=DEFAULT_BASE_FRAME, timeout_sec=5.0,
                      time_point_factory=None, poll_interval=0.1):
    """等 TF 树里出现 `map_frame -> base_frame`，返回 (x, y, yaw) 或 None。

    buffer 只要有 `lookup_transform(target, source, time)` 即可（tf2_ros.Buffer
    或测试用的假对象）。time_point_factory() 提供"最新时刻"，默认用
    rclpy.time.Time()（= 取最新可用变换）。
    """
    if time_point_factory is None:
        from rclpy.time import Time

        def time_point_factory():
            return Time()

    deadline = time.monotonic() + float(timeout_sec)
    while True:
        try:
            transform = buffer.lookup_transform(
                map_frame, base_frame, time_point_factory())
            return pose_from_transform(transform)
        except Exception:  # tf2 在"变换还没到"时会抛各种 LookupException
            pass
        if time.monotonic() >= deadline:
            return None
        time.sleep(poll_interval)


def lookup_map_pose(node, map_frame=DEFAULT_MAP_FRAME,
                    base_frame=DEFAULT_BASE_FRAME, timeout_sec=5.0):
    """在给定 rclpy 节点上查 `map -> base_footprint`，返回 (x, y, yaw) 或 None。

    自己建 Buffer + TransformListener 并 spin，调用方不需要关心 TF 细节。
    """
    import rclpy
    from tf2_ros import Buffer, TransformListener

    buffer = Buffer()
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + float(timeout_sec)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                transform = buffer.lookup_transform(map_frame, base_frame,
                                                    rclpy.time.Time())
                return pose_from_transform(transform)
            except Exception:
                pass
            if time.monotonic() >= deadline:
                return None
    finally:
        del listener
