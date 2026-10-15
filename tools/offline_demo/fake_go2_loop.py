#!/usr/bin/env python3
"""闭环假机器狗：不接真机也能把"导航 → 运动 → 传感器 → 定位"整条链路跑通。

与 fake_go2.py 的区别（fake_go2.py 只发"原地自转"的固定数据，位置恒为原点，
只能看建图效果，Nav2 永远到不了目标）：

  订阅 /api/sport/request   ← 接收 twist_bridge 下发的 MOVE(1008) / STOP_MOVE(1003)
  积分单轨运动模型           → 位姿随指令真实变化
  /lf/sportmodestate 50Hz   → position / velocity / yaw_speed / 四元数 都是真实积分结果
  /utlidar/cloud_deskewed   → 按当前位姿对房间几何做射线投射，10Hz
  /lf/lowstate 1Hz          → 电量可脚本化（--battery-drop-after 触发低电量返航）

房间几何来自 geometry.py，与 make_demo_map.py 生成的静态地图同源，
所以 AMCL 的观测与地图一致，定位不会因为"自己造的数据对不上"而发散。

用法:
  python3 tools/offline_demo/fake_go2_loop.py
  python3 tools/offline_demo/fake_go2_loop.py --obstacles "0.5,0,0.6,0.6,0.6"
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from geometry import DEFAULT_ELEVATIONS_DEG, SENSOR_Z, room_from_spec  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402
from unitree_api.msg import Request  # noqa: E402
from unitree_go.msg import LowState, SportModeState  # noqa: E402

SPORT_API_ID_MOVE = 1008
SPORT_API_ID_STOP_MOVE = 1003
COMMAND_TIMEOUT = 0.5  # 秒；超过这么久没收到 MOVE 就按"失去指令"处理，速度归零
SAFETY_CLEARANCE = 0.15  # 米；下一步会撞墙/撞箱子时拒绝移动（模拟机体被挡住）


def make_cloud(frame_id, stamp, points) -> PointCloud2:
    cloud = PointCloud2()
    cloud.header.stamp = stamp
    cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.point_step = 12
    cloud.row_step = 12 * len(points)
    cloud.is_bigendian = False
    cloud.is_dense = True
    cloud.data = b"".join(struct_pack(p) for p in points)
    return cloud


def struct_pack(point) -> bytes:
    import struct

    return struct.pack("<fff", float(point[0]), float(point[1]), float(point[2]))


class FakeGo2ClosedLoop(Node):
    """单轨运动模型 + 射线投射点云，位姿与传感器数据自洽。"""

    def __init__(self, options):
        super().__init__("fake_go2_closed_loop")
        self.room = room_from_spec(options.obstacles)
        self.azimuth_count = options.azimuth
        self.elevations = options.elevations
        self.noise = options.noise
        self.dropout = options.dropout
        self.rng = random.Random(options.seed)

        # 位姿（map 坐标系 = 假世界的世界坐标系）
        self.x = options.start_x
        self.y = options.start_y
        self.yaw = options.start_yaw
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_command_time = None
        self.blocked = False
        self.distance_travelled = 0.0

        # 电量脚本
        self.soc = float(options.battery_start)
        self.battery_rate = options.battery_rate
        self.battery_drop_after = options.battery_drop_after
        self.battery_drop_to = options.battery_drop_to
        self.started = self.get_clock().now()
        self.battery_dropped = False

        self.pub_state = self.create_publisher(SportModeState, "/lf/sportmodestate", 10)
        self.pub_low = self.create_publisher(LowState, "/lf/lowstate", 10)
        self.pub_cloud = self.create_publisher(PointCloud2, "/utlidar/cloud_deskewed", 10)
        self.static_tf = StaticTransformBroadcaster(self)
        self.create_subscription(Request, "/api/sport/request", self.on_request, 10)
        self.publish_base_footprint_to_base_link()
        self.create_timer(1.0 / 50.0, self.on_state)
        self.create_timer(1.0 / 10.0, self.on_cloud)
        self.create_timer(1.0, self.on_low)
        # 静态 TF 用 transient_local 只在启动时发一次，晚加入的节点可能错过，
        # 所以这里按 1Hz 重发（static_transform_publisher 也是这么做的）。
        self.create_timer(1.0, self.publish_base_footprint_to_base_link)
        self.get_logger().info(
            f"闭环假机器狗已启动：起点=({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f}) "
            f"房间 墙x=±{self.room.half_x:g} 墙y=±{self.room.half_y:g} "
            f"障碍={[b.name for b in self.room.obstacles]} 电量={self.soc:.0f}%"
        )

    # ---------------------------------------------------------------- TF 补链
    def publish_base_footprint_to_base_link(self):
        """补上 base_footprint → base_link。

        仓库里 odom_bridge 只发 odom → base_footprint，perception.launch.py 只发
        base_link → utlidar_lidar，中间这一环在生产 launch 里没有发布者
        （docs 里已列为真机待确认项）。真机上若机器人自身发布了这段 TF，
        再叠加发布者会造成 TF 冲突，所以这里只在离线夹具里补，不动生产 launch。
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_footprint"
        transform.child_frame_id = "base_link"
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.static_tf.sendTransform(transform)

    # ---------------------------------------------------------------- 指令
    def on_request(self, request: Request):
        api_id = int(request.header.identity.api_id)
        if api_id == SPORT_API_ID_MOVE:
            try:
                payload = json.loads(request.parameter or "{}")
            except json.JSONDecodeError:
                self.get_logger().warn(f"MOVE 指令参数不是合法 JSON: {request.parameter!r}")
                return
            self.vx = float(payload.get("x", 0.0))
            self.vy = float(payload.get("y", 0.0))
            self.wz = float(payload.get("z", 0.0))
            self.last_command_time = self.get_clock().now()
            return
        if api_id == SPORT_API_ID_STOP_MOVE:
            self.zero_velocity()
            self.last_command_time = self.get_clock().now()
            return
        self.get_logger().debug(f"忽略 api_id={api_id}（离线演示只模拟 MOVE/STOP_MOVE）")

    def zero_velocity(self):
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

    # ---------------------------------------------------------------- 运动积分
    def integrate(self, dt: float):
        now = self.get_clock().now()
        if self.last_command_time is None or (
            now - self.last_command_time
        ).nanoseconds * 1e-9 > COMMAND_TIMEOUT:
            self.zero_velocity()
            return

        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        dx = (self.vx * cos_yaw - self.vy * sin_yaw) * dt
        dy = (self.vx * sin_yaw + self.vy * cos_yaw) * dt
        next_x, next_y = self.x + dx, self.y + dy

        if self.room.clearance(next_x, next_y) < SAFETY_CLEARANCE:
            if not self.blocked:
                self.get_logger().warn(
                    f"机体被挡住：({next_x:.2f}, {next_y:.2f}) 距墙/障碍 "
                    f"{self.room.clearance(next_x, next_y):.2f}m，速度归零"
                )
            self.blocked = True
            self.zero_velocity()
            return
        self.blocked = False

        self.x, self.y = next_x, next_y
        self.yaw = math.atan2(math.sin(self.yaw + self.wz * dt), math.cos(self.yaw + self.wz * dt))
        self.distance_travelled += math.hypot(dx, dy)

    # ---------------------------------------------------------------- 发布
    def on_state(self):
        self.integrate(1.0 / 50.0)
        message = SportModeState()
        stamp = self.get_clock().now()
        message.stamp.sec = int(stamp.nanoseconds // 1_000_000_000)
        message.stamp.nanosec = int(stamp.nanoseconds % 1_000_000_000)
        message.position = [float(self.x), float(self.y), 0.0]
        message.velocity = [float(self.vx), float(self.vy), 0.0]
        message.yaw_speed = float(self.wz)
        message.imu_state.quaternion = [
            0.0,
            0.0,
            math.sin(self.yaw * 0.5),
            math.cos(self.yaw * 0.5),
        ]
        message.imu_state.rpy = [0.0, 0.0, float(self.yaw)]
        message.mode = 0
        message.gait_type = 0
        self.pub_state.publish(message)

    def on_low(self):
        elapsed = (self.get_clock().now() - self.started).nanoseconds * 1e-9
        if self.battery_rate:
            self.soc = max(0.0, self.soc - self.battery_rate * elapsed / 60.0)
        if (
            self.battery_drop_after is not None
            and not self.battery_dropped
            and elapsed >= self.battery_drop_after
        ):
            self.battery_dropped = True
            self.soc = float(self.battery_drop_to)
            self.get_logger().warn(f"脚本触发低电量：SOC 跳到 {self.soc:.0f}%（验证返航逻辑）")

        message = LowState()
        message.bms_state.soc = int(round(self.soc))
        self.pub_low.publish(message)

    def on_cloud(self):
        sensor_origin = (self.x, self.y, SENSOR_Z)
        hits = self.room.scan(
            sensor_origin,
            azimuth_count=self.azimuth_count,
            elevations_deg=self.elevations,
            noise=self.noise,
            dropout=self.dropout,
            rng=self.rng,
        )
        cos_yaw, sin_yaw = math.cos(-self.yaw), math.sin(-self.yaw)
        local = []
        for hx, hy, hz in hits:
            wx = sensor_origin[0] + hx
            wy = sensor_origin[1] + hy
            wz = sensor_origin[2] + hz
            local.append(
                (
                    cos_yaw * (wx - self.x) - sin_yaw * (wy - self.y),
                    sin_yaw * (wx - self.x) + cos_yaw * (wy - self.y),
                    wz - SENSOR_Z,
                )
            )
        try:
            self.pub_cloud.publish(
                make_cloud("utlidar_lidar", self.get_clock().now().to_msg(), local)
            )
        except Exception:  # 收尾阶段 context 已失效，这不是错误
            pass


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="闭环假机器狗（离线跑通巡检）")
    parser.add_argument("--obstacles", default=None, help='障碍规格 "x,y[,sx[,sy[,h]]];…"')
    parser.add_argument("--azimuth", type=int, default=360, help="水平射线数（默认 360）")
    parser.add_argument(
        "--elevations",
        type=float,
        nargs="+",
        default=list(DEFAULT_ELEVATIONS_DEG),
        help="俯仰角（度），默认 0 5 10 15 20 25",
    )
    parser.add_argument("--noise", type=float, default=0.01, help="测距高斯噪声标准差（米）")
    parser.add_argument("--dropout", type=float, default=0.01, help="单点丢失概率")
    parser.add_argument("--seed", type=int, default=7, help="随机种子（保证可复现）")
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--start-yaw", type=float, default=0.0)
    parser.add_argument("--battery-start", type=float, default=87.0, help="初始电量（%）")
    parser.add_argument("--battery-rate", type=float, default=0.0, help="耗电速率（%/分钟）")
    parser.add_argument(
        "--battery-drop-after",
        type=float,
        default=None,
        help="运行 N 秒后把电量跳到 --battery-drop-to，用于验证低电量返航",
    )
    parser.add_argument("--battery-drop-to", type=float, default=15.0)
    return parser.parse_args(argv)


def main(argv=None):
    options = parse_args(argv)
    rclpy.init()
    node = FakeGo2ClosedLoop(options)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.get_logger().info(
                f"结束：位姿=({node.x:.2f}, {node.y:.2f}, {node.yaw:.2f}) "
                f"累计里程={node.distance_travelled:.2f}m"
            )
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
