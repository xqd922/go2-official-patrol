#!/usr/bin/env python3
"""离线闭环演示的验收器：断言"巡检真的走完了一圈"，而不是"看起来起了"。

判定依据是 AMCL 发布的 /amcl_pose（map 坐标系）逐个到达预期点位，
预期顺序来自 patrol_sequence.PatrolSequence —— 与线上节点同一份调度规则，
所以验收器不会自己实现一套"看起来对"的顺序。

判定项：
  1. 预期目标序列全部到达（点位顺序 → 收工回 home）
  2. 每个到达点的水平误差 ≤ --tolerance
  3. 标了 fixed 的点位（含站点级 yaw_mode=fixed）到点朝向误差 ≤ --yaw-tolerance
  4. 累计行程 ≥ --min-distance，排除"原地不动却依次判定成功"
  5. 每个点位的播报 id 都出现过（不再只断言"有播报"）

退出码：0=通过，1=不通过，2=超时/环境不具备。

用法（先 source 工作空间环境）:
  python3 tools/offline_demo/demo_verify.py --waypoints sites/demo/waypoints.yaml
  python3 tools/offline_demo/demo_verify.py --waypoints sites/demo/waypoints.yaml --loops 0
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def import_patrol_modules():
    """优先用安装好的包；没装也能直接跑源码树（开发机便利）。"""
    try:
        from go2_patrol_patrol.patrol_sequence import PatrolSequence
        from go2_patrol_patrol.waypoints import load_waypoints

        return PatrolSequence, load_waypoints
    except ImportError:
        source_root = os.path.abspath(
            os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"
            )
        )
        if source_root not in sys.path:
            sys.path.insert(0, source_root)
        from go2_patrol_patrol.patrol_sequence import PatrolSequence
        from go2_patrol_patrol.waypoints import load_waypoints

        return PatrolSequence, load_waypoints


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def angle_error(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class DemoVerifier(Node):
    def __init__(self, options, sequence):
        super().__init__("demo_verifier")
        self.sequence = sequence
        self.expected = self._simulate_expected()
        self.tolerance = options.tolerance
        self.min_distance = options.min_distance
        self.report_path = options.report
        self.deadline = time.monotonic() + options.timeout
        # 站点级到点朝向策略：点位自己写了 yaw_mode 就用点位自己的
        self.yaw_mode = str(options.yaw_mode).strip().lower() or "fixed"
        self.yaw_tolerance = options.yaw_tolerance
        self.speech_grace = options.speech_grace
        self.expected_speeches = [w.speech or w.name for w in self.expected]
        self.complete_at = None

        self.pose = None
        self.path_length = 0.0
        self.speeches: list[str] = []
        self.arrivals: list[dict] = []
        # fixed 点位"进位置容差但还没转正"的起始时刻（见 tick）
        self.yaw_settle_since = None
        self.yaw_settle = float(options.yaw_settle)
        self.started = time.monotonic()
        self.finished = False
        self.result_code = 2

        self.create_subscription(
            PoseWithCovarianceStamped, options.pose_topic, self.on_pose, qos_profile_sensor_data
        )
        self.create_subscription(String, "/robot/say", self.on_say, 10)
        self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f"验收器就绪：预期 {len(self.expected)} 个目标（{' → '.join(w.name for w in self.expected)}），"
            f"容差 {self.tolerance:.2f}m，到点朝向 {self.yaw_mode}（fixed 点位判 ≤{self.yaw_tolerance:.2f}rad），"
            f"超时 {options.timeout:.0f}s，位姿话题 {options.pose_topic}"
        )

    def _simulate_expected(self):
        """用状态机预演一遍，得到预期访问顺序（不依赖播报）。"""
        sequence = self.sequence.copy()
        expected = []
        for _ in range(1000):
            waypoint = sequence.next_goal()
            if waypoint is None:
                break
            expected.append(waypoint)
            sequence.on_arrival(waypoint)
        return expected

    # ------------------------------------------------------------- 回调
    def on_pose(self, message: PoseWithCovarianceStamped):
        pose = message.pose.pose
        if self.pose is not None:
            self.path_length += math.hypot(
                pose.position.x - self.pose[0], pose.position.y - self.pose[1]
            )
        self.pose = (pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))

    def on_say(self, message: String):
        self.speeches.append(message.data.strip())

    # ------------------------------------------------------------- 主判定
    def tick(self):
        if self.finished:
            return
        now = time.monotonic()
        if self.pose is None:
            if now > self.deadline:
                self.finish(False, "超时：始终没收到位姿（AMCL 未启动或未定位）")
            return

        while len(self.arrivals) < len(self.expected):
            waypoint = self.expected[len(self.arrivals)]
            distance = math.hypot(self.pose[0] - waypoint.x, self.pose[1] - waypoint.y)
            if distance > self.tolerance:
                break
            yaw_error = angle_error(self.pose[2], waypoint.yaw)
            mode = (waypoint.yaw_mode or self.yaw_mode).strip().lower()
            # fixed 点位：Nav2 报成功前还要在点上原地把姿态转到 waypoint.yaw，
            # 而位姿进容差（这里 0.40m）比 Nav2 的 xy 容差（0.30m）还早。
            # 若在第一次进圈时就冻结 yaw，记下的是**行进朝向**：返航回 home 时
            # 朝向差 ~1.9rad，会被判成朝向超差，而机器狗随后转正、Nav2 也报成功。
            # 所以 fixed 点要等到朝向也进容差才定案；超时则照实记录（真超差仍会失败）。
            if mode == "fixed" and yaw_error > self.yaw_tolerance:
                if self.yaw_settle_since is None:
                    self.yaw_settle_since = now
                if now - self.yaw_settle_since < self.yaw_settle:
                    # 这里只能 break 出 while、**不能 return**：函数末尾的超时判定
                    # 就在循环之后，直接 return 会让它永远轮不到。--yaw-settle 可配且
                    # 无上限，一旦配得比 --timeout 还大，一个转不正的 fixed 点就会让
                    # 验收器一直挂到 settle 耗尽，与"退出码 2=超时"的对外契约不符。
                    break
                self.get_logger().warn(
                    f"{waypoint.name} 已进位置容差但朝向仍差 {yaw_error:.2f}rad"
                    f"（等待 {self.yaw_settle:.1f}s 仍未转正），按实测记录"
                )
            self.yaw_settle_since = None
            self.arrivals.append(
                {
                    "name": waypoint.name,
                    "elapsed_s": round(now - self.started, 2),
                    "distance_m": round(distance, 3),
                    "pose": [round(self.pose[0], 3), round(self.pose[1], 3)],
                    "target": [waypoint.x, waypoint.y],
                    "yaw_mode": mode,
                    "yaw_error_rad": round(yaw_error, 3),
                    # 朝向只在 fixed 点位上是有约束的：travel/next 的朝向取决于来路，
                    # 拿点位 yaw 去判会误判——这正是"朝向没被验收覆盖"的原因。
                    "yaw_ok": mode != "fixed" or yaw_error <= self.yaw_tolerance,
                }
            )
            self.get_logger().info(
                f"[{len(self.arrivals)}/{len(self.expected)}] 到达 {waypoint.name} "
                f"({self.pose[0]:+.2f}, {self.pose[1]:+.2f}) 误差 {distance:.2f}m "
                f"朝向 {mode} 偏差 {yaw_error:.2f}rad 用时 {now - self.started:.1f}s"
            )

        if len(self.arrivals) >= len(self.expected):
            # 到点播报是在 Nav2 报成功那一刻发的，可能比位姿进容差晚一拍。
            # 这里留一点收尾时间，否则"每个点位都播报过"会被误判成漏播。
            if self.complete_at is None:
                self.complete_at = now
                self.get_logger().info(
                    f"目标点已全部到达，等待 {self.speech_grace:.1f}s 收集收尾播报"
                )
                return
            if now >= self.complete_at + self.speech_grace:
                self.finish(True, "全部目标点到达")
            return
        if now > self.deadline:
            missing = ", ".join(w.name for w in self.expected[len(self.arrivals):])
            self.finish(False, f"超时：未到达 {missing}")

    def finish(self, passed: bool, reason: str):
        self.finished = True
        missing_speech = [s for s in self.expected_speeches if s and s not in self.speeches]
        yaw_violations = [a["name"] for a in self.arrivals if not a["yaw_ok"]]
        checks = {
            "目标序列全部到达": len(self.arrivals) >= len(self.expected),
            f"累计行程 ≥ {self.min_distance:g}m": self.path_length >= self.min_distance,
            "每个点位的播报都出现过": not missing_speech,
        }
        if any(a["yaw_mode"] == "fixed" for a in self.arrivals):
            checks[f"fixed 点位到点朝向误差 ≤ {self.yaw_tolerance:g}rad"] = not yaw_violations
        ok = bool(passed) and all(checks.values())
        self.result_code = 0 if ok else (2 if not passed else 1)
        report = {
            "passed": ok,
            "reason": reason,
            "checks": checks,
            "expected": [w.name for w in self.expected],
            "arrivals": self.arrivals,
            "speeches": self.speeches,
            "expected_speeches": self.expected_speeches,
            "missing_speeches": missing_speech,
            "path_length_m": round(self.path_length, 3),
            "elapsed_s": round(time.monotonic() - self.started, 2),
            "tolerance_m": self.tolerance,
            "yaw_mode": self.yaw_mode,
            "yaw_tolerance_rad": self.yaw_tolerance,
        }
        print("\n================ 离线闭环演示结果 ================")
        print(f"预期点位 : {' -> '.join(report['expected'])}")
        print(f"实际到达 : {' -> '.join(a['name'] for a in self.arrivals) or '(无)'}")
        print(f"播报记录 : {', '.join(self.speeches) or '(无)'}")
        if missing_speech:
            print(f"漏播点位 : {', '.join(missing_speech)}")
        if yaw_violations:
            print(f"朝向超差 : {', '.join(yaw_violations)}")
        print(f"累计行程 : {self.path_length:.2f} m")
        print(f"总耗时   : {report['elapsed_s']:.1f} s")
        for name, value in checks.items():
            print(f"  [{' OK ' if value else 'FAIL'}] {name}")
        print(f"结论     : {'通过' if ok else '不通过'}（{reason}）")
        print("==================================================")
        if self.report_path:
            try:
                with open(self.report_path, "w", encoding="utf-8") as handle:
                    json.dump(report, handle, ensure_ascii=False, indent=2)
                print(f"报告已写入 {self.report_path}")
            except OSError as exc:
                self.get_logger().warn(f"报告写入失败: {exc}")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="离线闭环巡检演示验收器")
    parser.add_argument("--waypoints", required=True, help="waypoints.yaml 路径")
    parser.add_argument(
        "--loops", type=int, default=1, help="巡检圈数；0 或负数（无限循环）按 1 圈验收"
    )
    parser.add_argument("--return-home", dest="return_home", action="store_true", default=True)
    parser.add_argument("--no-return-home", dest="return_home", action="store_false")
    parser.add_argument("--tolerance", type=float, default=0.40, help="到点判定容差（米）")
    parser.add_argument("--min-distance", type=float, default=5.0, help="累计行程下限（米）")
    parser.add_argument("--timeout", type=float, default=420.0, help="总超时（秒）")
    parser.add_argument(
        "--yaw-mode",
        default="fixed",
        help="站点级到点朝向策略（fixed/travel/next），与 site.yaml 的 patrol.yaw_mode 一致；"
        "只有 fixed 点位会被判朝向误差",
    )
    parser.add_argument("--yaw-tolerance", type=float, default=0.35, help="fixed 点位朝向容差（弧度）")
    parser.add_argument(
        "--yaw-settle",
        type=float,
        default=3.0,
        help="fixed 点位进位置容差后等多久转正（秒）；超时按实测记录，真超差照样失败",
    )
    parser.add_argument(
        "--speech-grace",
        type=float,
        default=3.0,
        help="全部到点后额外等待几秒再核对播报（等收尾那句发出来）",
    )
    parser.add_argument("--pose-topic", default="/amcl_pose", help="map 坐标系位姿话题")
    parser.add_argument("--report", default="/tmp/go2_demo_report.json", help="结果 JSON 路径")
    return parser.parse_args(argv)


def main(argv=None):
    options = parse_args(argv)
    PatrolSequence, load_waypoints = import_patrol_modules()

    waypoints = load_waypoints(options.waypoints)
    if not waypoints:
        print(f"[demo_verify] {options.waypoints} 里没有点位", file=sys.stderr)
        return 2
    sequence = PatrolSequence(
        waypoints,
        loops=options.loops if options.loops > 0 else 1,
        return_home=options.return_home,
    )

    rclpy.init()
    node = DemoVerifier(options, sequence)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.finish(False, "人工中断")
    finally:
        code = node.result_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
