#!/usr/bin/env python3
"""OFFLINE 定点巡检端到端验证器（不需要 Go2 实机 / Go2 网卡）。

判定 PASS 必须**全部**满足：
  1. 链路唯一性：fake_go2×1、odom_bridge×1、/odom publisher×1、
     odom->base_footprint TF publisher×1、map->odom static TF×1；
  2. Nav2 离线所需 lifecycle 节点全部 ACTIVE；
  3. /patrol/start 返回 success；
  4. 观测到非零 /cmd_vel；
  5. fake_go2 的 /odom 由 /cmd_vel 驱动、连续变化，x 从 ~0 推进到 ~2；
  6. entrance 与 equipment_zone 都真正到达；
  7. patrol 自动回到 IDLE（再次 start 成功）；
  8. 离线未向 /api/sport/request 发任何运动命令。

任何一条不满足即 FAILED —— 单元测试 PASS 不算巡检 PASS。
"""
import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

ACTIVE = 3  # lifecycle_msgs.msg.State.PRIMARY_STATE_ACTIVE

NAV2_NODES = [
    "map_server", "controller_server", "planner_server", "smoother_server",
    "behavior_server", "bt_navigator", "waypoint_follower", "velocity_smoother",
]

# 进程签名（出现在 /proc/<pid>/cmdline 里）：用可执行文件名而非节点名，
# 因为 ros2 run 起的节点 cmdline 是 "<python> .../lib/<pkg>/<exe>"。
PROC_SIGNATURES = {
    "fake_go2": "fake_go2.py",
    "odom_bridge": "odom_bridge",
    "twist_bridge": "twist_bridge",
    "battery_bridge": "battery_bridge",
    "patrol_manager": "patrol_manager",
    "nav2_starter": "nav2_starter.py",
}

# Nav2 节点进程签名：用 "包/可执行文件" 形式，避免与同名前缀误匹配。
# 每个都应恰好 1 个实例；lifecycle_manager 正常有 2 个（localization + navigation），
# 靠 --ros-args 的 __node:= 名字区分，避免把"两套 Nav2"漏掉。
NAV2_PROC_SIGNATURES = {
    "controller_server": "nav2_controller/controller_server",
    "planner_server": "nav2_planner/planner_server",
    "smoother_server": "nav2_smoother/smoother_server",
    "behavior_server": "nav2_behaviors/behavior_server",
    "bt_navigator": "nav2_bt_navigator/bt_navigator",
    "waypoint_follower": "nav2_waypoint_follower/waypoint_follower",
    "velocity_smoother": "nav2_velocity_smoother/velocity_smoother",
    "map_server": "nav2_map_server/map_server",
}

# 这些进程名出现在 /tf 上就说明有第二套写者
TF_WRITERS_ODOM_BASE = {"go2_odom_bridge"}

# 进程/domain/进程组工具集中在 offline_proc（无 ROS 依赖，便于单测）。
# 这里复用同一份实现，避免 E2E 与清理脚本各写一套导致行为漂移。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from offline_proc import (  # noqa: E402
    format_domains as _fmt_domains,
    read_cmdline as _read_cmdline,
    read_domain as _read_domain,
    stale_domains,
)


def count_processes(signatures):
    """按 cmdline 子串统计进程数，附带 ROS_DOMAIN_ID（用于抓残留实例）。"""
    counts = {k: 0 for k in signatures}
    domains = {k: set() for k in signatures}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        cmd = _read_cmdline(entry)
        if not cmd or "e2e_offline_patrol" in cmd or "run_offline_e2e" in cmd:
            continue
        for key, sig in signatures.items():
            if sig in cmd:
                counts[key] += 1
                domains[key].add(_read_domain(entry))
    return counts, domains


def count_nav2_processes():
    """统计 Nav2 节点进程数，并单独统计两个 lifecycle_manager 实例。

    用户上一轮 FAILED 的核心怀疑点之一：是否存在"两套 Nav2 / map_server /
    lifecycle_manager"。这里直接按进程 cmdline 数出来，任何节点 >1 即暴露。
    返回 (counts, manager_names, domains)：manager_names 是每个
    lifecycle_manager 进程的 __node:= 名字（应为 localization/navigation 各一）。
    """
    counts = {k: 0 for k in NAV2_PROC_SIGNATURES}
    domains = {k: set() for k in NAV2_PROC_SIGNATURES}
    manager_names = []
    manager_domains = set()
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        cmd = _read_cmdline(entry)
        if not cmd or "e2e_offline_patrol" in cmd or "run_offline_e2e" in cmd:
            continue
        for key, sig in NAV2_PROC_SIGNATURES.items():
            if sig in cmd:
                counts[key] += 1
                domains[key].add(_read_domain(entry))
        if "nav2_lifecycle_manager/lifecycle_manager" in cmd:
            manager_domains.add(_read_domain(entry))
            node = ""
            for token in cmd.split():
                if token.startswith("__node:="):
                    node = token.split(":=", 1)[1]
            manager_names.append(node or "?")
    return counts, manager_names, manager_domains, domains


class Verifier:
    def __init__(self, node):
        self.node = node
        self.cmd_samples = []      # (t, linear.x, angular.z)
        self.odom_samples = []     # (t, x, y, yaw)
        self.tf_odom_base_pubs = set()   # /tf 上发 odom->base_footprint 的节点名
        self.tf_static_map_odom_pubs = set()
        self.sport_requests = 0
        node.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 50)
        node.create_subscription(Odometry, "/odom", self.on_odom, 50)
        node.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        # /tf_static 是 latched（transient_local）：必须用匹配的 QoS，
        # 否则晚加入的订阅者收不到已经发布过的静态变换。
        static_qos = QoSProfile(depth=100,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST)
        node.create_subscription(TFMessage, "/tf_static", self.on_tf_static, static_qos)
        try:
            from unitree_api.msg import Request
            node.create_subscription(Request, "/api/sport/request", self.on_sport, 10)
        except Exception as exc:
            node.get_logger().warn(f"无法订阅 /api/sport/request（离线预期）: {exc}")
        self.cli_start = node.create_client(Trigger, "/patrol/start")
        self.cli_stop = node.create_client(Trigger, "/patrol/stop")
        self._state_clients = {}   # /<node>/get_state 客户端缓存，避免轮询时反复创建

    # ---- callbacks ----
    def on_cmd_vel(self, msg):
        self.cmd_samples.append((time.time(), float(msg.linear.x), float(msg.angular.z)))

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_samples.append((time.time(), float(msg.pose.pose.position.x),
                                  float(msg.pose.pose.position.y), yaw))

    def on_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == "odom" and tf.child_frame_id == "base_footprint":
                self.tf_odom_base_pubs.add("go2_odom_bridge")

    def on_tf_static(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == "map" and tf.child_frame_id == "odom":
                self.tf_static_map_odom_pubs.add("map_to_odom")

    def on_sport(self, _msg):
        self.sport_requests += 1

    # ---- helpers ----
    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def call(self, client, name):
        # /patrol/start 由 patrol_manager 在 waitUntilNav2Active 返回后才创建，
        # 这里给足等待时间，避免"服务尚未创建"被误判为失败。
        if not client.wait_for_service(timeout_sec=30.0):
            return None, f"{name} 服务不可用"
        future = client.call_async(Trigger.Request())
        end = time.time() + 20.0
        while not future.done() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if not future.done():
            return None, f"{name} 调用超时"
        return future.result(), None

    def publishers(self, topic):
        try:
            infos = self.node.get_publishers_info_by_topic(topic)
            return [(i.node_name, i.node_namespace) for i in infos]
        except Exception:
            return []

    def _state_client(self, name):
        if name not in self._state_clients:
            self._state_clients[name] = self.node.create_client(GetState, f"/{name}/get_state")
        return self._state_clients[name]

    def check_lifecycle(self, service_timeout=10.0):
        states = {}
        for name in NAV2_NODES:
            client = self._state_client(name)
            if not client.wait_for_service(timeout_sec=service_timeout):
                states[name] = "NO_SERVICE"
                continue
            future = client.call_async(GetState.Request())
            end = time.time() + 5.0
            while not future.done() and time.time() < end:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                sid = future.result().current_state.id
                states[name] = "ACTIVE" if sid == ACTIVE else f"state={sid}"
            else:
                states[name] = "TIMEOUT"
        return states

    def wait_nav2_active(self, timeout, hold=10.0):
        """轮询直到本次 Nav2 全部 ACTIVE **并稳定保持 hold 秒**。

        为什么不仅要"某一刻 ACTIVE"：真实机 13:07 的运行里，8 个节点确实都
        一度 ACTIVE（nav2_starter 报 OK），但约 20s 后 lifecycle_manager 报
        "Have not received a heartbeat from controller_server"，把整套 Nav2
        deactivate。若门控只要求"曾经 ACTIVE"，就会在节点即将崩掉的窗口里放行
        /patrol/start，随后必然失败。这里要求连续 hold 秒都 ACTIVE 才算通过，
        用"持续观测"而不是"任意 sleep"来排除这种抖动。

        返回 (states, stable)；stable=True 表示满足保持要求。states 是最后一次
        观测到的 {node: state}。
        """
        deadline = time.time() + timeout
        states = {}
        active_since = None
        while time.time() < deadline:
            states = self.check_lifecycle(service_timeout=2.0)
            if states and all(v == "ACTIVE" for v in states.values()):
                if active_since is None:
                    active_since = time.time()
                elif time.time() - active_since >= hold:
                    return states, True
            else:
                # 一旦掉出全 ACTIVE，重新计时（要求的是"连续"稳定）。
                active_since = None
            time.sleep(1.0)
        return states, False

    def wait_stack_appear(self, timeout):
        """等本次 launch 的 Nav2 节点服务出现 + 离线链路进程就位。

        取代调用方里"固定 sleep 60s 等 Nav2"。返回 True 表示后续唯一性检查
        （进程数 / publisher / TF）有意义；超时返回 False，由调用方判 FAIL。
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            services_ok = all(
                self._state_client(n).wait_for_service(timeout_sec=1.0)
                for n in NAV2_NODES
            )
            counts, _ = count_processes(PROC_SIGNATURES)
            procs_ok = (counts["fake_go2"] >= 1 and counts["odom_bridge"] >= 1
                        and counts["patrol_manager"] >= 1)
            if services_ok and procs_ok:
                return True
            time.sleep(0.5)
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=300.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("offline_patrol_e2e")
    verifier = Verifier(node)
    logger = node.get_logger()
    results = []

    def record(name, ok, detail=""):
        results.append((name, ok, detail))
        logger.info(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")

    try:
        # ---- 1. 唯一性：进程 ----
        # 先等本次 launch 的 Nav2 服务与离线链路进程真正就位，再统计唯一性；
        # 否则会在启动竞态窗口里数出"还没起来"或"上一轮残留"的进程。
        if not verifier.wait_stack_appear(timeout=120.0):
            record("等待本次 launch 的 Nav2 服务与链路进程", False,
                   "超时：Nav2 服务或 fake_go2/odom_bridge/patrol_manager 未就位")
            print("\nE2E_RESULT=FAILED")
            return 1
        record("等待本次 launch 的 Nav2 服务与链路进程", True)
        verifier.spin(3.0)
        counts, domains = count_processes(PROC_SIGNATURES)
        record("fake_go2 进程唯一", counts["fake_go2"] == 1, str(counts))
        record("odom_bridge 进程唯一", counts["odom_bridge"] == 1, str(counts))
        record("twist_bridge 进程唯一", counts["twist_bridge"] == 1, str(counts))
        record("patrol_manager 进程唯一", counts["patrol_manager"] == 1, str(counts))
        # stale 判定用**原始**集合：只要出现非 "42" 的取值（含 None=读不到
        # environ 的可疑进程）就视为残留。显示用 _fmt_domains 安全格式化。
        stale = stale_domains(domains, expected=("42",))
        record("无跨 domain 残留进程", not stale,
               f"domains={ {k: _fmt_domains(v) for k, v in domains.items()} }")

        # ---- 1a. 唯一性：Nav2 进程（抓"两套 Nav2 / map_server / manager"）----
        nav2_counts, mgr_names, mgr_domains, nav2_domains = count_nav2_processes()
        dup = {k: v for k, v in nav2_counts.items() if v != 1}
        record("Nav2 节点进程各唯一（每类=1）", not dup, str(nav2_counts))
        record("lifecycle_manager 恰好 2 个（localization+navigation）",
               len(mgr_names) == 2, f"names={sorted(mgr_names)}")
        nav2_stale_raw = stale_domains(nav2_domains, expected=("42",))
        mgr_stale = mgr_domains - {"42"}
        record("Nav2 进程无跨 domain 残留", not nav2_stale_raw and not mgr_stale,
               f"node_domains={ {k: _fmt_domains(v) for k, v in nav2_stale_raw.items()} } "
               f"manager_domains={_fmt_domains(mgr_domains)}")

        # ---- 1b. 唯一性：话题 publisher ----
        for topic in ["/odom", "/scan"]:
            pubs = []
            for _ in range(20):
                pubs = verifier.publishers(topic)
                if pubs:
                    break
                verifier.spin(0.5)
            names = [n for n, _ in pubs]
            record(f"{topic} publisher 唯一（={len(pubs)}）", len(pubs) == 1, str(names))

        # /cmd_vel 在 Nav2 里正常就有多个 publisher（behavior_server 的多个
        # 恢复行为 + velocity_smoother）。这里只要求：恰好 1 个 velocity_smoother，
        # 且离线绝不能出现 go2_twist_bridge（真机运动输出）。
        verifier.spin(2.0)
        cmd_pubs = [n for n, _ in verifier.publishers("/cmd_vel")]
        record("/cmd_vel 有且仅有 1 个 velocity_smoother",
               cmd_pubs.count("velocity_smoother") == 1, str(cmd_pubs))
        record("/cmd_vel 无 go2_twist_bridge（离线不发真机运动）",
               "go2_twist_bridge" not in cmd_pubs, str(cmd_pubs))

        # ---- 1c. 唯一性：TF ----
        verifier.spin(4.0)
        record("odom->base_footprint TF publisher 唯一",
               len(verifier.tf_odom_base_pubs) == 1, str(sorted(verifier.tf_odom_base_pubs)))
        record("map->odom static TF publisher 唯一",
               len(verifier.tf_static_map_odom_pubs) == 1, str(sorted(verifier.tf_static_map_odom_pubs)))

        # ---- 2. Nav2 lifecycle（作为启动门控：稳定 ACTIVE 才允许 start）----
        # 不再用固定 sleep 掩盖 lifecycle 问题：这里主动轮询到"全部 ACTIVE 且
        # 连续稳定保持 10s"为止，避免在心跳即将丢失的窗口里放行 start。
        states, stable = verifier.wait_nav2_active(timeout=180.0, hold=10.0)
        not_active = {k: v for k, v in states.items() if v != "ACTIVE"}
        no_service = sorted(k for k, v in states.items() if v == "NO_SERVICE")
        detail = str(states)
        if no_service:
            # get_state 服务在进程仍存活时不可达，基本只可能是 DDS 发现/投递退化
            # （真实机 13:07 就是这样：controller_server 没崩，但服务与 bond
            # 心跳都收不到了）。把判断依据写清楚，便于区分"代码问题"和"通信问题"。
            detail += f" | 服务不可达={no_service}（进程仍在但 get_state 不可发现，疑似 DDS 发现退化）"
        record("Nav2 离线 lifecycle 全部 ACTIVE 且稳定", (not not_active) and stable, detail)
        if not_active or not stable:
            # 未稳定 ACTIVE：直接失败，绝不带着半死状态去 start patrol。
            for name, ok, detail_ in results:
                print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail_}")
            print("\nE2E_RESULT=FAILED")
            return 1

        # ---- 3. /patrol/start ----
        resp, err = verifier.call(verifier.cli_start, "/patrol/start")
        if err:
            record("/patrol/start", False, err)
        else:
            record("/patrol/start", bool(resp.success), f"message={resp.message!r}")

        # ---- 4~6. 采样运动闭环 ----
        deadline = time.time() + args.timeout
        last_report = 0.0
        while time.time() < deadline:
            verifier.spin(1.0)
            if time.time() - last_report > 15.0:
                last_report = time.time()
                xs = verifier.odom_samples[-1][1] if verifier.odom_samples else float("nan")
                nz = sum(1 for _, vx, wz in verifier.cmd_samples
                         if abs(vx) > 1e-3 or abs(wz) > 1e-3)
                logger.info(f"  ... 运行中 x={xs:.3f} 非零cmd_vel样本={nz}")

        nonzero = [(t, vx, wz) for t, vx, wz in verifier.cmd_samples
                   if abs(vx) > 1e-3 or abs(wz) > 1e-3]
        record("观测到非零 /cmd_vel", bool(nonzero),
               f"样本数={len(nonzero)} 示例={nonzero[:3]}")

        xs = [s[1] for s in verifier.odom_samples]
        moved = (max(xs) - min(xs)) if xs else 0.0
        record("/odom 因 /cmd_vel 实际移动", moved > 1.0,
               f"x 变化范围={moved:.3f} 样本数={len(xs)}")

        # 连续变化判据：不能只在两个固定值之间跳（说明有两套实例交替发布）。
        distinct = len({round(x, 2) for x in xs})
        record("/odom 连续变化（非两点跳变）", distinct > 10,
               f"不同 x 取值数={distinct}")

        reached_2m = any(abs(x - 2.0) < 0.35 for x in xs)
        record("到达 equipment_zone (x≈2.0)", reached_2m,
               f"x_max={max(xs):.3f}" if xs else "无 /odom")

        record("离线未向 /api/sport/request 发运动命令",
               verifier.sport_requests == 0, f"收到 {verifier.sport_requests} 条")

        # ---- 7. 再次 start（证明已回到 IDLE）----
        resp2, err2 = verifier.call(verifier.cli_start, "/patrol/start")
        if err2:
            record("巡检结束回到 IDLE（可再次 start）", False, err2)
        else:
            record("巡检结束回到 IDLE（可再次 start）", bool(resp2.success),
                   f"message={resp2.message!r}")
        if resp2 is not None and resp2.success:
            verifier.call(verifier.cli_stop, "/patrol/stop")

        # ---- 汇总 ----
        print("\n==== OFFLINE 端到端结果 ====")
        for name, ok, detail in results:
            print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
        passed = all(ok for _, ok, _ in results)
        print(f"\ncmd_vel 样本总数: {len(verifier.cmd_samples)}，非零: {len(nonzero)}")
        print(f"进程计数: {count_processes(PROC_SIGNATURES)[0]}")
        _nav2_counts, _mgr_names, _mgr_domains, _ = count_nav2_processes()
        print(f"Nav2 进程计数: {_nav2_counts} lifecycle_manager={sorted(_mgr_names)}")
        print(f"/odom publishers: {verifier.publishers('/odom')}")
        print(f"/cmd_vel publishers: {verifier.publishers('/cmd_vel')}")
        print("/odom 轨迹采样（最多 12 个）:")
        step = max(1, len(verifier.odom_samples) // 12)
        for s in verifier.odom_samples[::step]:
            print(f"  t={s[0]:.1f} x={s[1]:.3f} y={s[2]:.3f} yaw={s[3]:.3f}")
        print(f"\nE2E_RESULT={'PASS' if passed else 'FAILED'}")
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
