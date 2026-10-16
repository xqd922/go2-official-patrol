#!/usr/bin/env python3
"""OFFLINE 专用：确认**当前这次 launch**的 Nav2 就绪后，才触发 lifecycle bringup。

上一轮真实机 FAILED 的根因：nav2_starter 用 ExecuteProcess 立即启动，而
navigation.launch.py 里 Nav2 被 TimerAction 延迟 3s 启动。若同 ROS_DOMAIN_ID
里残留了上一次运行的 Nav2，starter 会先发现并操作**旧实例**；3s 后当前 launch
又起一套新的，于是出现"planner_server / behavior_server 一直 state=1"、
"lifecycle_manager_localization STARTUP 超时"以及两套进程先后出现的现象。

修复分三部分（配套的另外两处在 navigation.launch.py 的 startup_delay 参数与
run_offline_e2e.py 的按 domain 清理残留）：
  1. 不再只看"服务是否存在"，而是确认该节点进程是**本次 launch 之后**才启动的
     （比较 /proc/<pid>/stat 的 starttime 与本进程 starttime）。旧的残留实例
     启动时间早于本次 launch，会被正确排除。
  2. 用进程启动时间比较而非 /proc/.../children，因为 launch 的中间 wrapper
     退出后节点会被 reparent，进程树读法会漏掉节点。
  3. 进程出现 + 服务可用两者都满足才发 STARTUP，且 STARTUP 返回 success=False
     立即失败，不再无声继续。
"""
import os
import sys
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node


def log(msg):
    print(f"[nav2_starter] {msg}", flush=True)


def _clock_ticks_per_sec():
    try:
        return os.sysconf("SC_CLK_TCK")
    except (ValueError, OSError):
        return 100


TICKS = _clock_ticks_per_sec()


def _self_starttime():
    with open("/proc/self/stat") as stream:
        # 字段 22 是 starttime；comm 可能含空格/括号，故从最后一个 ')' 之后切分
        fields = stream.read().rsplit(")", 1)[1].split()
    return int(fields[19])


def _fresh_exe_set(since_ticks):
    """返回启动时间晚于 since_ticks 的进程可执行名集合（含被 reparent 的节点）。

    用 /proc/<pid>/cmdline 的 argv[0] 基名，而不是 comm：
    Linux comm 只有 15 字符，controller_server(17) / velocity_smoother(16) /
    waypoint_follower(17) 会被截断，导致"节点明明起来了却匹配不上"。
    """
    names = set()
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = entry
        try:
            with open(f"/proc/{pid}/stat") as stream:
                stat = stream.read()
            starttime = int(stat.rsplit(")", 1)[1].split()[19])
            if starttime < since_ticks:
                continue
            with open(f"/proc/{pid}/cmdline", "rb") as stream:
                argv0 = stream.read().split(b"\0", 1)[0].decode(errors="replace")
        except (OSError, IndexError, ValueError):
            continue
        if argv0:
            names.add(os.path.basename(argv0))
    return names


class Nav2Starter(Node):
    def __init__(self):
        super().__init__("nav2_starter")
        # 本进程由 ExecuteProcess 拉起，其启动时间即"本次 launch"的时间基准。
        self.since_ticks = _self_starttime()
        self.declare_parameter("managers", ["lifecycle_manager_localization", "lifecycle_manager_navigation"])
        self.declare_parameter("nodes", ["map_server", "controller_server", "planner_server",
                                         "smoother_server", "behavior_server", "bt_navigator",
                                         "waypoint_follower", "velocity_smoother"])
        self.declare_parameter("ready_timeout", 120.0)
        self.declare_parameter("active_timeout", 120.0)

    def _wait_current_services(self, nodes):
        """等本次 launch 启动的节点进程出现，且其 get_state 服务可用。"""
        clients = {}
        start = time.time()
        timeout = float(self.get_parameter("ready_timeout").value)
        while time.time() - start < timeout:
            procs = _fresh_exe_set(self.since_ticks)
            fresh = {n for n in nodes if n in procs}
            if len(fresh) != len(nodes):
                log(f"等待本次 launch 的 Nav2 进程：还缺 {sorted(set(nodes) - fresh)}（已有: {sorted(fresh)}）")
            else:
                all_ready = True
                for name in nodes:
                    client = self.create_client(GetState, f"/{name}/get_state")
                    if not client.wait_for_service(timeout_sec=1.0):
                        all_ready = False
                        break
                    clients[name] = client
                if all_ready:
                    log("本次 launch 的所有 Nav2 lifecycle 节点已出现且服务可用")
                    return clients
                clients = {}
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.3)
        return None

    def _wait_service(self, srv_type, name, timeout):
        client = self.create_client(srv_type, name)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                log(f"服务 {name} 已就绪")
                return client
            rclpy.spin_once(self, timeout_sec=0.1)
        log(f"服务 {name} 超时未出现")
        return None

    def run(self):
        nodes = list(self.get_parameter("nodes").value)
        managers = list(self.get_parameter("managers").value)
        active_timeout = float(self.get_parameter("active_timeout").value)
        ready_timeout = float(self.get_parameter("ready_timeout").value)
        log(f"nav2_starter 启动，基准 starttime={self.since_ticks} ticks")
        log(f"节点列表: {nodes}")
        log(f"manager 列表: {managers}")

        get_state = self._wait_current_services(nodes)
        if get_state is None:
            self.get_logger().error("本次 launch 的 Nav2 节点未能在 ready_timeout 内全部就绪")
            return False

        for manager in managers:
            client = self._wait_service(ManageLifecycleNodes, f"/{manager}/manage_nodes", ready_timeout)
            if client is None:
                self.get_logger().error(f"{manager}: manage_nodes 服务未就绪")
                return False
            request = ManageLifecycleNodes.Request()
            request.command = ManageLifecycleNodes.Request.STARTUP
            log(f"调用 {manager} STARTUP")
            future = client.call_async(request)
            deadline = time.time() + active_timeout
            while not future.done() and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not future.done():
                self.get_logger().error(f"{manager}: STARTUP 在 {active_timeout}s 内未返回")
                return False
            ok = future.result().success
            log(f"{manager}: STARTUP -> success={ok}")
            if not ok:
                return False

        deadline = time.time() + active_timeout
        pending = set(nodes)
        while pending and time.time() < deadline:
            for name in list(pending):
                future = get_state[name].call_async(GetState.Request())
                spin_deadline = time.time() + 5.0
                while not future.done() and time.time() < spin_deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)
                if future.done() and future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                    self.get_logger().info(f"{name}: active")
                    pending.discard(name)
            if pending:
                time.sleep(0.5)
        if pending:
            self.get_logger().error(f"以下节点未进入 active：{sorted(pending)}")
            return False
        log("Nav2 全部 lifecycle 节点已 active")
        return True


def main(args=None):
    log("nav2_starter 启动")
    rclpy.init(args=args)
    node = Nav2Starter()
    ok = False
    try:
        ok = node.run()
    finally:
        log(f"nav2_starter 结束，结果={'OK' if ok else 'FAIL'}")
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

