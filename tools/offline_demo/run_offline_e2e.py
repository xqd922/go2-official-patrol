#!/usr/bin/env python3
"""单进程跑完 OFFLINE 定点巡检端到端。

用法：
    source scripts/setup_offline.sh
    $GO2_PYTHON tools/offline_demo/run_offline_e2e.py [timeout_seconds]

进程生命周期（本文件的重点）：
  1. ros2 launch 跑在**独立会话/进程组**里（子进程启动时 os.setsid()）。
     ros2 launch 派生的所有节点都继承这个进程组，因此清理时对**整组**发信号
     就能一次带走完整子进程树；同时因为换到了新会话，绝不会误伤调用 E2E 的
     shell/终端（它在另一个会话里），也不会碰到别的 ROS_DOMAIN_ID 任务。
  2. 无论 PASS / FAIL / Python 异常 / 超时，都走 finally 里的同一套清理：
     先 SIGTERM 整组 → 有限时间内等它自己退出 → 仍存活的再 SIGKILL 整组。
     不用 pgrep + ROS_DOMAIN_ID 当主生命周期管理（那只作最后兜底）。
  3. 清理后统计本次链路进程，确认 fake_go2 / driver / Nav2 / patrol_manager = 0。

为什么不用 ros2 launch 的父进程 terminate()：launch 对 SIGTERM 只取消自身
asyncio 任务、不会给子节点转发信号（launch 源码里自己都打印 "using SIGTERM
can result in orphaned processes"），于是 ros2 节点会变成 PPID=1 的孤儿残留。
所以必须对**进程组**发信号，而不是只 terminate launch 父进程。
"""
import multiprocessing as mp
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from offline_proc import (  # noqa: E402
    domain_matches,
    filter_started_since,
    find_processes,
    proc_starttime,
    terminate_group,
    wait_pgid_ready,
)

WS = os.path.abspath(os.path.join(HERE, "..", ".."))          # tools/offline_demo -> 仓库根
LAUNCH = os.path.join(WS, "src", "go2_patrol_bringup", "launch", "offline_patrol.launch.py")
MAP = os.path.join(WS, "src", "go2_patrol_bringup", "maps", "offline_room.yaml")

# 本次离线链路进程签名：仅用于"清理后确认残留=0"，不用于杀进程。
CHAIN_SIGNATURES = {
    "fake_go2": "fake_go2.py",
    "odom_bridge": "odom_bridge",
    "twist_bridge": "twist_bridge",
    "battery_bridge": "battery_bridge",
    "patrol_manager": "patrol_manager",
    "nav2_starter": "nav2_starter.py",
    "controller_server": "nav2_controller/controller_server",
    "planner_server": "nav2_planner/planner_server",
    "smoother_server": "nav2_smoother/smoother_server",
    "behavior_server": "nav2_behaviors/behavior_server",
    "bt_navigator": "nav2_bt_navigator/bt_navigator",
    "waypoint_follower": "nav2_waypoint_follower/waypoint_follower",
    "velocity_smoother": "nav2_velocity_smoother/velocity_smoother",
    "map_server": "nav2_map_server/map_server",
    "lifecycle_manager": "nav2_lifecycle_manager/lifecycle_manager",
    "pointcloud_to_laserscan": "pointcloud_to_laserscan",
    "static_transform_publisher": "static_transform_publisher",
    "voice_player": "voice_player",
}


# 排除本 E2E 自身的 cmdline：它们包含上面的签名字符串（如 offline_proc），
# 若不排除会把 E2E 进程自己误判成"残留链路进程"。
_SELF_CMDLINE_MARKERS = ("run_offline_e2e", "e2e_offline_patrol", "offline_proc")


def _chain_processes():
    """列出当前仍存活的本次链路进程 (pid, signature, cmdline, domain)。"""
    return find_processes(
        CHAIN_SIGNATURES,
        exclude_substrings=_SELF_CMDLINE_MARKERS,
    )


def _chain_processes_since(ticks):
    """只列出 starttime >= ticks（即"本次运行期间新起"）的链路进程。

    用于残留判定：上一轮崩溃留下的孤儿启动时间早于本次运行，不应该被算作
    "本次 E2E 泄漏"。启动前 _cleanup_domain() 会先把它们清掉；这里再做一次
    过滤，保证残留=0 这个结论只反映本次运行的清理是否干净。
    """
    return filter_started_since(_chain_processes(), ticks)


def _cleanup_domain(domain, grace=5.0):
    """兜底：按 ROS_DOMAIN_ID 精确清理本离线 domain 的链路残留进程。

    这是**启动前**的安全网（清掉上一次 E2E 崩溃留下的孤儿，真实机曾观测到
    12:26 那批 static_transform_publisher / voice_player 变成 PPID=1 的孤儿，
    导致下一次运行出现重复 TF publisher 与 /odom 跳变），以及**结束后**的
    兜底。主生命周期管理仍是"独立进程组 + 整组信号"，不依赖本函数。

    只清 ROS_DOMAIN_ID == domain 的进程，绝不误伤其它 domain 的任务；
    只按 cmdline 里的链路签名匹配，绝不匹配调用本脚本的 shell/终端
    （shell 的 cmdline 不含这些签名）。
    """
    me = os.getpid()
    targets = []
    for pid, key, cmd, dom in _chain_processes():
        if pid == me:
            continue
        if domain_matches(pid, domain):
            targets.append((pid, key, cmd))
    if not targets:
        return 0
    for pid, key, cmd in targets:
        print(f"[driver] 清理 domain={domain} 残留 pid={pid} ({key}) cmd={cmd}", flush=True)
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    # 有限等待；仍存活的升级 SIGKILL。只对上面匹配到的具体 pid 发信号，
    # 只对匹配到的具体 pid 发信号，从不给进程组 0 或自身所在组发信号，
    # 因此不可能连带杀掉调用本脚本的 shell/终端。
    deadline = time.time() + grace
    while time.time() < deadline:
        alive = [pid for pid, _, _ in targets if _pid_alive(pid)]
        if not alive:
            return len(targets)
        time.sleep(0.2)
    for pid in [pid for pid, _, _ in targets if _pid_alive(pid)]:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    time.sleep(0.5)
    return len(targets)


def _pid_alive(pid):
    """进程存在且非僵尸。"""
    try:
        with open(f"/proc/{pid}/stat") as stream:
            return stream.read().rsplit(")", 1)[1].split()[0] != "Z"
    except (OSError, IndexError):
        return False


if not os.path.isfile(MAP):
    # 退回：挑 maps/ 下第一张 yaml
    maps_dir = os.path.dirname(MAP)
    if os.path.isdir(maps_dir):
        cands = sorted(n for n in os.listdir(maps_dir) if n.endswith(".yaml"))
        if cands:
            MAP = os.path.join(maps_dir, cands[0])
if not os.path.isfile(MAP):
    sys.exit(f"找不到离线地图，请先准备 {os.path.dirname(MAP)}/*.yaml")


def launch_proc():
    # 关键：把自己变成新的会话/进程组组长（pgid == pid）。ros2 launch 派生的
    # 所有节点都继承这个 pgid，于是清理时对整组发一次信号即可带走完整子进程树。
    # 因为换到了独立会话，调用 E2E 的 shell/终端在另一个会话，绝不受影响。
    try:
        os.setsid()
    except OSError:
        pass

    from launch import LaunchDescription, LaunchService
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import LaunchConfiguration

    ld = LaunchDescription([
        DeclareLaunchArgument("map", default_value=MAP),
        DeclareLaunchArgument("rviz", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(LAUNCH),
            launch_arguments={"map": LaunchConfiguration("map"),
                              "rviz": LaunchConfiguration("rviz")}.items(),
        ),
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    sys.exit(ls.run())


def verify_proc(timeout):
    sys.argv = ["e2e_offline_patrol.py", "--timeout", str(timeout)]
    sys.path.insert(0, HERE)
    import e2e_offline_patrol
    os._exit(e2e_offline_patrol.main())


if __name__ == "__main__":
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0

    # 本次运行的起点：残留判定只看"此刻之后"新起的进程，历史孤儿不计入。
    run_start_ticks = proc_starttime(os.getpid())

    # 启动前先清理本离线 domain 的残留进程。上一轮真实机 FAILED 的直接诱因：
    # ROS_DOMAIN_ID=42 里残留了上一次的 fake_go2/odom_bridge/Nav2/TF/voice，
    # 导致 /odom x=0 <-> 1.727 两个实例交替发布、nav2_starter 连到旧实例、
    # 出现重复 TF publisher。这里必须**真正**清掉历史 domain=42 残留（此前
    # 因 `"42" == 42` 恒 False 而完全没生效）。主生命周期管理仍是下面的
    # "独立进程组 + 整组 SIGTERM/SIGKILL"。
    _cleanup_domain(42)

    launch_pgid = None
    lp = None
    vp = None
    code = -1
    try:
        # daemon=False：launch 子进程要能继续 fork 出 ros2 节点，且我们要
        # 显式管理它的生命周期（在 finally 里）。
        lp = mp.Process(target=launch_proc, daemon=False)
        lp.start()
        # 等它 setsid 完成，拿到确定且隔离的 pgid（== pid）。
        launch_pgid = wait_pgid_ready(lp.pid, timeout=5.0)
        if launch_pgid is None:
            print(f"[driver] 警告：未能确认 launch 进程组（pid={lp.pid}），"
                  f"将退回到逐进程 + domain 清理", flush=True)
        else:
            print(f"[driver] launch 进程组 pgid={launch_pgid}", flush=True)
        print(f"[driver] launch 子进程 pid={lp.pid}，交由验证器轮询 Nav2 就绪", flush=True)
        time.sleep(5)
        # 验证器放在主进程组（不是 launch 组），因此清理 launch 组不会误杀它。
        vp = mp.Process(target=verify_proc, args=(timeout,), daemon=False)
        vp.start()
        vp.join(timeout + 120)
        code = vp.exitcode if vp.exitcode is not None else -1
        print(f"[driver] 验证进程退出码={code}", flush=True)
    finally:
        # 无论 PASS / FAIL / 异常 / 超时，都执行同一套清理。
        # 1) 先收掉可能还在跑的验证器进程（它属于主进程组，不会被动到 launch 组）。
        if vp is not None and vp.is_alive():
            vp.terminate()
            vp.join(5)
            if vp.is_alive():
                vp.kill()
                vp.join(5)
        # 2) SIGTERM 整组 -> 有限等待 -> 仍存活的 SIGKILL 整组。
        if launch_pgid is not None:
            terminate_group(launch_pgid)
        # 3) 兜底：万一 setsid 没成功（pgid 未能隔离），只终止 launch 父进程，
        #    绝不 killpg 主进程组（否则会连带杀掉本 E2E 自己/调用它的 shell）。
        if lp is not None and lp.is_alive():
            lp.terminate()
            lp.join(5)
            if lp.is_alive():
                lp.kill()
                lp.join(5)
        # 4) 再跑一次 domain 兜底清理，清掉任何 reparent 到 init 的漏网节点
        #    （包括上一轮遗留的历史孤儿）。
        _cleanup_domain(42)
        # 5) 确认**本次运行**的链路进程残留=0（历史孤儿已在上一步清掉；即便
        #    有清不掉的历史孤儿，也不算本次泄漏，只单独提示）。
        residue = _chain_processes_since(run_start_ticks)
        leftover_pre = [x for x in _chain_processes()
                        if x not in residue]
        if residue:
            print(f"[driver] 清理后仍有 {len(residue)} 个本次链路进程残留：", flush=True)
            for pid, key, cmd, domain in residue:
                print(f"[driver]   pid={pid} {key} domain={domain} cmd={cmd}", flush=True)
            code = 1 if code == 0 else code
        else:
            print("[driver] 清理后本次链路进程残留=0"
                  "（fake_go2/driver/Nav2/patrol_manager/static TF/voice 全清）",
                  flush=True)
        if leftover_pre:
            print(f"[driver] 注意：另有 {len(leftover_pre)} 个启动前就存在的"
                  f"同 domain 进程未被清理（非本次泄漏）：", flush=True)
            for pid, key, cmd, domain in leftover_pre:
                print(f"[driver]   pid={pid} {key} domain={domain} cmd={cmd}", flush=True)
    sys.exit(0 if code == 0 else 1)
