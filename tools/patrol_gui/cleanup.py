"""GUI 退出时的 domain 维度兜底清理。

只清理**当前 ROS_DOMAIN_ID** 下、cmdline 命中项目链路签名的进程：
  * 不会碰别的 ROS_DOMAIN_ID 的任务（先比对 /proc/<pid>/environ）；
  * 不会碰调用 GUI 的 shell/终端（shell 的 cmdline 不含这些签名）；
  * 读不到 environ 的进程一律跳过（宁可漏清，绝不误杀）。

主生命周期管理仍是 launch_manager 的"独立进程组整组 SIGTERM/SIGKILL"，
这里只是防止"某个 launch 意外脱组"时留下孤儿。
"""
import os

from . import proc

# 只覆盖本项目会启动的节点；与 OFFLINE E2E 的签名保持同一风格。
CHAIN_SIGNATURES = {
    "fake_go2": "fake_go2.py",
    "odom_bridge": "odom_bridge",
    "twist_bridge": "twist_bridge",
    "battery_bridge": "battery_bridge",
    "patrol_manager": "patrol_manager",
    "voice_player": "voice_player",
    "slam_toolbox": "slam_toolbox",
    "pointcloud_to_laserscan": "pointcloud_to_laserscan",
    "static_transform_publisher": "static_transform_publisher",
    "controller_server": "nav2_controller/controller_server",
    "planner_server": "nav2_planner/planner_server",
    "smoother_server": "nav2_smoother/smoother_server",
    "behavior_server": "nav2_behaviors/behavior_server",
    "bt_navigator": "nav2_bt_navigator/bt_navigator",
    "waypoint_follower": "nav2_waypoint_follower/waypoint_follower",
    "velocity_smoother": "nav2_velocity_smoother/velocity_smoother",
    "map_server": "nav2_map_server/map_server",
    "lifecycle_manager": "nav2_lifecycle_manager/lifecycle_manager",
    "amcl": "nav2_amcl/amcl",
}

# 排除 GUI 自身 cmdline（含 patrol_gui 路径），避免把自己算成残留。
_SELF_MARKERS = ("patrol_gui", "e2e_offline_patrol", "run_offline_e2e",
                 "offline_proc")


def current_domain():
    return os.environ.get("ROS_DOMAIN_ID")


def cleanup_domain(logger=print, domain=None):
    """清理当前 domain 下的链路残留进程；返回被清理的数量。

    ROS_DOMAIN_ID 未设置时直接返回 0 —— 无法界定范围就不动手，
    这比"猜一个 domain 去清"安全得多。
    """
    domain = domain if domain is not None else current_domain()
    if domain is None:
        logger("[gui] 未设置 ROS_DOMAIN_ID，跳过 domain 兜底清理")
        return 0
    count = proc.kill_domain_processes(
        domain, CHAIN_SIGNATURES, exclude_substrings=_SELF_MARKERS, log=logger)
    if count:
        logger(f"[gui] domain={domain} 兜底清理完成，共清理 {count} 个进程")
    else:
        logger(f"[gui] domain={domain} 无残留进程")
    return count
