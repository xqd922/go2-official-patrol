#!/usr/bin/env python3
"""GUI 入口：`bash tools/patrol_gui/run.sh` 最终执行到这里。

只做三件事：定位仓库根、建 ROS 桥（失败也不崩，界面仍可用来看航点）、
起 tkinter 主循环。真正的业务逻辑都在 gui.py / launch_manager.py 里，
方便在不打开窗口的情况下做单元测试。
"""
import argparse
import os
import sys
import traceback


def repo_root():
    """tools/patrol_gui/app.py -> 仓库根。"""
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def main(argv=None):
    parser = argparse.ArgumentParser(description="Go2 建图与巡检点可视化工具")
    parser.add_argument("--no-ros", action="store_true",
                        help="不连 ROS（只查看/编辑 waypoints.yaml，用于调试界面）")
    options = parser.parse_args(argv)

    import tkinter as tk

    from .gui import PatrolGui

    bridge = None
    if not options.no_ros:
        try:
            from .ros_bridge import RosBridge
            import rclpy
            rclpy.init()
            # 把 GUI 正在编辑的 waypoints.yaml 交给桥：RViz 里的
            # /patrol/waypoint_markers 直接反映这份文件（A/B/C/D + 名称）。
            # 位置与文件都取自同一处，避免"界面显示一套、RViz 显示另一套"。
            waypoints_path = os.path.join(
                repo_root(), "src", "go2_patrol_bringup", "config",
                "waypoints.yaml")
            bridge = RosBridge(waypoints_path=waypoints_path)
        except Exception as exc:
            # ROS 起不来不应让界面打不开：用户仍能查看/编辑航点，
            # 只是位姿与巡检按钮不可用，界面会明确提示。
            print(f"[gui] ROS 初始化失败，进入只读模式：{exc}", file=sys.stderr)
            traceback.print_exc()
            bridge = None

    root = tk.Tk()
    app = PatrolGui(root, repo_root(), bridge=bridge)
    try:
        root.mainloop()
    finally:
        if bridge is not None:
            try:
                import rclpy
                rclpy.shutdown()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
