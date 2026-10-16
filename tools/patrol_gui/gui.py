#!/usr/bin/env python3
"""Go2 可视化建图与巡检点设置工具（tkinter，纯 Python 标准库）。

为什么用 tkinter：ROS 2 Humble 的 Python 环境自带 tkinter（python3-tk），
不需要引入任何 Web 框架/前端构建链，`bash tools/patrol_gui/run.sh` 一条命令
即可打开，符合"稳定、简单、能用"的目标。

它只做三件事，全部复用项目现有资产：
  1. 建图 / 导航：跑现有 mapping.launch.py 与 patrol.launch.py；
  2. 存图：跑现有 scripts/save_map.sh；
  3. 记航点：读 **map 系** 的机器人位姿写入现有 waypoints.yaml，
     巡检按钮直接调现有 /patrol/{start,pause,resume,stop}，不重写状态机。

OFFLINE / REAL 由启动脚本选择环境（setup_offline.sh / setup.sh），
GUI 本身不含任何 map->odom 逻辑，因此不会把离线静态定位带进真机。
"""
import os
import queue
import threading
import traceback

import tkinter as tk
from tkinter import messagebox, ttk

from .launch_manager import MAPPING, NAVIGATION, LaunchManager, save_map
from .waypoint_store import WaypointStore


class PatrolGui:
    def __init__(self, root, ws_root, bridge=None, launch_manager=None,
                 store=None, logger=print):
        self.root = root
        self.ws_root = ws_root
        self.log = logger
        self.maps_dir = os.path.join(ws_root, "src", "go2_patrol_bringup", "maps")
        self.waypoints_path = os.path.join(
            ws_root, "src", "go2_patrol_bringup", "config", "waypoints.yaml")

        self.bridge = bridge
        self.launcher = launch_manager or LaunchManager(ws_root, logger=logger)
        self.store = store or WaypointStore(self.waypoints_path)

        self._results = queue.Queue()
        self._busy = False

        self.root.title("Go2 建图与巡检点工具")
        self.root.geometry("1000x680")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self._build_widgets()
        self.refresh_maps()
        self.refresh_waypoints()
        self._poll_results()
        self._tick_status()

    # ================= 界面 =================
    def _build_widgets(self):
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        # ---- 顶部：机器人状态 ----
        status = ttk.LabelFrame(outer, text="机器人状态（map 坐标系）", padding=8)
        status.pack(fill="x")
        self.var_pose = tk.StringVar(value="x=--  y=--  yaw=--")
        self.var_patrol = tk.StringVar(value="巡检状态：--")
        self.var_goal = tk.StringVar(value="当前目标点：--")
        self.var_count = tk.StringVar(value="已记录航点：0")
        ttk.Label(status, textvariable=self.var_pose, font=("TkDefaultFont", 12, "bold")
                  ).grid(row=0, column=0, sticky="w", padx=(0, 24))
        ttk.Label(status, textvariable=self.var_patrol).grid(row=0, column=1, sticky="w", padx=(0, 24))
        ttk.Label(status, textvariable=self.var_goal).grid(row=0, column=2, sticky="w", padx=(0, 24))
        ttk.Label(status, textvariable=self.var_count).grid(row=0, column=3, sticky="w")
        self.var_hint = tk.StringVar(value="")
        ttk.Label(status, textvariable=self.var_hint, foreground="#a05000"
                  ).grid(row=1, column=0, columnspan=4, sticky="w", pady=(6, 0))

        # ---- 中部：左=操作区，右=航点列表 ----
        middle = ttk.Frame(outer)
        middle.pack(fill="both", expand=True, pady=10)

        left = ttk.Frame(middle)
        left.pack(side="left", fill="y", padx=(0, 10))

        # 地图
        mapbox = ttk.LabelFrame(left, text="地图", padding=8)
        mapbox.pack(fill="x")
        self.var_map = tk.StringVar()
        self.combo_map = ttk.Combobox(mapbox, textvariable=self.var_map, width=34,
                                      state="readonly")
        self.combo_map.pack(fill="x")
        row = ttk.Frame(mapbox)
        row.pack(fill="x", pady=(6, 0))
        ttk.Button(row, text="刷新地图列表", command=self.refresh_maps).pack(side="left")
        self.var_map_name = tk.StringVar(value="office_floor")
        ttk.Label(mapbox, text="保存为新地图名：").pack(anchor="w", pady=(8, 0))
        ttk.Entry(mapbox, textvariable=self.var_map_name).pack(fill="x")
        self.var_map_info = tk.StringVar(value="尚未保存地图")
        ttk.Label(mapbox, textvariable=self.var_map_info, wraplength=260,
                  foreground="#206020").pack(anchor="w", pady=(6, 0))

        # 建图 / 导航
        runbox = ttk.LabelFrame(left, text="建图与导航", padding=8)
        runbox.pack(fill="x", pady=10)
        self.btn_mapping = ttk.Button(runbox, text="开始建图",
                                      command=lambda: self.on_start_launch(MAPPING))
        self.btn_mapping.pack(fill="x")
        ttk.Label(runbox, text="建图：遥控走一圈，然后点【保存地图】",
                  wraplength=260, foreground="#555555").pack(anchor="w", pady=(2, 8))
        self.btn_save = ttk.Button(runbox, text="保存地图", command=self.on_save_map)
        self.btn_save.pack(fill="x")
        self.btn_nav = ttk.Button(runbox, text="启动导航",
                                  command=lambda: self.on_start_launch(NAVIGATION))
        self.btn_nav.pack(fill="x", pady=(8, 0))
        ttk.Label(runbox, text="导航：使用选中的真实地图启动 patrol.launch.py（AMCL）",
                  wraplength=260, foreground="#555555").pack(anchor="w", pady=(2, 8))
        self.btn_stop_launch = ttk.Button(runbox, text="停止建图/导航",
                                          command=self.on_stop_launch)
        self.btn_stop_launch.pack(fill="x")

        # 记录航点
        wpbox = ttk.LabelFrame(left, text="巡检点", padding=8)
        wpbox.pack(fill="x")
        ttk.Label(wpbox, text="名称（例如 entrance / equipment_zone）：").pack(anchor="w")
        self.var_wp_name = tk.StringVar()
        entry = ttk.Entry(wpbox, textvariable=self.var_wp_name)
        entry.pack(fill="x")
        entry.bind("<Return>", lambda _e: self.on_record_waypoint())
        ttk.Button(wpbox, text="记录当前位置为巡检点",
                   command=self.on_record_waypoint).pack(fill="x", pady=(6, 0))
        self.var_home = tk.BooleanVar(value=False)
        ttk.Checkbutton(wpbox, text="设为 home（回充/起点）",
                        variable=self.var_home).pack(anchor="w", pady=(4, 0))
        ttk.Button(wpbox, text="删除选中的巡检点",
                   command=self.on_delete_waypoint).pack(fill="x", pady=(6, 0))

        # 巡检控制
        ctl = ttk.LabelFrame(left, text="巡检控制", padding=8)
        ctl.pack(fill="x", pady=10)
        for label, action in (("开始巡检", "start"), ("暂停巡检", "pause"),
                              ("继续巡检", "resume"), ("停止巡检", "stop")):
            ttk.Button(ctl, text=label,
                       command=lambda a=action: self.on_patrol(a)).pack(fill="x", pady=1)

        # 右侧：航点列表
        right = ttk.LabelFrame(middle, text="已记录的巡检点", padding=8)
        right.pack(side="left", fill="both", expand=True)
        columns = ("idx", "name", "x", "y", "yaw", "dwell", "home")
        self.tree = ttk.Treeview(right, columns=columns, show="headings", height=18)
        headings = {"idx": "#", "name": "名称", "x": "x", "y": "y",
                    "yaw": "yaw", "dwell": "dwell", "home": "home"}
        widths = {"idx": 36, "name": 150, "x": 80, "y": 80, "yaw": 80,
                  "dwell": 60, "home": 60}
        for col in columns:
            self.tree.heading(col, text=headings[col])
            self.tree.column(col, width=widths[col],
                             anchor="center" if col != "name" else "w")
        self.tree.pack(fill="both", expand=True)
        ttk.Button(right, text="查看巡检点（刷新）",
                   command=self.refresh_waypoints).pack(fill="x", pady=(6, 0))
        self.var_wp_path = tk.StringVar()
        ttk.Label(right, textvariable=self.var_wp_path, wraplength=420,
                  foreground="#555555").pack(anchor="w", pady=(6, 0))

        # ---- 底部：日志 ----
        logbox = ttk.LabelFrame(outer, text="日志", padding=6)
        logbox.pack(fill="both")
        self.text_log = tk.Text(logbox, height=8, wrap="word")
        self.text_log.pack(fill="both", expand=True)
        self.text_log.configure(state="disabled")

    # ================= 工具方法 =================
    def _log(self, message):
        self.text_log.configure(state="normal")
        self.text_log.insert("end", str(message) + "\n")
        self.text_log.see("end")
        self.text_log.configure(state="disabled")
        self.log(message)

    def _run_async(self, work, on_done, label):
        """在后台线程跑 work()，结果回到 GUI 线程由 on_done 处理。"""
        if self._busy:
            self._log("[gui] 上一个操作还没结束，请稍候")
            return
        self._busy = True
        self._log(f"[gui] {label} ...")

        def target():
            try:
                result = work()
                self._results.put((on_done, True, result))
            except Exception as exc:
                self._results.put((on_done, False, f"{exc}\n{traceback.format_exc()}"))

        threading.Thread(target=target, daemon=True).start()

    def _poll_results(self):
        try:
            while True:
                callback, ok, payload = self._results.get_nowait()
                self._busy = False
                try:
                    callback(ok, payload)
                except Exception:
                    self._log("[gui] 回调异常：\n" + traceback.format_exc())
        except queue.Empty:
            pass
        self.root.after(100, self._poll_results)

    # ================= 地图 =================
    def list_maps(self):
        if not os.path.isdir(self.maps_dir):
            return []
        names = [n for n in sorted(os.listdir(self.maps_dir)) if n.endswith(".yaml")]
        return names

    def refresh_maps(self):
        names = self.list_maps()
        self.combo_map["values"] = names
        if names and not self.var_map.get():
            # 默认选最新修改的那张，符合"刚保存完就启动导航"的使用顺序。
            newest = max(names, key=lambda n: os.path.getmtime(
                os.path.join(self.maps_dir, n)))
            self.var_map.set(newest)
        elif self.var_map.get() not in names:
            self.var_map.set("")
        self._log(f"[gui] 地图列表：{names or '（空）'}")

    def selected_map_path(self):
        name = self.var_map.get()
        if not name:
            return None
        return os.path.join(self.maps_dir, name)

    # ================= 建图 / 导航 =================
    def on_start_launch(self, mode):
        running = self.launcher.running_modes()
        if mode in running:
            messagebox.showinfo("提示", f"{mode} 已经在运行")
            return
        if running:
            messagebox.showwarning(
                "提示", "已经有建图/导航在运行，请先点【停止建图/导航】再启动另一个")
            return
        if mode == NAVIGATION:
            map_path = self.selected_map_path()
            if not map_path:
                messagebox.showwarning("提示", "请先选择一张地图（或先建图并保存）")
                return
            work = lambda: self.launcher.start_navigation(map_path)
        else:
            work = lambda: self.launcher.start_mapping()
        self._run_async(work, lambda ok, payload: self._after_launch(mode, ok, payload),
                        f"启动 {mode}")

    def _after_launch(self, mode, ok, payload):
        if not ok:
            self._log(f"[gui] 启动 {mode} 失败：{payload}")
            messagebox.showerror("启动失败", f"启动 {mode} 失败：\n{payload}")
            return
        log_path = self.launcher.log_path(mode)
        self._log(f"[gui] {mode} 已启动，日志：{log_path}")
        if mode == MAPPING:
            self.var_hint.set("建图中：请用遥控器控制 Go2 走一圈，完成后点【保存地图】。")
        else:
            self.var_hint.set("导航已启动：等 Nav2 就绪后可记录巡检点并开始巡检。")

    def on_stop_launch(self):
        if not self.launcher.running_modes():
            self._log("[gui] 当前没有正在运行的建图/导航")
            return
        self._run_async(lambda: self.launcher.stop_all(),
                        lambda ok, payload: self._log("[gui] 已停止建图/导航"),
                        "停止建图/导航")
        self.var_hint.set("")

    def on_save_map(self):
        name = (self.var_map_name.get() or "").strip()
        if not name:
            messagebox.showwarning("提示", "请先填写地图名称")
            return

        def work():
            ok, output = save_map(self.ws_root, name)
            return {"ok": ok, "output": output, "name": name}

        self._run_async(work, self._after_save_map, f"保存地图 {name}")

    def _after_save_map(self, ok, payload):
        if not ok:
            self._log(f"[gui] 保存地图失败：{payload}")
            messagebox.showerror("保存失败", str(payload))
            return
        name = payload["name"]
        yaml_path = os.path.join(self.maps_dir, f"{name}.yaml")
        self.var_map_info.set(f"已保存：{name}.yaml\n{yaml_path}")
        self._log(f"[gui] 地图已保存：{yaml_path} / {name}.pgm")
        for line in str(payload["output"]).splitlines():
            self._log("    " + line)
        self.refresh_maps()
        self.var_map.set(f"{name}.yaml")
        messagebox.showinfo("保存成功", f"地图已保存：\n{yaml_path}")

    # ================= 巡检点 =================
    def refresh_waypoints(self):
        try:
            waypoints = self.store.list()
        except Exception as exc:
            self._log(f"[gui] 读取巡检点失败：{exc}")
            return
        self.tree.delete(*self.tree.get_children())
        for index, waypoint in enumerate(waypoints):
            label = chr(ord("A") + index) if index < 26 else str(index + 1)
            self.tree.insert("", "end", iid=waypoint.name, values=(
                label, waypoint.name, f"{waypoint.x:.2f}", f"{waypoint.y:.2f}",
                f"{waypoint.yaw:.2f}", f"{waypoint.dwell:.1f}",
                "是" if waypoint.home else ""))
        self.var_count.set(f"已记录航点：{len(waypoints)}")
        self.var_wp_path.set(f"文件：{self.waypoints_path}")
        self._log(f"[gui] 已加载 {len(waypoints)} 个巡检点：{[w.name for w in waypoints]}")

    def selected_waypoint(self):
        selection = self.tree.selection()
        return selection[0] if selection else None

    def on_record_waypoint(self):
        name = self.var_wp_name.get()
        if not name.strip():
            messagebox.showwarning("提示", "请先输入巡检点名称（例如 entrance）")
            return
        if self.bridge is None:
            messagebox.showerror("不可用", "ROS 未连接，无法读取机器人位姿")
            return
        pose = self.bridge.pose()
        if pose is None:
            detail = self.bridge.tf_error() or "TF map->base_footprint 尚未就绪"
            messagebox.showwarning(
                "无法记录",
                "取不到 map 坐标系下的机器人位姿。\n"
                "请确认已【开始建图】或【启动导航】，并且定位已就绪。\n\n"
                f"细节：{detail}")
            return
        x, y, yaw = pose
        is_home = bool(self.var_home.get())

        def work():
            self.store.upsert(name, x, y, yaw, dwell=4.0, home=is_home)
            return {"name": name, "x": x, "y": y, "yaw": yaw}

        self._run_async(work, self._after_record, f"记录巡检点 {name}")

    def _after_record(self, ok, payload):
        if not ok:
            self._log(f"[gui] 记录巡检点失败：{payload}")
            messagebox.showerror("记录失败", str(payload))
            return
        data = payload
        self._log(f"[gui] 已记录 {data['name']}：x={data['x']:.3f} y={data['y']:.3f} "
                  f"yaw={data['yaw']:.3f}")
        self.var_wp_name.set("")
        self.var_home.set(False)
        self.refresh_waypoints()

    def on_delete_waypoint(self):
        name = self.selected_waypoint()
        if not name:
            messagebox.showwarning("提示", "请先在右侧列表里选中一个巡检点")
            return
        if not messagebox.askyesno("确认删除", f"确定删除巡检点 {name} 吗？"):
            return
        self._run_async(lambda: self.store.remove(name),
                        lambda ok, payload: self._after_delete(name, ok, payload),
                        f"删除巡检点 {name}")

    def _after_delete(self, name, ok, payload):
        if not ok:
            self._log(f"[gui] 删除失败：{payload}")
            messagebox.showerror("删除失败", str(payload))
            return
        self._log(f"[gui] 已删除巡检点 {name}")
        self.refresh_waypoints()

    # ================= 巡检控制 =================
    def on_patrol(self, action):
        if self.bridge is None:
            messagebox.showerror("不可用", "ROS 未连接，无法调用巡检服务")
            return
        if not self.store.list() and action == "start":
            messagebox.showwarning("提示", "还没有记录任何巡检点，请先记录 A/B/C…")
            return

        def work():
            return self.bridge.call_patrol(action)

        self._run_async(work, lambda ok, payload: self._after_patrol(action, ok, payload),
                        f"巡检动作 {action}")

    def _after_patrol(self, action, ok, payload):
        if not ok:
            self._log(f"[gui] /patrol/{action} 调用失败：{payload}")
            messagebox.showwarning("巡检服务", str(payload))
            return
        success, message = payload
        self._log(f"[gui] /patrol/{action} -> success={success} message={message!r}")
        if not success:
            messagebox.showinfo("巡检服务", f"{action}: {message}")

    # ================= 状态刷新 =================
    def _tick_status(self):
        if self.bridge is not None:
            pose = self.bridge.pose()
            if pose is None:
                self.var_pose.set("x=--  y=--  yaw=--（等待定位/TF）")
            else:
                self.var_pose.set(f"x={pose[0]:.2f}  y={pose[1]:.2f}  yaw={pose[2]:.2f}")
            self.var_patrol.set(f"巡检状态：{self.bridge.status()}")
            goal = self.bridge.goal()
            self.var_goal.set(f"当前目标点：{goal or '--'}")
        else:
            self.var_pose.set("x=--  y=--  yaw=--（ROS 未连接）")
        self.root.after(500, self._tick_status)

    # ================= 退出清理 =================
    def on_close(self):
        if not messagebox.askokcancel("退出", "退出前会停止本工具启动的建图/导航，继续？"):
            return
        self._log("[gui] 正在退出并清理本工具启动的进程 ...")
        try:
            self.launcher.stop_all()
        except Exception:
            self._log("[gui] 停止 launch 出错：\n" + traceback.format_exc())
        # 兜底：只清本 domain 的链路进程；绝不触碰其它 domain，也不会杀掉终端。
        try:
            from .cleanup import cleanup_domain
            cleanup_domain(self.log)
        except Exception:
            self._log("[gui] domain 兜底清理出错：\n" + traceback.format_exc())
        if self.bridge is not None:
            try:
                self.bridge.shutdown()
            except Exception:
                pass
        self.root.destroy()
