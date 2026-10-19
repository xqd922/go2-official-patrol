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

from .launch_manager import (MAPPING, NAVIGATION, LaunchManager,
                             current_mode, map_files_exist,
                             navigation_launch_for, save_map)
from .map_model import MapError
from .map_view import MapView
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
        self.map_view = None            # 地图画布（_build_widgets 里创建）
        # 导航启动校验的等待时长/轮询间隔：生产默认 60s（Nav2 + map_server
        # 冷启动确实可能几十秒），测试里注入小值以免拖慢用例。
        self.nav_verify_timeout = 60.0
        self.nav_verify_interval = 1.0
        # TF 只等一小段宽限：刚启动时 TF buffer 还没填满，查不到是正常的，
        # 不能据此永久判定"TF 链缺失"。
        self.tf_verify_grace = 8.0
        # 机器人位姿所在的坐标系（与 RViz marker / ros_bridge 保持一致）。
        self.map_frame = "map"
        self.base_frame = "base_footprint"
        # 导航校验状态：静态行 + 实时 TF 状态分离，TF 由后台持续刷新。
        self._nav_verify_active = False
        self._nav_verify_lines = []
        self._nav_tf_ok = False
        self._nav_tf_first_ok_logged = False
        self._pending_panel = []        # 右侧“待添加”区域控件（有临时点才显示）
        self._map_error = None          # 最近一次加载地图失败的原因

        self.root.title("Go2 建图与巡检点工具")
        self.root.geometry("1180x760")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self._build_widgets()
        self.refresh_maps()
        self.refresh_waypoints()
        self._poll_results()
        self._tick_status()

    # ================= 界面 =================
    def _build_widgets(self):
        outer = ttk.Frame(self.root, padding=8)
        outer.pack(fill="both", expand=True)

        # 顶部：状态条
        status = ttk.LabelFrame(outer, text="机器人状态（map 坐标系）", padding=6)
        status.pack(fill="x")
        self.var_pose = tk.StringVar(value="x=--  y=--  yaw=--")
        self.var_patrol = tk.StringVar(value="巡检状态：--")
        self.var_goal = tk.StringVar(value="当前目标点：--")
        self.var_count = tk.StringVar(value="已记录航点：0")
        ttk.Label(status, textvariable=self.var_pose,
                  font=("TkDefaultFont", 11, "bold")).grid(
            row=0, column=0, sticky="w", padx=(0, 20))
        ttk.Label(status, textvariable=self.var_patrol).grid(
            row=0, column=1, sticky="w", padx=(0, 20))
        ttk.Label(status, textvariable=self.var_goal).grid(
            row=0, column=2, sticky="w", padx=(0, 20))
        ttk.Label(status, textvariable=self.var_count).grid(row=0, column=3, sticky="w")
        self.var_hint = tk.StringVar(value="")
        ttk.Label(status, textvariable=self.var_hint, foreground="#a05000").grid(
            row=1, column=0, columnspan=4, sticky="w", pady=(4, 0))

        # 中部：左=地图，右=控制面板
        middle = ttk.Frame(outer)
        middle.pack(fill="both", expand=True, pady=8)

        # ---------- 左：地图 ----------
        mapframe = ttk.LabelFrame(middle, text="地图（左键点击 = 在该处添加巡检点）",
                                  padding=6)
        mapframe.pack(side="left", fill="both", expand=True)
        self.canvas = tk.Canvas(mapframe, background="#e8e8e4",
                                highlightthickness=0, width=720, height=520)
        self.canvas.pack(fill="both", expand=True)
        # 缩放/尺寸变化时重画
        self.canvas.bind("<Configure>", lambda _e: self._on_canvas_resize())
        self.map_view = MapView(self.canvas, on_click=self.on_map_click,
                                logger=self._log)
        self.var_map_legend = tk.StringVar(
            value="图例：R=机器人  A/B/C…=巡检点  紫点=待添加（点【添加地图巡检点】写入）")
        ttk.Label(mapframe, textvariable=self.var_map_legend,
                  foreground="#555555").pack(anchor="w", pady=(4, 0))

        # ---------- 右：控制面板 ----------
        right = ttk.Frame(middle, width=340)
        right.pack(side="left", fill="y", padx=(8, 0))
        right.pack_propagate(False)

        # 地图选择
        mapbox = ttk.LabelFrame(right, text="1. 选择地图", padding=6)
        mapbox.pack(fill="x")
        self.var_map = tk.StringVar()
        self.combo_map = ttk.Combobox(mapbox, textvariable=self.var_map, width=34,
                                      state="readonly")
        self.combo_map.pack(fill="x")
        self.combo_map.bind("<<ComboboxSelected>>", lambda _e: self.on_load_map())
        row = ttk.Frame(mapbox)
        row.pack(fill="x", pady=(4, 0))
        ttk.Button(row, text="加载并显示地图", command=self.on_load_map).pack(
            side="left", fill="x", expand=True)
        ttk.Button(row, text="刷新列表", command=self.refresh_maps).pack(
            side="left", padx=(4, 0))
        self.var_map_info = tk.StringVar(value="尚未加载地图")
        ttk.Label(mapbox, textvariable=self.var_map_info, wraplength=310,
                  foreground="#206020", justify="left").pack(anchor="w", pady=(4, 0))

        # 保存地图
        savebox = ttk.LabelFrame(right, text="保存地图", padding=6)
        savebox.pack(fill="x", pady=(6, 0))
        ttk.Label(savebox, text="保存为新地图名：").pack(anchor="w")
        self.var_map_name = tk.StringVar(value="office_floor")
        ttk.Entry(savebox, textvariable=self.var_map_name).pack(fill="x")
        ttk.Button(savebox, text="保存地图", command=self.on_save_map).pack(
            fill="x", pady=(4, 0))
        self.var_save_info = tk.StringVar(value="")
        ttk.Label(savebox, textvariable=self.var_save_info, wraplength=310,
                  justify="left").pack(anchor="w", pady=(4, 0))

        # 添加巡检点
        addbox = ttk.LabelFrame(right, text="2. 添加巡检点", padding=6)
        addbox.pack(fill="x", pady=(6, 0))
        self.var_pending = tk.StringVar(value="点击地图选择位置（或遥控后记录当前位置）")
        ttk.Label(addbox, textvariable=self.var_pending, wraplength=310,
                  foreground="#8e24aa", justify="left").pack(anchor="w")

        grid = ttk.Frame(addbox)
        grid.pack(fill="x", pady=(4, 0))
        ttk.Label(grid, text="名称：").grid(row=0, column=0, sticky="w")
        self.var_wp_name = tk.StringVar()
        entry = ttk.Entry(grid, textvariable=self.var_wp_name)
        entry.grid(row=0, column=1, sticky="ew", columnspan=3)
        grid.columnconfigure(1, weight=1)
        entry.bind("<Return>", lambda _e: self.on_add_map_waypoint())

        ttk.Label(grid, text="yaw：").grid(row=1, column=0, sticky="w", pady=(4, 0))
        # 普通巡检点默认"自动朝向下一点"；需要固定朝向时填数值（如 1.57）。
        self.var_yaw = tk.StringVar(value="auto")
        ttk.Entry(grid, textvariable=self.var_yaw, width=8).grid(
            row=1, column=1, sticky="w", pady=(4, 0))
        ttk.Label(grid, text="auto=朝下一点 / 1.57=固定",
                  foreground="#666666").grid(row=1, column=2, sticky="w",
                                             padx=(6, 0), pady=(4, 0))
        ttk.Label(grid, text="dwell：").grid(row=1, column=3, sticky="w",
                                             padx=(4, 0), pady=(4, 0))
        self.var_dwell = tk.StringVar(value="4.0")
        ttk.Entry(grid, textvariable=self.var_dwell, width=6).grid(
            row=1, column=4, sticky="w", pady=(4, 0))

        self.var_home = tk.BooleanVar(value=False)
        ttk.Checkbutton(grid, text="设为 home（起点/回充）",
                        variable=self.var_home).grid(row=2, column=0,
                                                     columnspan=4, sticky="w",
                                                     pady=(4, 0))

        ttk.Button(addbox, text="添加地图巡检点（用点击的 x/y）",
                   command=self.on_add_map_waypoint).pack(fill="x", pady=(6, 0))
        ttk.Button(addbox, text="记录机器人当前位置为巡检点",
                   command=self.on_record_waypoint).pack(fill="x", pady=(3, 0))
        ttk.Label(addbox,
                  text="两种方式写入同一个 waypoints.yaml：地图点击取地图坐标；"
                       "记录当前位置取 map->base_footprint 实时位姿。",
                  wraplength=310, foreground="#666666",
                  justify="left").pack(anchor="w", pady=(4, 0))

        # 巡检点列表 + 删除
        listbox = ttk.LabelFrame(right, text="3. 巡检点列表", padding=6)
        listbox.pack(fill="both", expand=True, pady=(6, 0))
        columns = ("idx", "name", "x", "y", "yaw", "orient")
        self.tree = ttk.Treeview(listbox, columns=columns, show="headings",
                                 height=8, selectmode="browse")
        headings = {"idx": "#", "name": "名称", "x": "x", "y": "y", "yaw": "yaw",
                    "orient": "朝向"}
        widths = {"idx": 28, "name": 110, "x": 52, "y": 52, "yaw": 52, "orient": 68}
        for col in columns:
            self.tree.heading(col, text=headings[col])
            self.tree.column(col, width=widths[col],
                             anchor="center" if col != "name" else "w")
        self.tree.pack(fill="both", expand=True)
        self.tree.bind("<<TreeviewSelect>>", lambda _e: self._on_tree_select())
        btns = ttk.Frame(listbox)
        btns.pack(fill="x", pady=(4, 0))
        ttk.Button(btns, text="刷新", command=self.refresh_waypoints).pack(
            side="left", fill="x", expand=True)
        ttk.Button(btns, text="删除选中", command=self.on_delete_waypoint).pack(
            side="left", fill="x", expand=True, padx=(4, 0))
        self.var_wp_path = tk.StringVar()
        ttk.Label(listbox, textvariable=self.var_wp_path, wraplength=310,
                  foreground="#555555").pack(anchor="w", pady=(4, 0))

        # ---------- 底部：建图/导航/巡检 ----------
        bottom = ttk.Frame(outer)
        bottom.pack(fill="x", pady=(6, 0))

        runbox = ttk.LabelFrame(bottom, text="建图与导航", padding=6)
        runbox.pack(side="left", fill="both", expand=True, padx=(0, 6))
        brow = ttk.Frame(runbox)
        brow.pack(fill="x")
        ttk.Button(brow, text="开始建图",
                   command=lambda: self.on_start_launch(MAPPING)).pack(
            side="left", fill="x", expand=True)
        ttk.Button(brow, text="启动导航",
                   command=lambda: self.on_start_launch(NAVIGATION)).pack(
            side="left", fill="x", expand=True, padx=(4, 0))
        ttk.Button(brow, text="停止建图/导航", command=self.on_stop_launch).pack(
            side="left", fill="x", expand=True, padx=(4, 0))
        ttk.Label(runbox,
                  text="建图=mapping.launch.py（SLAM）；导航=patrol.launch.py（REAL/AMCL）",
                  foreground="#555555").pack(anchor="w", pady=(4, 0))

        ctlbox = ttk.LabelFrame(bottom, text="巡检控制", padding=6)
        ctlbox.pack(side="left", fill="both", expand=True)
        crow = ttk.Frame(ctlbox)
        crow.pack(fill="x")
        for label, action in (("开始巡检", "start"), ("暂停巡检", "pause"),
                              ("继续巡检", "resume"), ("停止巡检", "stop")):
            ttk.Button(crow, text=label,
                       command=lambda a=action: self.on_patrol(a)).pack(
                side="left", fill="x", expand=True, padx=1)
        ttk.Label(ctlbox, text="复用现有 /patrol/{start,pause,resume,stop}",
                  foreground="#555555").pack(anchor="w", pady=(4, 0))

        # 日志
        logbox = ttk.LabelFrame(outer, text="日志", padding=4)
        logbox.pack(fill="both", pady=(6, 0))
        self.text_log = tk.Text(logbox, height=6, wrap="word")
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

    # ================= 地图显示 =================
    def _on_canvas_resize(self):
        """Canvas 尺寸变化时按新尺寸重画（保持等比缩放）。"""
        if self.map_view is not None and self.map_view.has_map():
            self.map_view.redraw()

    def on_load_map(self):
        """加载并显示选中的地图；失败给出明确原因，不静默。"""
        name = self.var_map.get()
        if not name:
            messagebox.showwarning("提示", "请先选择一张地图")
            return
        map_path = os.path.join(self.maps_dir, name)
        try:
            grid = self.map_view.load(map_path)
        except MapError as exc:
            self._map_error = str(exc)
            self.var_map_info.set(f"地图加载失败：{exc}")
            self._log(f"[gui] 地图加载失败：{exc}")
            messagebox.showerror("地图加载失败", str(exc))
            return
        except Exception as exc:                       # 兜底，避免界面崩
            self._map_error = str(exc)
            self.var_map_info.set(f"地图加载失败：{exc}")
            self._log(f"[gui] 地图加载异常：\n{traceback.format_exc()}")
            messagebox.showerror("地图加载失败", str(exc))
            return
        self._map_error = None
        width_m, height_m = grid.world_size()
        bounds = grid.bounds()
        self.var_map_info.set(
            f"已加载 {grid.name}\n"
            f"{grid.width}x{grid.height} px  res={grid.resolution} m/px\n"
            f"范围 x[{bounds[0]:.2f},{bounds[2]:.2f}] "
            f"y[{bounds[1]:.2f},{bounds[3]:.2f}] m\n"
            f"（{width_m:.2f} x {height_m:.2f} m）")
        # 地图换了，已有航点重新投影
        self.refresh_waypoints()
        self._log(f"[gui] 地图已显示：{name}")

    def on_map_click(self, x, y):
        """地图被点击：更新待添加点提示（真正写入要点【添加地图巡检点】）。"""
        self.var_pending.set(f"已选择位置：x={x:.3f}  y={y:.3f}（输入名称后点【添加地图巡检点】）")
        self.var_hint.set(f"地图点击位置 x={x:.2f} y={y:.2f}，请输入名称并添加")

    # ================= 添加 / 删除巡检点 =================
    def _parse_float(self, var, label, default):
        text = (var.get() or "").strip()
        if not text:
            return default
        try:
            return float(text)
        except ValueError:
            raise ValueError(f"{label} 必须是数字，当前为 {text!r}")

    def _parse_yaw(self):
        """解析 yaw 输入框 -> (raw_yaw, orientation_mode)。

        填 auto（默认）/空：自动朝向下一个巡检点；填数值：固定朝向。
        """
        text = (self.var_yaw.get() or "").strip()
        if not text or text.lower() == "auto":
            return 0.0, "auto_next"
        try:
            return float(text), "fixed"
        except ValueError:
            raise ValueError(f"yaw 必须是 auto 或数字，当前为 {text!r}")

    def on_add_map_waypoint(self):
        """用地图点击的坐标添加航点（方式 B）。"""
        point = self.map_view.pending() if self.map_view else None
        if point is None:
            messagebox.showwarning(
                "提示", "请先在地图上点击一个位置，再点【添加地图巡检点】")
            return
        name = self.var_wp_name.get()
        ok, error = self._validate_name(name)
        if not ok:
            messagebox.showwarning("提示", error)
            return
        try:
            yaw, orient = self._parse_yaw()
            dwell = self._parse_float(self.var_dwell, "dwell", 4.0)
        except ValueError as exc:
            messagebox.showwarning("提示", str(exc))
            return

        x, y = point
        is_home = bool(self.var_home.get())

        def work():
            self.store.upsert(name, x, y, yaw, dwell=dwell, home=is_home,
                              orientation_mode=orient)
            return {"name": name, "x": x, "y": y, "yaw": yaw,
                    "dwell": dwell, "orient": orient}

        self._run_async(work, self._after_add_map_waypoint, f"添加地图巡检点 {name}")

    def _validate_name(self, name):
        from .waypoint_store import validate_name
        return validate_name(name)

    def _after_add_map_waypoint(self, ok, payload):
        if not ok:
            self._log(f"[gui] 添加巡检点失败：{payload}")
            messagebox.showerror("添加失败", str(payload))
            return
        data = payload
        self._log(f"[gui] 已添加 {data['name']}：x={data['x']:.3f} "
                  f"y={data['y']:.3f} yaw={data['yaw']:.3f} dwell={data['dwell']}")
        self.var_wp_name.set("")
        self.var_home.set(False)
        if self.map_view:
            self.map_view.clear_pending()
        self.var_pending.set("点击地图选择位置（或遥控后记录当前位置）")
        self.refresh_waypoints()

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
            # 记下本次真正传出去的地图路径，导航起来后用它校验 /map 是否同一张。
            self._launch_map_path = os.path.abspath(map_path)
            work = lambda: self.launcher.start_navigation(
                map_path, mode=current_mode())
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
            return
        # 导航：不能只因为 launch 进程还活着就报"已启动"。这里明确说明用的
        # 是哪套 launch、哪张地图，并启动后台校验（/map + TF 链）。
        mode_name = current_mode()
        launch_file = navigation_launch_for(mode_name)
        selected = getattr(self, "_launch_map_path", None)
        self._log(f"GUI_SELECTED_MAP={selected}")
        self._log(f"NAV_MAP={selected}")
        self._log(f"[gui] 导航 launch={launch_file} 模式={mode_name}")
        self.var_hint.set(
            f"导航已启动（{mode_name}/{launch_file}）\n"
            f"地图={os.path.basename(selected) if selected else '?'}\n"
            f"正在校验 /map 与 TF 链 ...")
        self._verify_navigation_async()

    def _expected_map_size(self, yaml_path):
        """读选中地图的 (width, height, resolution)，用于和 /map 对照。"""
        if not yaml_path:
            return None
        try:
            from .map_model import load_map
            grid = load_map(yaml_path)
            return (grid.width, grid.height, round(grid.resolution, 4))
        except Exception as exc:
            self._log(f"[gui] 无法读取选中地图用于对照：{exc}")
            return None

    # ================= 导航启动校验 =================
    def _verify_navigation_async(self, timeout=None):
        """后台等 /map 就绪，并在**有限宽限期**内顺带看一眼 TF。

        硬性要求：不能因为 "ros2 launch 进程还活着" 就显示"导航已启动"。
        map_server 没加载成功时必须显示「导航启动失败：地图未加载」。

        重要（历史坑）：TF 不能在这里"检查一次失败就永久判定链路故障"。
        ROS2 的 TF buffer 需要几秒才填满，刚启动时 map->base_footprint 查不到
        属于正常现象。这里 TF 只等一段宽限（tf_grace），拿不到就如实返回
        tf_result（可能 None/未就绪），交给 _after_verify_navigation 显示
        "等待 TF"，并最终由 _update_tf_status() 的后台持续重试负责恢复。
        """
        if self.bridge is None:
            self.var_hint.set("导航进程已启动，但 ROS 未连接，无法校验 /map 与 TF。")
            return

        timeout = self.nav_verify_timeout if timeout is None else timeout
        interval = self.nav_verify_interval
        tf_grace = getattr(self, "tf_verify_grace", 8.0)

        def work():
            import time
            map_deadline = time.monotonic() + float(timeout)
            map_info = None
            tf_result = None
            # 第一步：等 /map 真正有数据（这才是硬性失败条件）。
            while True:
                map_info = self.bridge.map_info()
                if map_info and map_info["width"] > 0:
                    break
                if time.monotonic() >= map_deadline:
                    return {"map_info": None, "tf_result": None}
                time.sleep(interval)
            # 第二步：TF 只等一段**宽限**时间，拿不到不算失败。
            tf_deadline = time.monotonic() + float(tf_grace)
            while True:
                tf_result = self._check_tf_chain()
                if tf_result is not None and tf_result.ok:
                    break
                if time.monotonic() >= tf_deadline:
                    break
                time.sleep(interval)
            return {"map_info": map_info, "tf_result": tf_result}

        self._run_async(work, self._after_verify_navigation, "校验 /map 与 TF")

    def _check_tf_chain(self):
        """查 map->odom->base_footprint->base_link，缺哪段报哪段。"""
        try:
            from .tf_check import check_chain
        except Exception:
            return None
        buffer = getattr(self.bridge, "_buffer", None)
        if buffer is None:
            return None
        return check_chain(buffer)

    def _after_verify_navigation(self, ok, payload):
        if not ok:
            self._log(f"[gui] 导航校验异常：{payload}")
            self.var_hint.set(f"导航校验异常：{payload}")
            return
        info = payload.get("map_info")
        tf_result = payload.get("tf_result")
        selected = getattr(self, "_launch_map_path", None)
        selected_name = os.path.basename(selected) if selected else "?"

        # ---- 地图校验 ----
        if not info or not info.get("width"):
            self._log("[gui] 导航启动失败：地图未加载（60s 内 /map 没有数据）")
            self.var_hint.set(
                f"导航启动失败：地图未加载\n"
                f"选中地图={selected_name}\n"
                f"60 秒内未在 /map 上收到任何地图数据。\n"
                f"请查看日志：{self.launcher.log_path(NAVIGATION)}")
            messagebox.showerror(
                "导航启动失败",
                f"导航启动失败：地图未加载\n\n"
                f"选中地图：{selected}\n"
                f"/map 上 60 秒内没有收到地图数据，map_server 可能没起来。\n\n"
                f"日志：{self.launcher.log_path(NAVIGATION)}")
            return

        expected = self._expected_map_size(selected)
        actual = (info["width"], info["height"], round(info["resolution"], 4))
        self._log(f"[gui] /map 已加载：{info['width']}x{info['height']} "
                  f"res={info['resolution']} frame={info['frame_id']}")
        map_ok = True
        mismatch = ""
        if expected:
            if actual[:2] != expected[:2] or abs(actual[2] - expected[2]) > 1e-6:
                map_ok = False
                mismatch = (f"选中地图 {expected[0]}x{expected[1]} res={expected[2]}，"
                            f"但 /map 上是 {actual[0]}x{actual[1]} res={actual[2]}")
                self._log(f"[gui] 警告：/map 与选中地图尺寸不一致：{mismatch}")

        # ---- TF 校验 ----
        # 关键：TF 未就绪**不是**失败。TF buffer 刚启动时还没填满，查不到
        # map->base_footprint 是正常的。这里只如实显示"等待 TF"，绝不弹
        # 模态框、绝不永久写死"TF 链缺失"。真正的持续判定与自动恢复交给
        # _update_tf_status()（每 0.5s 调一次，一旦可解析就自动转正常）。
        tf_text = "TF 未检查"
        tf_ok = False
        if tf_result is not None:
            tf_text = tf_result.describe()
            tf_ok = tf_result.ok
            self._log(f"[gui] {tf_text}")
        if not tf_ok:
            tf_text = (f"等待 TF（{self.map_frame}->{self.base_frame}）…"
                       f"刚启动时 TF buffer 需要几秒填充，界面会自动重试")

        lines = [
            f"导航模式：{current_mode()}（{navigation_launch_for(current_mode())}）",
            f"GUI_SELECTED_MAP={selected}",
            f"NAV_MAP={selected}",
            f"/map：{info['width']}x{info['height']} res={info['resolution']} "
            f"frame={info['frame_id']}"
            + ("" if map_ok else f"  ⚠ {mismatch}"),
            tf_text,
            "RViz 中可看到：地图 + 机器人标记(/go2/pose_marker) + "
            "巡检点(/patrol/waypoint_markers)",
        ]
        # 记录导航校验的静态部分（地图/launch 信息），供后台 TF 监视器
        # 动态拼接"实时 TF 状态"行；不要把易变的 TF 状态固化进来。
        self._nav_verify_lines = [
            f"导航模式：{current_mode()}（{navigation_launch_for(current_mode())}）",
            f"GUI_SELECTED_MAP={selected}",
            f"NAV_MAP={selected}",
            f"/map：{info['width']}x{info['height']} res={info['resolution']} "
            f"frame={info['frame_id']}"
            + ("" if map_ok else f"  ⚠ {mismatch}"),
        ]
        self._nav_verify_active = True
        self._nav_tf_ok = bool(tf_ok)
        self.var_hint.set("\n".join(lines))

        if not map_ok:
            # 地图不一致仍要提醒（这是静态事实，不会自己恢复），但不阻塞导航。
            messagebox.showwarning(
                "地图不一致",
                f"/map 上的地图与 GUI 选中的地图不一致：\n\n{mismatch}")
        # 注意：TF 未就绪**不再**弹模态框、不再判定"链路故障"。

    def on_stop_launch(self):
        if not self.launcher.running_modes():
            self._log("[gui] 当前没有正在运行的建图/导航")
            return
        self._run_async(lambda: self.launcher.stop_all(),
                        lambda ok, payload: self._log("[gui] 已停止建图/导航"),
                        "停止建图/导航")
        # 停止后清掉导航校验状态，避免后台 TF 监视器继续刷新旧提示。
        self._nav_verify_active = False
        self._nav_verify_lines = []
        self.var_hint.set("")

    def on_save_map(self):
        name = (self.var_map_name.get() or "").strip()
        if not name:
            messagebox.showwarning("提示", "请先填写地图名称")
            return

        # 保存前先记下旧文件状态，便于事后判断"到底有没有真的写出新文件"
        def work():
            ok, output = save_map(self.ws_root, name)
            # 关键：脚本返回 0 不等于保存成功。真实机出现过
            # [ERROR] [map_saver]: Failed to spin map subscription
            # 但脚本仍以 0 退出，GUI 却打印"已保存" —— 假成功。
            # 因此这里必须再校验两个文件确实存在且非空。
            exists, detail = map_files_exist(self.ws_root, name)
            return {"ok": ok and exists, "script_ok": ok, "output": output,
                    "name": name, "exists": exists, "detail": detail}

        self._run_async(work, self._after_save_map, f"保存地图 {name}")

    def _after_save_map(self, ok, payload):
        """保存结果处理：只有脚本成功**且**文件确实存在才算成功。

        真实机出现过 [ERROR] [map_saver]: Failed to spin map subscription，
        脚本却仍以 0 退出，旧实现于是打印"地图已保存" —— 典型假成功。
        现在脚本返回码与文件存在性都通过才显示成功。
        """
        name = payload.get("name", "")
        script_ok = payload.get("script_ok", False)
        exists = payload.get("exists", False)
        detail = payload.get("detail", "")
        output = str(payload.get("output", ""))

        for line in output.splitlines():
            self._log("    " + line)

        if not ok:
            if script_ok and not exists:
                reason = f"脚本返回成功，但地图文件未生成或为空：{detail}"
            elif not script_ok:
                reason = "save_map.sh 返回失败"
                for line in reversed(output.splitlines()):
                    low = line.lower()
                    if "error" in low or "失败" in line or "fail" in low:
                        reason += f"：{line.strip()}"
                        break
            else:
                reason = str(payload)
            self.var_save_info.set(f"地图保存失败：{reason}")
            self._log(f"[gui] 地图保存失败：{reason}")
            messagebox.showerror("保存失败", f"地图保存失败\n\n{reason}")
            return

        yaml_path = os.path.join(self.maps_dir, f"{name}.yaml")
        self.var_save_info.set(
            f"地图保存成功：{name}.yaml / {name}.pgm\n{yaml_path}")
        self._log(f"[gui] 地图保存成功（已校验文件存在）：{yaml_path}")
        self.refresh_maps()
        self.var_map.set(f"{name}.yaml")
        self.on_load_map()
        messagebox.showinfo("保存成功", f"地图保存成功：\n{yaml_path}\n{name}.pgm")

    # ================= 巡检点 =================
    def refresh_waypoints(self):
        """重新读取 waypoints.yaml，同步右侧列表与地图标记。"""
        try:
            waypoints = self.store.list()
        except Exception as exc:
            self._log(f"[gui] 读取巡检点失败：{exc}")
            return
        # 保留当前选中项，刷新后不丢选择
        previous = self.selected_waypoint()
        self.tree.delete(*self.tree.get_children())
        for index, waypoint in enumerate(waypoints):
            label = chr(ord("A") + index) if index < 26 else str(index + 1)
            auto = getattr(waypoint, "auto_orient", False)
            yaw_text = "auto" if auto else f"{waypoint.yaw:.2f}"
            orient_text = "→下一点" if auto else "固定"
            self.tree.insert("", "end", iid=waypoint.name, values=(
                label, waypoint.name, f"{waypoint.x:.2f}", f"{waypoint.y:.2f}",
                yaw_text, orient_text))
        if previous and previous in self.tree.get_children():
            self.tree.selection_set(previous)
        # 同步地图标记（顺序即 A/B/C… 编号来源）
        if self.map_view is not None:
            self.map_view.set_waypoints([
                (w.name, w.x, w.y, w.yaw, w.home) for w in waypoints])
        self.var_count.set(f"已记录航点：{len(waypoints)}")
        self.var_wp_path.set(f"文件：{self.waypoints_path}")
        self._log(f"[gui] 已加载 {len(waypoints)} 个巡检点："
                  f"{[w.name for w in waypoints]}")

    def _on_tree_select(self):
        """列表选中 -> 地图对应点高亮。"""
        name = self.selected_waypoint()
        if self.map_view is not None:
            self.map_view.set_selected(name)
        if name:
            self.var_hint.set(f"已选中巡检点 {name}（地图上洋红色标记）")

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
        x, y, _current_yaw = pose
        is_home = bool(self.var_home.get())
        try:
            yaw, orient = self._parse_yaw()
        except ValueError as exc:
            messagebox.showwarning("提示", str(exc))
            return

        def work():
            self.store.upsert(name, x, y, yaw, dwell=4.0, home=is_home,
                              orientation_mode=orient)
            return {"name": name, "x": x, "y": y, "yaw": yaw, "orient": orient}

        self._run_async(work, self._after_record, f"记录巡检点 {name}")

    def _after_record(self, ok, payload):
        if not ok:
            self._log(f"[gui] 记录巡检点失败：{payload}")
            messagebox.showerror("记录失败", str(payload))
            return
        data = payload
        auto = data.get("orient") == "auto_next"
        yaw_text = "auto(朝下一点)" if auto else f"{data['yaw']:.3f}"
        self._log(f"[gui] 已记录 {data['name']}：x={data['x']:.3f} y={data['y']:.3f} "
                  f"yaw={yaw_text}")
        self.var_wp_name.set("")
        self.var_home.set(False)
        if self.map_view is not None:
            self.map_view.clear_pending()
        self.var_pending.set("点击地图选择位置（或遥控后记录当前位置）")
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
        # 删掉后地图上对应标记也要立即消失，并清掉高亮
        if self.map_view is not None and self.map_view._selected == name:
            self.map_view.set_selected(None)
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
    def _update_tf_status(self):
        """每帧刷新导航提示区里的实时 TF 状态，可自愈、不误判。

        设计要点（对齐用户要求）：
          * 不在启动时"检查一次失败就永久判定 TF 缺失"；
          * 只要 map->base_footprint 此刻可解析，就显示"TF 正常"；
          * 暂时不可用只显示"等待 TF"，绝不写成"链路故障"；
          * 从"等待"变为"正常"时在日志里打一条，便于复盘。
        """
        if not getattr(self, "_nav_verify_active", False):
            return
        # 若当前正显示"launch 意外退出"告警，别把它覆盖掉（那是更严重的状态）。
        if getattr(self, "_reported_exits", None):
            return
        if self.bridge is None:
            return
        pose = self.bridge.pose()          # 由 ros_bridge 后台持续刷新（TF 一好就非 None）
        tf_ok = pose is not None
        if tf_ok and not self._nav_tf_ok:
            self._log(f"[gui] TF 已就绪：{self.map_frame}->{self.base_frame} "
                      f"= x={pose[0]:.2f} y={pose[1]:.2f} yaw={pose[2]:.2f}")
        self._nav_tf_ok = tf_ok
        self._nav_tf_first_ok_logged = True

        tf_line = (
            f"TF：{self.map_frame}->{self.base_frame} 正常"
            f"（x={pose[0]:.2f} y={pose[1]:.2f} yaw={pose[2]:.2f}）"
            if tf_ok else
            f"TF：等待 {self.map_frame}->{self.base_frame} …（后台持续重试，不影响导航）"
        )
        self.var_hint.set("\n".join(list(self._nav_verify_lines) + [
            tf_line,
            "RViz 中可看到：地图 + 机器人标记(/go2/pose_marker) + "
            "巡检点(/patrol/waypoint_markers)",
        ]))

    def _check_launch_health(self):
        """每帧检查导航/建图 launch 是否意外退出；退出就明确报警，不再假装"已启动"。

        真实事故：离线 launch 被整套结束，GUI 却仍显示"导航已启动"。这里一旦
        发现 Popen 已结束且非用户主动停止，就把 returncode/信号写进日志与提示区。
        """
        reported = getattr(self, "_reported_exits", None)
        if reported is None:
            reported = self._reported_exits = set()
        for mode in (MAPPING, NAVIGATION):
            try:
                message = self.launcher.unexpected_exit(mode)
            except Exception:
                message = None
            if not message:
                # 重新起来后允许再次报警
                reported.discard(mode)
                continue
            if mode in reported:
                continue
            reported.add(mode)
            self._log(f"[gui] {message}")
            self.var_hint.set(f"⚠ {message}\n"
                              f"日志：{self.launcher.log_path(mode)}")

    def _tick_status(self):
        self._check_launch_health()
        self._update_tf_status()
        if self.bridge is not None:
            pose = self.bridge.pose()
            if pose is None:
                self.var_pose.set("x=--  y=--  yaw=--（等待定位/TF）")
            else:
                self.var_pose.set(f"x={pose[0]:.2f}  y={pose[1]:.2f}  yaw={pose[2]:.2f}")
            self.var_patrol.set(f"巡检状态：{self.bridge.status()}")
            goal = self.bridge.goal()
            self.var_goal.set(f"当前目标点：{goal or '--'}")
            # 地图上的机器人标记（拿不到 TF 时传 None，界面显示"不可用"而不是崩）
            self._update_robot_marker(pose)
        else:
            self.var_pose.set("x=--  y=--  yaw=--（ROS 未连接）")
            self._update_robot_marker(None)
        self.root.after(500, self._tick_status)

    def _update_robot_marker(self, pose):
        if self.map_view is None or not self.map_view.has_map():
            return
        # 只在位姿真的变化时重画，避免每 0.5s 全量重绘导致闪烁/卡顿
        if pose == getattr(self, "_last_marker_pose", object()):
            return
        self._last_marker_pose = pose
        self.map_view.set_pose(pose)

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
