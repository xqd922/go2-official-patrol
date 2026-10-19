"""地图画布：把 ROS 地图画到 tkinter Canvas，并支持鼠标点击定点。

职责划分：
  * map_model.py  负责「像素 <-> map 坐标」这类纯数学；
  * 本模块负责「画出来」和「点回去」，即 图片像素 <-> Canvas 屏幕像素。

两者之间只差一个缩放比例 scale 和居中偏移 offset：
    screen_x = offset_x + col * scale
    screen_y = offset_y + row * scale
因此点击屏幕 -> 反推图片像素 -> 交给 map_model 转成 map 坐标。

缩放的关键：**只允许等比缩放**。若 x/y 用不同比例，地图会被拉变形，点击
定位就会系统性偏移。
"""
import tkinter as tk

from .map_model import MapError, load_map

# 地图渲染配色（浅色，不抢眼，保证黑色障碍物清晰）
COLOR_FREE = "#f7f7f5"
COLOR_WALL = "#2b2b2b"
COLOR_UNKNOWN = "#c9c9c4"
GREY_BY_CLASS = None  # 由 _grey_for 惰性生成

ROBOT_COLOR = "#1565c0"          # 机器人本体（蓝）
ROBOT_HEADING = "#e53935"        # 朝向指示（红）
WAYPOINT_COLOR = "#2e7d32"       # 普通航点（绿）
WAYPOINT_HOME = "#ef6c00"        # home 航点（橙）
WAYPOINT_SELECTED = "#d81b60"    # 选中航点（洋红）
PENDING_COLOR = "#8e24aa"        # 待确认的临时点（紫）


class MapView:
    """把 OccupancyGrid 渲染到 Canvas；接收点击并回报 map 坐标。"""

    def __init__(self, canvas, on_click=None, logger=print):
        self.canvas = canvas
        self.on_click = on_click
        self.log = logger
        self.grid = None
        self.scale = 1.0
        self.offset = (0.0, 0.0)
        self._photo = None
        self._pending = None          # 待确认点的 map 坐标 (x, y)
        self._waypoints = []          # [(name, x, y, yaw, home)]
        self._selected = None
        self._pose = None             # 机器人 (x, y, yaw)
        self.canvas.bind("<Button-1>", self._on_button)

    # ---------- 加载 ----------
    def load(self, yaml_path):
        """加载地图并首次绘制。失败抛 MapError（调用方负责提示）。"""
        self.grid = load_map(yaml_path)
        self.log(f"[map] 已加载 {self.grid.name} "
                 f"{self.grid.width}x{self.grid.height}px "
                 f"res={self.grid.resolution} origin=({self.grid.origin_x:.2f},"
                 f"{self.grid.origin_y:.2f})")
        self.redraw()
        return self.grid

    def has_map(self):
        return self.grid is not None

    # ---------- 布局计算 ----------
    def _compute_layout(self):
        """按当前 Canvas 尺寸做等比缩放并居中，返回 (scale, off_x, off_y)。"""
        grid = self.grid
        canvas_w = max(int(self.canvas.winfo_width()), 1)
        canvas_h = max(int(self.canvas.winfo_height()), 1)
        if canvas_w <= 1 or canvas_h <= 1:      # 尚未布局完成
            canvas_w, canvas_h = 700, 560
        margin = 12
        avail_w = max(canvas_w - 2 * margin, 1)
        avail_h = max(canvas_h - 2 * margin, 1)
        # 等比缩放：取两个方向里更小的比例，保证整张地图都放得下
        scale = min(avail_w / grid.width, avail_h / grid.height)
        drawn_w = grid.width * scale
        drawn_h = grid.height * scale
        off_x = margin + (avail_w - drawn_w) / 2.0
        off_y = margin + (avail_h - drawn_h) / 2.0
        return scale, off_x, off_y

    # 屏幕 <-> 图片像素
    def screen_to_pixel(self, sx, sy):
        return ((sx - self.offset[0]) / self.scale,
                (sy - self.offset[1]) / self.scale)

    def pixel_to_screen(self, col, row):
        return (self.offset[0] + col * self.scale,
                self.offset[1] + row * self.scale)

    def map_to_screen(self, x, y):
        """map 坐标 -> Canvas 屏幕坐标。"""
        col, row = self.grid.map_to_pixel(x, y)
        return self.pixel_to_screen(col, row)

    def screen_to_map(self, sx, sy):
        """Canvas 屏幕坐标 -> map 坐标。"""
        col, row = self.screen_to_pixel(sx, sy)
        return self.grid.pixel_to_map(col, row)

    # ---------- 绘制 ----------
    def redraw(self):
        """重画整张图（地图位图 + 机器人 + 航点 + 临时点）。"""
        self.canvas.delete("all")
        if self.grid is None:
            self.canvas.create_text(
                self.canvas.winfo_width() // 2 or 350, 40,
                text="请选择一张地图（例如 offline_room.yaml）",
                fill="#777777")
            return
        self.scale, off_x, off_y = self._compute_layout()
        self.offset = (off_x, off_y)
        self._draw_bitmap()
        self._draw_waypoints()
        self._draw_pending()
        self._draw_robot()

    def _draw_bitmap(self):
        """把灰度像素转成 RGB 位图；缩放优先交给 PhotoImage（最近邻）。

        无 display 时（单测/CI）PhotoImage 不可用，直接退回逐像素绘制，
        避免 Pillow 在析构时报出无意义的警告。
        """
        grid = self.grid
        try:
            from PIL import Image, ImageTk
        except ImportError:
            self._draw_bitmap_manual()
            return
        if not _display_available():
            self._draw_bitmap_manual()
            return

        # 每个像素按占用分类映射到颜色，再让 PIL 做缩放。
        palette = {
            "free": (247, 247, 245),
            "unknown": (201, 201, 196),
            "occupied": (43, 43, 43),
        }
        rgb = bytearray()
        for value in grid.pixels:
            r, g, b = palette[grid.classify(value)]
            rgb += bytes((r, g, b))
        image = Image.frombytes("RGB", (grid.width, grid.height), bytes(rgb))
        target_w = max(int(round(grid.width * self.scale)), 1)
        target_h = max(int(round(grid.height * self.scale)), 1)
        if (target_w, target_h) != (grid.width, grid.height):
            image = image.resize((target_w, target_h), Image.NEAREST)
        try:
            self._photo = ImageTk.PhotoImage(image)
            self.canvas.create_image(self.offset[0], self.offset[1], anchor="nw",
                                     image=self._photo)
            return
        except Exception:
            # ImageTk 需要真实的 Tk 解释器；在没有 display 的环境（单测/CI）
            # 会失败。此时退回逐像素绘制，保证渲染逻辑本身仍被完整执行。
            self._photo = None
            self._draw_bitmap_manual()

    def _draw_bitmap_manual(self):
        """无 Pillow 时的兜底：逐像素画矩形（大地图会慢，但保证能看）。"""
        grid = self.grid
        for row in range(grid.height):
            for col in range(grid.width):
                value = grid.pixels[row * grid.width + col]
                x0, y0 = self.pixel_to_screen(col - 0.5, row - 0.5)
                x1, y1 = self.pixel_to_screen(col + 0.5, row + 0.5)
                self.canvas.create_rectangle(
                    x0, y0, x1, y1, outline="", width=0,
                    fill={"free": COLOR_FREE, "unknown": COLOR_UNKNOWN,
                          "occupied": COLOR_WALL}[grid.classify(value)])

    def _draw_robot(self):
        if self._pose is None:
            return
        x, y, yaw = self._pose
        if not self.grid.contains_map(x, y, margin=1.0):
            return
        sx, sy = self.map_to_screen(x, y)
        r = 9
        # 朝向：map 系 yaw 逆时针为正，但屏幕 y 向下、且地图行号已翻转，
        # 所以屏幕上的角度 = -yaw。
        import math
        hx = sx + r * 1.9 * math.cos(-yaw)
        hy = sy + r * 1.9 * math.sin(-yaw)
        self.canvas.create_line(sx, sy, hx, hy, fill=ROBOT_HEADING, width=3,
                                arrow="last")
        self.canvas.create_oval(sx - r, sy - r, sx + r, sy + r,
                                fill=ROBOT_COLOR, outline="white", width=2)
        self.canvas.create_text(sx, sy, text="R", fill="white")
        self.canvas.create_text(sx, sy - r - 10, text="机器人",
                                fill=ROBOT_COLOR, font=("TkDefaultFont", 8, "bold"))

    def _draw_waypoints(self):
        for index, (name, x, y, yaw, home) in enumerate(self._waypoints):
            if not self.grid.contains_map(x, y, margin=1.0):
                continue
            sx, sy = self.map_to_screen(x, y)
            label = chr(ord("A") + index) if index < 26 else str(index + 1)
            color = WAYPOINT_HOME if home else WAYPOINT_COLOR
            selected = (name == self._selected)
            if selected:
                color = WAYPOINT_SELECTED
            r = 11 if selected else 9
            self.canvas.create_oval(sx - r, sy - r, sx + r, sy + r,
                                    fill=color, outline="white",
                                    width=3 if selected else 1)
            self.canvas.create_text(sx, sy, text=label, fill="white",
                                    font=("TkDefaultFont", 8, "bold"))
            # 名称标签带浅色底，避免压在黑色墙上看不清
            text = self.canvas.create_text(sx + r + 3, sy - r - 3, text=name,
                                           anchor="w", fill="#103010",
                                           font=("TkDefaultFont", 8))
            box = self.canvas.bbox(text)
            if box:
                self.canvas.create_rectangle(box[0] - 2, box[1] - 1,
                                             box[2] + 2, box[3] + 1,
                                             fill="#ffffff", outline="")
                self.canvas.tag_raise(text)

    def _draw_pending(self):
        if self._pending is None:
            return
        x, y = self._pending
        sx, sy = self.map_to_screen(x, y)
        r = 7
        self.canvas.create_oval(sx - r, sy - r, sx + r, sy + r,
                                outline=PENDING_COLOR, width=2)
        self.canvas.create_line(sx - r, sy, sx + r, sy, fill=PENDING_COLOR)
        self.canvas.create_line(sx, sy - r, sx, sy + r, fill=PENDING_COLOR)
        self.canvas.create_text(sx, sy - r - 9,
                                text=f"待添加 ({x:.2f}, {y:.2f})",
                                fill=PENDING_COLOR,
                                font=("TkDefaultFont", 8, "bold"))

    # ---------- 数据更新 ----------
    def set_pose(self, pose):
        """pose = (x, y, yaw) 或 None（不可用）。"""
        self._pose = pose
        self.redraw()

    def set_waypoints(self, waypoints):
        """waypoints = [(name, x, y, yaw, home), ...]，按列表顺序编号 A/B/C…"""
        self._waypoints = list(waypoints)
        self.redraw()

    def set_selected(self, name):
        self._selected = name
        self.redraw()

    def set_pending(self, xy):
        self._pending = xy
        self.redraw()

    def clear_pending(self):
        self._pending = None

    def pending(self):
        return self._pending

    # ---------- 鼠标 ----------
    def _on_button(self, event):
        if self.grid is None:
            return
        x, y = self.screen_to_map(event.x, event.y)
        # 点到地图外面时忽略，避免误建离谱航点
        if not self.grid.contains_map(x, y):
            self.log("[map] 点击位置在地图范围外，已忽略")
            return
        self._pending = (x, y)
        self.redraw()
        if self.on_click is not None:
            self.on_click(x, y)

def _display_available():
    """当前进程是否连得上 X display（决定能否用 ImageTk.PhotoImage）。"""
    import tkinter
    try:
        root = tkinter._default_root
        if root is not None:
            return True
        probe = tkinter.Tk()
        probe.destroy()
        return True
    except Exception:
        return False
