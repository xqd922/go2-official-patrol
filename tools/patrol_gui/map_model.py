"""ROS/Nav2 标准地图（.yaml + .pgm）的读取，以及「像素坐标 <-> map 坐标」换算。

为什么单独一个模块：坐标换算是整个地图定点功能里最容易出错、也最值得单测的
部分。GUI 只负责画图与鼠标事件，换算规则集中在这里。

ROS 地图约定（nav2_map_server / map_server 文档）：

  * resolution：米 / 像素。
  * origin: [origin_x, origin_y, origin_yaw]，是**地图左下角**在 map 坐标系
    里的位姿（不是左上角）。本模块只支持 origin_yaw == 0（Nav2 保存的地图
    几乎都是 0；非 0 时显式报错，而不是悄悄算错）。
  * 图像原点在左上角，而 map 原点在左下角 => 图像行号必须翻转：

        map_x = origin_x + (col + 0.5) * resolution
        map_y = origin_y + (height - 1 - row + 0.5) * resolution

    （+0.5 表示取像素中心，与 map_server 的取整约定一致。）

  * occupied_thresh / free_thresh：按像素颜色判定占用，
    占用概率 p = (255 - value) / 255.0（negate=0 时）；
    p > occupied_thresh -> 占用(黑)，p < free_thresh -> 空闲(白)，之间为未知。

本模块只依赖标准库 + Pillow（用于解码 PGM/PNG）：C 语言实现的 Pillow 解码
比手写 PGM 解析更稳，且能顺带支持 Nav2 导出的 PNG。
"""
import os

import yaml

DEFAULT_RESOLUTION = 0.05


class MapError(Exception):
    """地图读取/换算失败（路径不存在、yaml 字段缺失、origin_yaw 非 0 等）。"""


class OccupancyGrid:
    """一张已解码的 ROS 地图：像素 + yaml 元数据，附带坐标换算。"""

    def __init__(self, width, height, resolution, origin, pixels, *,
                 name="", path="", negate=0, occupied_thresh=0.65,
                 free_thresh=0.25):
        if width <= 0 or height <= 0:
            raise MapError(f"地图尺寸非正: {width}x{height}")
        if resolution <= 0:
            raise MapError(f"resolution 必须为正: {resolution}")
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        # origin = (x, y, yaw)，单位米 / 弧度
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.origin_yaw = float(origin[2]) if len(origin) > 2 else 0.0
        self.pixels = pixels          # 扁平序列，行优先；索引 = row*width + col
        self.name = name
        self.path = path
        self.negate = int(negate)
        self.occupied_thresh = float(occupied_thresh)
        self.free_thresh = float(free_thresh)

    # ---------- 坐标换算 ----------
    def pixel_to_map(self, col, row):
        """图像像素坐标 (col, row) -> map 坐标 (x, y)，单位米。

        取像素中心；row 需按左上->左下翻转（见模块头注释）。
        """
        x = self.origin_x + (float(col) + 0.5) * self.resolution
        y = self.origin_y + (self.height - 1 - float(row) + 0.5) * self.resolution
        return x, y

    def map_to_pixel(self, x, y):
        """map 坐标 (x, y) -> 图像像素坐标 (col, row)，返回浮点（便于画图）。

        与 pixel_to_map 互为逆运算（在像素中心意义上）。
        """
        col = (float(x) - self.origin_x) / self.resolution - 0.5
        row = (self.height - 1) - ((float(y) - self.origin_y) / self.resolution - 0.5)
        return col, row

    def contains_map(self, x, y, margin=0.0):
        """map 坐标是否落在地图范围内（margin 单位米，可放宽用于容差）。"""
        col, row = self.map_to_pixel(x, y)
        return (-margin <= col <= self.width - 1 + margin
                and -margin <= row <= self.height - 1 + margin)

    def world_size(self):
        """地图物理尺寸（米）：(宽, 高)。"""
        return self.width * self.resolution, self.height * self.resolution

    def bounds(self):
        """map 坐标系下的包围盒：(min_x, min_y, max_x, max_y)，单位米。"""
        return (self.origin_x, self.origin_y,
                self.origin_x + self.width * self.resolution,
                self.origin_y + self.height * self.resolution)

    # ---------- 像素 -> 占用分类 ----------
    def occupancy_probability(self, value):
        """像素灰度值 -> 占用概率 [0, 1]。"""
        if self.negate:
            return float(value) / 255.0
        return (255.0 - float(value)) / 255.0

    def classify(self, value):
        """返回 'occupied' / 'free' / 'unknown'。用于测试与调试。"""
        p = self.occupancy_probability(value)
        if p > self.occupied_thresh:
            return "occupied"
        if p < self.free_thresh:
            return "free"
        return "unknown"


def load_map(yaml_path):
    """读取 ROS 地图 yaml + 对应图像，返回 OccupancyGrid。

    相对路径的 image: 按 ROS 约定相对 yaml 所在目录解析。
    任何字段缺失/类型不对都抛 MapError（附上人类可读原因）。
    """
    yaml_path = os.path.abspath(yaml_path)
    if not os.path.isfile(yaml_path):
        raise MapError(f"地图 yaml 不存在：{yaml_path}")
    try:
        with open(yaml_path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except OSError as exc:
        raise MapError(f"读取地图 yaml 失败：{exc}") from exc
    except yaml.YAMLError as exc:
        raise MapError(f"地图 yaml 解析失败：{exc}") from exc

    if not isinstance(data, dict):
        raise MapError("地图 yaml 顶层不是字典")

    image_name = data.get("image")
    if not image_name:
        raise MapError("地图 yaml 缺少 image 字段")
    image_path = image_name
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_name)
    if not os.path.isfile(image_path):
        raise MapError(f"地图图像不存在：{image_path}")

    try:
        resolution = float(data.get("resolution", DEFAULT_RESOLUTION))
    except (TypeError, ValueError) as exc:
        raise MapError(f"resolution 不是数字：{data.get('resolution')!r}") from exc

    origin = data.get("origin", [0.0, 0.0, 0.0])
    if not isinstance(origin, (list, tuple)) or len(origin) < 2:
        raise MapError(f"origin 格式不对：{origin!r}")
    yaw = float(origin[2]) if len(origin) > 2 and origin[2] is not None else 0.0
    if abs(yaw) > 1e-9:
        # 显式拒绝而不是悄悄忽略：带旋转的 origin 会让所有换算偏移。
        raise MapError(
            f"暂不支持 origin yaw != 0 的地图（当前 {yaw}）。"
            "Nav2 保存的地图通常 yaw=0；若确有旋转，需要先重新保存。")

    width, height, pixels = _decode_image(image_path)

    return OccupancyGrid(
        width=width, height=height, resolution=resolution,
        origin=(float(origin[0]), float(origin[1]), yaw),
        pixels=pixels, name=os.path.basename(yaml_path), path=yaml_path,
        negate=data.get("negate", 0),
        occupied_thresh=data.get("occupied_thresh", 0.65),
        free_thresh=data.get("free_thresh", 0.25),
    )


def _decode_image(image_path):
    """解码 PGM/PNG -> (width, height, gray_bytes)。用 Pillow，失败则手写 PGM。"""
    try:
        from PIL import Image
    except ImportError:
        return _decode_pgm_manual(image_path)
    try:
        with Image.open(image_path) as image:
            image = image.convert("L")          # 灰度
            width, height = image.size
            return width, height, image.tobytes()
    except Exception as exc:  # Pillow 认不出就退回手写 PGM 解析
        try:
            return _decode_pgm_manual(image_path)
        except MapError:
            raise MapError(f"图像解码失败：{image_path}（{exc}）") from exc


def _decode_pgm_manual(image_path):
    """最小 PGM(P5) 解析，作为 Pillow 不可用时的兜底。"""
    try:
        with open(image_path, "rb") as stream:
            blob = stream.read()
    except OSError as exc:
        raise MapError(f"读取图像失败：{exc}") from exc

    if not blob.startswith(b"P5"):
        raise MapError(f"仅支持二进制 PGM(P5)：{image_path}")

    index = 2
    values = []
    while len(values) < 3:
        while index < len(blob) and blob[index:index + 1].isspace():
            index += 1
        if blob[index:index + 1] == b"#":       # 注释行
            while index < len(blob) and blob[index:index + 1] != b"\n":
                index += 1
            continue
        start = index
        while index < len(blob) and not blob[index:index + 1].isspace():
            index += 1
        if start == index:
            raise MapError("PGM 头部解析失败")
        values.append(int(blob[start:index]))
    index += 1                                   # 跳过分隔空白
    width, height, maxval = values
    if maxval > 255:
        raise MapError("仅支持 8 位 PGM（maxval<=255）")
    data = blob[index:index + width * height]
    if len(data) != width * height:
        raise MapError(f"PGM 数据长度不符：期望 {width * height}，实得 {len(data)}")
    return width, height, data
