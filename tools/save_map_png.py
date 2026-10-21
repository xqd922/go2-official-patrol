#!/usr/bin/env python3
"""把 /map（nav_msgs/OccupancyGrid）存成 PNG，便于在没有 RViz 时检查建图质量。

用法：
    ros2 run ...                # 或在已 source 环境后
    python3 tools/save_map_png.py /tmp/map.png --scale 8

颜色约定与 RViz 一致：
    未知 = 灰(205)，空闲 = 白，占用 = 黑，中间值按灰度插值。
"""
import argparse
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def grid_to_image(grid: OccupancyGrid, scale: int, legend: bool = False) -> Image.Image:
    width, height = grid.info.width, grid.info.height
    data = np.array(grid.data, dtype=np.int16).reshape(height, width)

    # OccupancyGrid: -1 未知，0 空闲，100 占用
    gray = np.full(data.shape, 205, dtype=np.uint8)
    known = data >= 0
    gray[known] = (255 - data[known] * 255 // 100).astype(np.uint8)

    # 栅格原点在左下，图片行从上到下，需垂直翻转
    image = Image.fromarray(np.flipud(gray), mode="L")
    if scale > 1:
        image = image.resize(
            (width * scale, height * scale), resample=Image.NEAREST
        )
    image = image.convert("RGB")

    if legend:
        from PIL import ImageDraw

        bar_height = 34
        canvas = Image.new("RGB", (image.width, image.height + bar_height), "white")
        canvas.paste(image, (0, bar_height))
        draw = ImageDraw.Draw(canvas)
        # 用英文标签：PIL 默认位图字体只支持 latin-1，中文会直接抛 UnicodeEncodeError
        items = [("unknown", (205, 205, 205)), ("free", (255, 255, 255)), ("occupied", (0, 0, 0))]
        x = 6
        for label, color in items:
            draw.rectangle([x, 8, x + 16, 24], fill=color, outline=(90, 90, 90))
            draw.text((x + 22, 11), label, fill=(20, 20, 20))
            x += 22 + 7 * len(label) + 18
        return canvas
    return image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", nargs="?", default="/tmp/map.png")
    parser.add_argument("--topic", default="/map")
    parser.add_argument("--scale", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--legend", action="store_true", help="在顶部加颜色图例")
    args = parser.parse_args()

    rclpy.init()
    node = Node("save_map_png")
    received = {}

    # /map 是 latched（transient_local），用默认 QoS 收不到已经发布过的那一份
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    node.create_subscription(OccupancyGrid, args.topic, lambda m: received.setdefault("map", m), qos)

    deadline = time.time() + args.timeout
    while time.time() < deadline and "map" not in received:
        rclpy.spin_once(node, timeout_sec=0.2)

    if "map" not in received:
        print(f"超时：{args.timeout:.0f}s 内没有收到 {args.topic}")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    grid = received["map"]
    image = grid_to_image(grid, args.scale, legend=args.legend)
    image.save(args.output)

    info = grid.info
    cells = np.array(grid.data, dtype=np.int16)
    print(f"保存 {args.output}")
    print(f"尺寸 {info.width}x{info.height} @ {info.resolution:.3f} m/cell")
    print(f"原点 ({info.origin.position.x:.3f}, {info.origin.position.y:.3f})")
    print(f"占用 {int((cells == 100).sum())}，空闲 {int((cells == 0).sum())}，"
          f"未知 {int((cells < 0).sum())}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
