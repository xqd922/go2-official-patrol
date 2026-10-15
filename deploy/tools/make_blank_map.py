#!/usr/bin/env python3
"""生成一份空白站点骨架（空房间地图 + 示例点位 + 站点参数），用于台架自检。

用途：没有真机/没建图时，也能把 systemd → run_site.sh → launch → Nav2 这条链路跑通。

用法：
  python3 make_blank_map.py <输出目录> [--name demo] [--size 10] [--resolution 0.05]
  然后：deploy/apply_site.sh --site-dir <输出目录> --name demo
"""
from __future__ import annotations

import argparse
import os


def write_pgm(path: str, width: int, height: int) -> None:
    """白色空地 + 1 像素黑色墙，构成一个封闭房间。"""
    row_free = bytes([254]) * (width - 2)
    wall = bytes([0]) * width
    with open(path, "wb") as handle:
        handle.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        handle.write(wall + b"\n")
        for _ in range(height - 2):
            handle.write(bytes([0]) + row_free + bytes([0]) + b"\n")
        handle.write(wall + b"\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="生成空白站点骨架")
    parser.add_argument("out_dir", help="输出目录（不存在则创建）")
    parser.add_argument("--name", default=None, help="站点名，默认取输出目录名")
    parser.add_argument("--size", type=float, default=10.0, help="房间边长（米），默认 10")
    parser.add_argument("--resolution", type=float, default=0.05, help="分辨率（米/像素），默认 0.05")
    args = parser.parse_args()

    out_dir = os.path.abspath(args.out_dir)
    name = args.name or os.path.basename(out_dir.rstrip(os.sep))
    width = height = max(20, int(round(args.size / args.resolution)))
    half = width * args.resolution / 2.0

    os.makedirs(os.path.join(out_dir, "audio"), exist_ok=True)
    write_pgm(os.path.join(out_dir, "map.pgm"), width, height)

    with open(os.path.join(out_dir, "map.yaml"), "w", encoding="utf-8") as handle:
        handle.write(
            "image: map.pgm\n"
            "mode: trinary\n"
            f"resolution: {args.resolution}\n"
            f"origin: [{-half:.3f}, {-half:.3f}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.25\n"
        )

    with open(os.path.join(out_dir, "site.yaml"), "w", encoding="utf-8") as handle:
        handle.write(
            f"# 站点参数（台架自检用骨架，现场交付请按实际标定）\n"
            f"name: {name}\n"
            "iface: \"\"              # 留空=自动取默认路由网卡；生产环境填接机器狗的网卡名\n"
            "dog_ip: 192.168.123.161\n"
            "initial_pose:          # 机器人每次上电的固定起点（map 坐标系，米/弧度）\n"
            "  x: 0.0\n"
            "  y: 0.0\n"
            "  yaw: 0.0\n"
            "robot_radius: 0.35\n"
            "rviz: false\n"
            "patrol:\n"
            "  loops: 1\n"
            "  return_home: true\n"
            "  battery_low: 20.0\n"
            "  arrival_tolerance: 0.30\n"
        )

    with open(os.path.join(out_dir, "waypoints.yaml"), "w", encoding="utf-8") as handle:
        handle.write(
            "# 台架空房间示例点位（房间中心为原点）\n"
            "waypoints:\n"
            "  - name: home\n"
            "    x: 0.0\n"
            "    y: 0.0\n"
            "    yaw: 0.0\n"
            "    dwell: 3.0\n"
            "    speech: home\n"
            "    home: true\n"
            "  - name: corner_a\n"
            "    x: 2.0\n"
            "    y: 2.0\n"
            "    yaw: 0.0\n"
            "    dwell: 3.0\n"
            "    speech: corner_a\n"
        )

    with open(os.path.join(out_dir, "audio", "README.md"), "w", encoding="utf-8") as handle:
        handle.write(
            "# 到点语音\n\n"
            "文件名 = waypoints.yaml 里 `speech` 字段 + `.wav`，例如 `corner_a.wav`。\n\n"
            "```bash\n"
            "# 用 ffmpeg 生成一段测试语音（TTS 结果转 wav）\n"
            "ffmpeg -i corner_a.mp3 -ar 48000 -ac 1 corner_a.wav\n"
            "```\n"
        )

    print(f"[make_blank_map] 已生成站点骨架: {out_dir}")
    print(f"[make_blank_map] 地图: {width}x{height} 像素，约 {args.size}m x {args.size}m")
    print(f"[make_blank_map] 下一步: deploy/apply_site.sh --site-dir {out_dir} --name {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
