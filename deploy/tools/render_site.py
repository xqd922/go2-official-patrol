#!/usr/bin/env python3
"""由站点目录里的 site.yaml 生成站点专用参数，并做落地校验。

生成物（都在站点目录内，属生成文件，勿手工编辑）：
  nav2_params.yaml  以仓库默认 nav2_params.yaml 为模板（保留注释），写入 AMCL 初始位姿等站点差异
  site.env          run_site.sh 会 source 的站点级环境变量（巡检任务参数、网卡等）

用法：
  python3 render_site.py --site-dir /opt/go2/sites/office [--ws-root /opt/go2/current] [--name office]
"""
from __future__ import annotations

import argparse
import datetime
import os
import re
import sys

import yaml

FAILS: list[str] = []
WARNINGS: list[str] = []


def ok(msg: str) -> None:
    print(f"  [ OK ] {msg}")


def warn(msg: str) -> None:
    WARNINGS.append(msg)
    print(f"  [WARN] {msg}")


def fail(msg: str) -> None:
    FAILS.append(msg)
    print(f"  [FAIL] {msg}")


# ---------------------------------------------------------------- 模板与生成
def find_template(ws_root: str) -> str:
    candidates = [
        os.path.join(ws_root, "src", "go2_patrol_bringup", "config", "nav2_params.yaml"),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            os.path.join(
                get_package_share_directory("go2_patrol_bringup"), "config", "nav2_params.yaml"
            )
        )
    except Exception:
        pass
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise SystemExit(
        f"[render_site] 找不到 nav2_params.yaml 模板（已查: {', '.join(candidates)}）"
    )


def amcl_block_range(lines: list[str], amcl_idx: int) -> int:
    """返回 amcl: 段的结束行号（下一个 0 缩进的非空行）。"""
    idx = amcl_idx + 1
    while idx < len(lines):
        line = lines[idx]
        if line.strip() and not line.startswith((" ", "\t")):
            break
        idx += 1
    return idx


def patch_amcl_initial_pose(text: str, initial: dict) -> str:
    lines = text.splitlines()
    amcl_idx = next((i for i, line in enumerate(lines) if line.rstrip() == "amcl:"), None)
    if amcl_idx is None:
        raise SystemExit("[render_site] 模板里找不到 amcl: 段")

    block_end = amcl_block_range(lines, amcl_idx)
    set_idx = next(
        (
            i
            for i in range(amcl_idx + 1, block_end)
            if re.match(r"^\s+set_initial_pose:\s*\S", lines[i])
        ),
        None,
    )
    if set_idx is None:
        raise SystemExit("[render_site] 模板缺少 amcl.set_initial_pose，无法写入初始位姿")

    indent = len(lines[set_idx]) - len(lines[set_idx].lstrip())
    pad = " " * indent
    lines[set_idx] = f"{pad}set_initial_pose: true"

    # 模板里若已有 initial_pose 块（含嵌套行），先删除，避免重复键
    old_idx = next(
        (
            i
            for i in range(set_idx + 1, block_end)
            if re.match(r"^\s+initial_pose\s*:", lines[i])
        ),
        None,
    )
    if old_idx is not None:
        base = len(lines[old_idx]) - len(lines[old_idx].lstrip())
        end = old_idx + 1
        while end < len(lines) and (
            not lines[end].strip()
            or (len(lines[end]) - len(lines[end].lstrip())) > base
        ):
            end += 1
        del lines[old_idx:end]

    block = [
        f"{pad}# 由站点 site.yaml 生成：机器人每次上电的固定起点（map 坐标系，米/弧度）",
        f"{pad}initial_pose:",
        f"{pad}  x: {float(initial.get('x', 0.0))}",
        f"{pad}  y: {float(initial.get('y', 0.0))}",
        f"{pad}  z: {float(initial.get('z', 0.0))}",
        f"{pad}  yaw: {float(initial.get('yaw', 0.0))}",
    ]
    lines[set_idx + 1 : set_idx + 1] = block
    return "\n".join(lines) + "\n"


def patch_robot_radius(text: str, value: float) -> str:
    pattern = re.compile(r"^(\s*robot_radius:\s*)([-+]?[0-9]*\.?[0-9]+)(.*)$")
    out, count = [], 0
    for line in text.splitlines():
        match = pattern.match(line)
        if match:
            line = f"{match.group(1)}{value}{match.group(3)}"
            count += 1
        out.append(line)
    if count == 0:
        warn("模板里没有 robot_radius，未写入该覆盖")
    else:
        ok(f"robot_radius 覆盖为 {value}（{count} 处）")
    return "\n".join(out) + "\n"


def banner(name: str, template: str) -> str:
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    return (
        "# ============================================================\n"
        f"# 本文件由 deploy/tools/render_site.py 生成（站点 {name}，{now}）\n"
        f"# 模板: {template}\n"
        "# 勿手工编辑：改 site.yaml 后重新执行 deploy/apply_site.sh\n"
        "# ============================================================\n"
    )


# ---------------------------------------------------------------- site.env
def write_site_env(site_dir: str, name: str, cfg: dict) -> str:
    lines = [
        f"# 由 deploy/tools/render_site.py 生成（站点 {name}），deploy/run_site.sh 会 source 本文件",
        "# 勿手工编辑：改 site.yaml 后重新执行 deploy/apply_site.sh",
    ]
    if cfg.get("iface"):
        lines.append(f"GO2_IFACE={cfg['iface']}")
    if cfg.get("dog_ip"):
        lines.append(f"GO2_DOG_IP={cfg['dog_ip']}")
    if cfg.get("rviz") is not None:
        lines.append(f"GO2_RVIZ={'true' if cfg['rviz'] else 'false'}")

    patrol = cfg.get("patrol") or {}
    for key, env_name in (
        ("loops", "GO2_PATROL_LOOPS"),
        ("return_home", "GO2_PATROL_RETURN_HOME"),
        ("battery_low", "GO2_PATROL_BATTERY_LOW"),
        ("arrival_tolerance", "GO2_PATROL_ARRIVAL_TOLERANCE"),
    ):
        if key in patrol and patrol[key] is not None:
            value = patrol[key]
            if isinstance(value, bool):
                value = "true" if value else "false"
            lines.append(f"{env_name}={value}")

    for key, value in (cfg.get("env") or {}).items():
        lines.append(f"{key}={value}")

    path = os.path.join(site_dir, "site.env")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")
    return path


# ---------------------------------------------------------------- 校验
def pgm_dimensions(path: str) -> tuple[int, int] | None:
    try:
        with open(path, "rb") as handle:
            if handle.readline().strip() not in (b"P5", b"P2"):
                return None
            dims: list[int] = []
            while len(dims) < 2:
                line = handle.readline()
                if not line:
                    return None
                if line.startswith(b"#"):
                    continue
                dims.extend(int(token) for token in line.split())
            return dims[0], dims[1]
    except (OSError, ValueError):
        return None


def validate_site(site_dir: str) -> None:
    print("[render_site] 站点校验")

    map_yaml = os.path.join(site_dir, "map.yaml")
    resolution = None
    if not os.path.isfile(map_yaml):
        fail(f"缺少 {map_yaml}")
    else:
        try:
            with open(map_yaml, encoding="utf-8") as handle:
                map_cfg = yaml.safe_load(handle) or {}
            ok("map.yaml 可解析")
            resolution = map_cfg.get("resolution")
            image = str(map_cfg.get("image", "map.pgm"))
            image_path = image if os.path.isabs(image) else os.path.join(site_dir, image)
            if not os.path.isfile(image_path):
                fail(f"map.yaml 指向的地图不存在: {image_path}")
            else:
                if image_path.endswith(".pgm"):
                    dims = pgm_dimensions(image_path)
                    if dims is None:
                        warn("地图 .pgm 头部无法解析（确认是 P5/P2 格式）")
                    else:
                        width, height = dims
                        size_note = ""
                        if resolution:
                            size_note = f"，约 {width * float(resolution):.1f}m x {height * float(resolution):.1f}m"
                        ok(f"地图: {width}x{height} 像素{size_note}")
                else:
                    ok(f"地图数据: {os.path.basename(image_path)}")
        except yaml.YAMLError as exc:
            fail(f"map.yaml 解析失败: {exc}")

    waypoints_path = os.path.join(site_dir, "waypoints.yaml")
    if not os.path.isfile(waypoints_path):
        fail(f"缺少 {waypoints_path}")
    else:
        try:
            with open(waypoints_path, encoding="utf-8") as handle:
                waypoints_cfg = yaml.safe_load(handle) or {}
            items = waypoints_cfg.get("waypoints") or []
            if not items:
                fail("waypoints.yaml 里没有 waypoints 列表")
            else:
                ok(f"巡检点位 {len(items)} 个: {', '.join(str(i.get('name', '?')) for i in items)}")
                homes = [i for i in items if i.get("home")]
                if not homes:
                    warn("没有 home: true 的点位，低电量返航/结束回原点会缺少目标")
                for index, item in enumerate(items):
                    for key in ("name", "x", "y"):
                        if key not in item:
                            fail(f"waypoints[{index}] 缺少字段 {key}")
        except yaml.YAMLError as exc:
            fail(f"waypoints.yaml 解析失败: {exc}")

    audio_dir = os.path.join(site_dir, "audio")
    if os.path.isdir(audio_dir):
        waves = sorted(f for f in os.listdir(audio_dir) if f.endswith(".wav"))
        if waves:
            ok(f"到点语音 {len(waves)} 个: {', '.join(waves[:6])}{' …' if len(waves) > 6 else ''}")
        else:
            warn("audio/ 里没有 .wav，到点只打印日志不发声")
    else:
        warn("没有 audio/ 目录，到点只打印日志不发声")


# ---------------------------------------------------------------- 主流程
def main() -> int:
    parser = argparse.ArgumentParser(description="生成站点 Nav2 参数与环境变量")
    parser.add_argument("--site-dir", required=True, help="站点目录")
    parser.add_argument("--name", default=None, help="站点名，默认取目录名")
    parser.add_argument("--ws-root", default=None, help="工作空间根（找模板），默认取本文件上两级")
    args = parser.parse_args()

    site_dir = os.path.abspath(args.site_dir)
    if not os.path.isdir(site_dir):
        raise SystemExit(f"[render_site] 站点目录不存在: {site_dir}")
    name = args.name or os.path.basename(site_dir.rstrip(os.sep))
    ws_root = (
        os.path.abspath(args.ws_root)
        if args.ws_root
        else os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    )

    site_yaml = os.path.join(site_dir, "site.yaml")
    if os.path.isfile(site_yaml):
        try:
            with open(site_yaml, encoding="utf-8") as handle:
                cfg = yaml.safe_load(handle) or {}
        except yaml.YAMLError as exc:
            raise SystemExit(f"[render_site] site.yaml 解析失败: {exc}")
    else:
        cfg = {}
        warn("没有 site.yaml：初始位姿按 (0,0,0) 写入，请标定固定起点后重跑 apply_site.sh")

    initial = cfg.get("initial_pose") or {}
    template = find_template(ws_root)
    print(f"[render_site] 站点={name} 目录={site_dir}")
    print(f"[render_site] 模板={template}")

    with open(template, encoding="utf-8") as handle:
        text = handle.read()
    text = patch_amcl_initial_pose(text, initial)
    ok(
        "AMCL 初始位姿: set_initial_pose=true "
        f"(x={float(initial.get('x', 0.0))}, y={float(initial.get('y', 0.0))}, yaw={float(initial.get('yaw', 0.0))})"
    )
    if cfg.get("robot_radius") is not None:
        text = patch_robot_radius(text, float(cfg["robot_radius"]))

    params_path = os.path.join(site_dir, "nav2_params.yaml")
    with open(params_path, "w", encoding="utf-8") as handle:
        handle.write(banner(name, template) + text)
    ok(f"已生成 {params_path}")

    env_path = write_site_env(site_dir, name, cfg)
    ok(f"已生成 {env_path}")

    validate_site(site_dir)

    print(f"[render_site] 结果: 失败 {len(FAILS)} 项，警告 {len(WARNINGS)} 项")
    if FAILS:
        print("[render_site] 站点资产不完整，请修好后重跑", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
