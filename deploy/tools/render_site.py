#!/usr/bin/env python3
"""由站点目录里的 site.yaml 生成站点专用参数，并做落地校验。

生成物（都在站点目录内，属生成文件，勿手工编辑）：
  nav2_params.yaml  以仓库默认 nav2_params.yaml 为模板（保留注释），写入 AMCL 初始位姿等站点差异
  site.env          run_site.sh 会 source 的站点级环境变量（巡检任务参数、网卡等）

用法：
  python3 render_site.py --site-dir /opt/go2/sites/office [--ws-root /opt/go2/current] [--name office]
  python3 render_site.py --site-dir <站点源目录> --check-only    # 只校验配置，不写任何文件
"""
from __future__ import annotations

import argparse
import datetime
import math
import os
import re
import shlex
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


def patch_goal_tolerance(text: str, value: float) -> str:
    """把 patrol.arrival_tolerance 落到 Nav2 goal_checker.xy_goal_tolerance。

    到点判定实际由 Nav2 的 goal_checker 执行（patrol_manager 只做复核并告警），
    所以站点里的 arrival_tolerance 必须写进参数文件才算生效，否则就是个死配置。
    """
    pattern = re.compile(r"^(\s*xy_goal_tolerance:\s*)([-+]?[0-9]*\.?[0-9]+)(.*)$")
    out, count = [], 0
    for line in text.splitlines():
        match = pattern.match(line)
        if match:
            line = f"{match.group(1)}{value}{match.group(3)}"
            count += 1
        out.append(line)
    if count == 0:
        warn("模板里没有 xy_goal_tolerance，arrival_tolerance 只用于 patrol_manager 的复核告警")
    else:
        ok(f"Nav2 到点容差 xy_goal_tolerance 覆盖为 {value}（{count} 处）")
    return "\n".join(out) + "\n"


def validate_generated_params(path: str, initial: dict) -> None:
    """确认生成的参数文件本身可用：能解析、初始位姿真的写进去了。"""
    try:
        with open(path, encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        fail(f"生成的 {os.path.basename(path)} 无法解析: {exc}")
        return
    if not isinstance(data, dict):
        fail(f"生成的 {os.path.basename(path)} 结构异常（顶层不是映射）")
        return
    amcl = (data.get("amcl") or {}).get("ros__parameters") or {}
    if not amcl:
        fail("生成的参数文件里没有 amcl.ros__parameters")
        return
    if amcl.get("set_initial_pose") is not True:
        fail("生成的参数里 amcl.set_initial_pose 不是 true，开机自启不会自动定位")
    pose = amcl.get("initial_pose") or {}
    for key in ("x", "y", "yaw"):
        if key not in pose:
            fail(f"生成的参数里缺少 amcl.initial_pose.{key}")
            continue
        if abs(float(pose[key]) - float(initial.get(key, 0.0))) > 1e-6:
            fail(
                f"生成的 amcl.initial_pose.{key}={pose[key]} 与 site.yaml 的 "
                f"{float(initial.get(key, 0.0))} 不一致"
            )
    if not FAILS:
        ok(f"生成的参数已复核：set_initial_pose=true, initial_pose=({pose.get('x')}, {pose.get('y')}, {pose.get('yaw')})")


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
# site.yaml 的 safety: 段 → site.env 的环境变量名（driver.launch.py 用
# EnvironmentVariable 读同名的 GO2_*，run_site.sh 以 set -a 导出）。
SAFETY_KEYS = (
    ("max_linear", "GO2_MAX_LINEAR"),
    ("max_angular", "GO2_MAX_ANGULAR"),
    ("front_half_angle_deg", "GO2_FRONT_HALF_ANGLE_DEG"),
    ("emergency_distance", "GO2_EMERGENCY_DISTANCE"),
    ("stop_distance", "GO2_STOP_DISTANCE"),
    ("slow_distance", "GO2_SLOW_DISTANCE"),
    ("cmd_vel_timeout", "GO2_CMD_VEL_TIMEOUT"),
)

DEFAULT_SAFETY_NOTE = "前进 0.7m/s、转向 1.0rad/s、急停 0.20m、停车 0.35m、减速 0.60m"

# 与 twist_bridge.declare_parameter 的默认值保持一致（改这里要同步改那边）。
# 用途：site.yaml 只写其中一两个距离时，缺的那些要按默认值补进来一起校验，
# 否则"只把 slow_distance 调到 0.35"会与默认的 stop_distance=0.35 相等，
# 减速分支永远轮不到，而自检却报通过。
DRIVER_SAFETY_DEFAULTS = {
    "max_linear": 0.7,
    "max_angular": 1.0,
    "front_half_angle_deg": 30.0,
    "emergency_distance": 0.20,
    "stop_distance": 0.35,
    "slow_distance": 0.60,
    "cmd_vel_timeout": 0.5,
}

DISTANCE_KEYS = ("emergency_distance", "stop_distance", "slow_distance")


def validate_sections(cfg: dict) -> None:
    """site.yaml 的每个缩进块必须是映射，否则给出可读的拒绝信息。

    写成 `patrol: 3` 这类错法时，后面 `patrol.get(...)` 会在生成中途抛
    AttributeError，现场看到的是 Python 回溯、且**可能已经写完一半生成物**。
    """
    for section in ("initial_pose", "patrol", "safety", "env"):
        value = cfg.get(section)
        if value is not None and not isinstance(value, dict):
            fail(f"site.yaml 的 {section}: 段必须是映射（key: value 的缩进块），实际是 {type(value).__name__}")


def as_float(raw):
    """把站点里的数值写成 float；非数字或非有限值（nan/inf）返回 None。

    为什么要专门挡 nan：`float('nan')` 解析成功，而 nan 与任何数比较都为假——
    `nan <= 0`、`0.35 <= nan` 全是假，于是"必须大于 0""三个距离必须递增"
    这些检查会**逐条放行**，自检报通过、阈值却写进 site.env；到了 twist_bridge，
    safety.py 里 `minimum <= emergency_distance` 同样恒假，前置急停/停车/减速
    全部静默失效。inf 也一并挡掉（只当 low 端时递增检查会漏）。
    """
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def validate_safety(cfg: dict) -> None:
    """站点安全阈值自检。

    这几个数是给真机挡障碍的：顺序写反（比如急停距离大于停车距离）会让
    减速/急停永远轮不到，等于把保护关掉了，所以这里直接判失败、拒绝落地。
    只写一部分时按驱动默认值补全后再比，看的是**实际生效**的那一组数。
    """
    safety = cfg.get("safety") or {}
    env_block = cfg.get("env") or {}

    if not isinstance(safety, dict):
        # 写成 `safety: 0.7` / `safety: [0.2, 0.35]` 这种：下面 set(safety) 会直接
        # 抛 TypeError，现场看到的是 Python 回溯而不是"哪一行写错了"。
        fail("site.yaml 的 safety: 段必须是映射（key: value 的缩进块），不能是数字/列表")
        return
    if not isinstance(env_block, dict):
        fail("site.yaml 的 env: 段必须是映射（key: value 的缩进块）")
        return

    # 同名键在两处定义会互相覆盖（site.env 里后写者胜），现场改 site.yaml 却不见效
    collided = [
        env_name
        for _, env_name in SAFETY_KEYS
        if env_name in env_block
    ]
    if collided:
        fail(
            "safety: 段与 env: 段重复定义了同一个安全阈值变量 "
            f"（{', '.join(collided)}）：只保留 safety: 段里的那份，"
            "否则 site.env 里哪个生效取决于写入顺序"
        )
        fail(
            "如果只想临时覆盖，请用命令行环境变量（例如 "
            "GO2_MAX_LINEAR=0.4 deploy/run_site.sh <站点>），不要写进 env: 段"
        )

    unknown = sorted(set(safety) - {key for key, _ in SAFETY_KEYS})
    if unknown:
        fail(
            f"site.yaml 的 safety: 段有认不出来的键: {', '.join(unknown)}"
            f"（可用键: {', '.join(key for key, _ in SAFETY_KEYS)}）——拼错的阈值不会生效"
        )

    if not safety:
        if collided:
            return      # 已经判失败，别再打一行"用驱动默认值"把人绕晕
        ok(f"未覆盖安全阈值，用驱动默认值（{DEFAULT_SAFETY_NOTE}）")
        return

    bad = bool(collided) or bool(unknown)
    values: dict[str, float] = {}
    for key, env_name in SAFETY_KEYS:
        raw = safety.get(key)
        if raw is None or raw == "":
            continue
        value = as_float(raw)
        if value is None:
            fail(f"site.yaml 的 safety.{key} 不是有限数字: {raw!r}（对应 {env_name}）")
            bad = True
            continue
        values[key] = value

    for key in ("max_linear", "max_angular", "cmd_vel_timeout", "front_half_angle_deg"):
        if key in values and values[key] <= 0:
            fail(f"safety.{key}={values[key]} 必须大于 0")
            bad = True

    # 负的距离会让 safety.py 的比较永远不成立（minimum <= -1 恒假），等于把急停关掉
    effective = {key: values.get(key, DRIVER_SAFETY_DEFAULTS[key]) for key in DISTANCE_KEYS}
    for key, value in effective.items():
        if value <= 0:
            fail(f"safety.{key}={value} 必须大于 0（负值/零会让对应的保护永不触发）")
            bad = True
    for low_key, high_key in zip(DISTANCE_KEYS, DISTANCE_KEYS[1:]):
        if effective[high_key] <= effective[low_key]:
            fail(
                f"生效值 safety.{low_key}={effective[low_key]} 不小于 "
                f"safety.{high_key}={effective[high_key]}：三个距离必须递增"
                "（急停 < 停车 < 减速；没写的那项按驱动默认值参与比较）"
            )
            bad = True

    if not bad:
        overridden = ", ".join(f"{key}={values[key]}" for key, _ in SAFETY_KEYS if key in values)
        ok(f"安全阈值覆盖: {overridden}")
        ok(
            "实际生效: "
            + ", ".join(f"{key}={effective[key]}" for key in DISTANCE_KEYS)
        )


def write_site_env(site_dir: str, name: str, cfg: dict) -> str:
    lines = [
        f"# 由 deploy/tools/render_site.py 生成（站点 {name}），deploy/run_site.sh 会 source 本文件",
        "# 勿手工编辑：改 site.yaml 后重新执行 deploy/apply_site.sh",
        "# 巡检与安全阈值的写法是 VAR=${VAR:-值}：命令行/环境里显式给的同名变量优先",
        "# （现场临时降速验证可以用 GO2_MAX_LINEAR=0.4 deploy/run_site.sh <站点>）；",
        "# 网卡与狗地址是裸赋值：站点 site.yaml 优先于机器级 /etc/go2/site.env",
    ]
    if cfg.get("iface"):
        lines.append(f"GO2_IFACE={shlex.quote(str(cfg['iface']))}")
    if cfg.get("dog_ip"):
        lines.append(f"GO2_DOG_IP={shlex.quote(str(cfg['dog_ip']))}")
    if cfg.get("rviz") is not None:
        lines.append(f"GO2_RVIZ={'true' if cfg['rviz'] else 'false'}")

    patrol = cfg.get("patrol") or {}
    for key, env_name in (
        ("loops", "GO2_PATROL_LOOPS"),
        ("return_home", "GO2_PATROL_RETURN_HOME"),
        ("battery_low", "GO2_PATROL_BATTERY_LOW"),
        ("arrival_tolerance", "GO2_PATROL_ARRIVAL_TOLERANCE"),
        ("turn_speech", "GO2_PATROL_TURN_SPEECH"),
        ("turn_speech_w", "GO2_PATROL_TURN_SPEECH_W"),
        ("turn_speech_min_interval", "GO2_PATROL_TURN_SPEECH_MIN_INTERVAL"),
        ("speech_lead_distance", "GO2_PATROL_SPEECH_LEAD_DISTANCE"),
        ("dwell_override", "GO2_PATROL_DWELL_OVERRIDE"),
        ("yaw_mode", "GO2_PATROL_YAW_MODE"),
        ("goal_retry_limit", "GO2_PATROL_GOAL_RETRY_LIMIT"),
        ("goal_retry_delay", "GO2_PATROL_GOAL_RETRY_DELAY"),
        ("on_goal_failure", "GO2_PATROL_ON_GOAL_FAILURE"),
    ):
        if key in patrol and patrol[key] is not None:
            value = patrol[key]
            if isinstance(value, bool):
                value = "true" if value else "false"
            lines.append(f"{env_name}=${{{env_name}:-{shlex.quote(str(value))}}}")

    # env: 段先写、safety 后写：两者若定义了同名键，validate_safety 已经判失败，
    # 这里让 safety 胜出只是保证"万一漏过也以 safety: 为准"的确定性。
    env_block = cfg.get("env")
    for key, value in (env_block if isinstance(env_block, dict) else {}).items():
        if value is None or value == "":
            # 空值写进去会变成 `GO2_X=`：环境变量"存在但为空串"时
            # EnvironmentVariable 不会退回默认值，数值参数会直接解析失败
            continue
        lines.append(f"{key}={shlex.quote(str(value))}")

    safety = cfg.get("safety") or {}
    if isinstance(safety, dict):
        for key, env_name in SAFETY_KEYS:
            # 非数字/非有限值（nan、inf）不写：validate_safety 已经判过失败，
            # 这里再兜一层，免得 site.env 里留下一个把保护悄悄关掉的阈值。
            value = as_float(safety.get(key))
            if value is None:
                continue
            lines.append(f"{env_name}=${{{env_name}:-{value}}}")

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
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="只校验站点资产与安全阈值，不生成/覆盖任何文件（apply_site.sh 落地前预检用）",
    )
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

    # 安全阈值先校验：不合法就**什么都不写**。参数文件是生成物，拿一份坏配置盖掉
    # 上一次能跑的那份，服务重启时才会因为参数读不出来而崩，现场看到的根因完全不搭。
    validate_sections(cfg)
    validate_safety(cfg)

    if args.check_only:
        # 只校验、不落地：apply_site.sh 用它先查**源目录**的 site.yaml 是否可用，
        # 通过了才往站点目录里拷——否则"拷完才发现阈值写反"会把已上线站点的
        # site.yaml 换成坏的那份，而参数文件还是旧的，目录进入半成品状态。
        # 这里只查配置段（safety:/env:/patrol:/initial_pose:）与安全阈值，
        # 不查地图/点位是否存在：源目录缺 waypoints.yaml 是允许的（apply_site 会用
        # 仓库示例填充），拿站点资产的完整性去卡预检会误伤这条正常路径。
        print(f"[render_site] 预检（只读）站点={name} 目录={site_dir}")
        print(f"[render_site] 预检结果: 失败 {len(FAILS)} 项，警告 {len(WARNINGS)} 项（未写入任何文件）")
        if FAILS:
            print("[render_site] 站点配置未通过预检，请改好 site.yaml 后重跑", file=sys.stderr)
            return 1
        return 0

    if FAILS:
        print("[render_site] 安全阈值不合法，未写入任何参数文件，请改好 site.yaml 后重跑", file=sys.stderr)
        return 1

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

    patrol = cfg.get("patrol") or {}
    if patrol.get("arrival_tolerance") is not None:
        text = patch_goal_tolerance(text, float(patrol["arrival_tolerance"]))

    params_path = os.path.join(site_dir, "nav2_params.yaml")
    with open(params_path, "w", encoding="utf-8") as handle:
        handle.write(banner(name, template) + text)
    ok(f"已生成 {params_path}")
    validate_generated_params(params_path, initial)

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
