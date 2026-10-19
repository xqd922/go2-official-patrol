"""GUI 启动/停止项目现有的 ros2 launch（建图 / 导航）。

**不重写任何 launch**，只按运行模式选对入口：

  * OFFLINE（run.sh --offline，环境由 setup_offline.sh 设定）
      建图 = mapping.launch.py
      导航 = **offline_patrol.launch.py**（fake_go2 + 静态 map->odom + Nav2，
             不起 AMCL、不碰 Go2 网卡）
  * REAL（run.sh，环境由 setup.sh 设定）
      建图 = mapping.launch.py
      导航 = patrol.launch.py（真机 driver + 真雷达 + AMCL）

历史坑：早期版本无论什么模式都起 patrol.launch.py，于是 --offline 下点
【启动导航】会去起 REAL 的 AMCL 链路 —— 没有 Go2 网卡、没有真雷达，
AMCL 拿不到 map->odom，RViz 里既看不到机器人也没有正确的 map 坐标系。
GUI 只负责把 launch 跑在独立进程组里，并在需要时整组收掉。

为什么用 subprocess + os.setsid 而不是 import launch：
  1. 每个 launch 独立进程组，退出时一次 killpg 带走全部派生节点；
  2. GUI 进程本身不背 launch 的 asyncio 事件循环，界面不会被 Nav2 日志卡住；
  3. 命令与用户在终端里手敲的完全一致，出问题可以直接看日志复现。

无 ROS 依赖，便于单测（测试里注入一个假的 Popen 工厂）。
"""
import os
import shlex
import subprocess
import time

from . import proc

MAPPING = "mapping"
NAVIGATION = "navigation"

# 运行模式：由环境变量决定，避免 GUI 自己猜。
# setup_offline.sh 会 export GO2_OFFLINE_MODE=1（见该脚本）；REAL 不设。
OFFLINE = "offline"
REAL = "real"


def current_mode(environ=None):
    """判断当前是 OFFLINE 还是 REAL。

    判据优先级（不依赖任何写死的路径或地图名）：
      1. GO2_OFFLINE_MODE=1            -> offline（setup_offline.sh 显式声明）
      2. GO2_OFFLINE_LOCALHOST_ONLY 有值 -> offline（同源脚本的另一个标志）
      3. 其余                           -> real
    """
    env = os.environ if environ is None else environ
    if str(env.get("GO2_OFFLINE_MODE", "")).strip() in ("1", "true", "True", "yes"):
        return OFFLINE
    if env.get("GO2_OFFLINE_LOCALHOST_ONLY") is not None:
        return OFFLINE
    return REAL


def navigation_launch_for(mode):
    """按模式返回导航用的 launch 文件名（都在 go2_patrol_bringup 包里）。"""
    return "offline_patrol.launch.py" if mode == OFFLINE else "patrol.launch.py"


def _describe_returncode(rc):
    """把 returncode 翻译成人话：负数表示被信号杀死。"""
    import signal as _signal
    if rc is None:
        return "仍在运行"
    if rc < 0:
        try:
            return f"被信号 {_signal.Signals(-rc).name} 终止"
        except ValueError:
            return f"被信号 {-rc} 终止"
    return f"正常退出码 {rc}"


class LaunchManager:
    """管理最多两个长跑 launch（建图 / 导航），互斥使用。"""

    def __init__(self, ws_root, log_dir=None, popen_factory=None, logger=print):
        self.ws_root = ws_root
        self.log_dir = log_dir or os.path.join(ws_root, "log", "patrol_gui")
        self._popen = popen_factory or subprocess.Popen
        self.log = logger
        self._groups = {}     # mode -> pgid
        self._procs = {}      # mode -> Popen
        self._logs = {}       # mode -> 日志路径
        # mode -> "user"（用户点了停止/关窗）或 None（未主动停止）。
        # 用来区分"用户主动停止"与"launch 意外退出"，后者必须显式报给界面。
        self._exit_reason = {}
        os.makedirs(self.log_dir, exist_ok=True)

    # ---------- 查询 ----------
    def is_running(self, mode):
        popen = self._procs.get(mode)
        return popen is not None and popen.poll() is None

    def running_modes(self):
        return [m for m in (MAPPING, NAVIGATION) if self.is_running(m)]

    def log_path(self, mode):
        return self._logs.get(mode)

    def unexpected_exit(self, mode):
        """若 launch 已退出且**不是**用户主动停止，返回可读说明；否则 None。

        真实事故：离线 launch 被整套结束，但 GUI 仍显示"导航已启动"，用户完全
        不知道 Nav2 已经没了。这里把"进程消失"这件事变成界面可见的告警。
        """
        popen = self._procs.get(mode)
        if popen is None:
            return None
        if self._exit_reason.get(mode) == "user":
            return None
        rc = popen.poll()
        if rc is None:
            return None
        return f"{mode} 进程意外退出，returncode={rc}（{_describe_returncode(rc)}）"

    # ---------- 启动 ----------
    def _spawn(self, mode, args, extra_env=None):
        """在独立会话里跑一条命令，返回 pgid。"""
        if self.is_running(mode):
            raise RuntimeError(f"{mode} 已经在运行")
        log_path = os.path.join(self.log_dir, f"{mode}.log")
        handle = open(log_path, "w", encoding="utf-8")
        env = dict(os.environ)
        if extra_env:
            env.update(extra_env)
        # start_new_session=True == 子进程里 os.setsid()：子进程成为新会话/组长，
        # 因此 killpg(pgid) 只会带走它派生的进程树，绝不波及 GUI 与终端。
        popen = self._popen(
            args,
            stdout=handle,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            env=env,
            start_new_session=True,
        )
        pgid = proc.wait_pgid_ready(popen.pid, timeout=5.0)
        if pgid is None:
            pgid = popen.pid  # 兜底：多数情况下 setsid 后 pgid == pid
        self._procs[mode] = popen
        self._groups[mode] = pgid
        self._logs[mode] = log_path
        self._exit_reason.pop(mode, None)
        self.log(f"[gui] 已启动 {mode}（pid={popen.pid} pgid={pgid}），日志：{log_path}")
        return pgid

    def start_mapping(self):
        """启动项目现有 mapping.launch.py（SLAM 建图）。"""
        return self._spawn(
            MAPPING,
            ["ros2", "launch", "go2_patrol_bringup", "mapping.launch.py"],
        )

    def start_navigation(self, map_path, mode=None, rviz=True):
        """用**指定地图**启动导航；按模式选 OFFLINE 或 REAL 的 launch。

        map_path 必须是 GUI 当前选中地图的绝对路径 —— 这是唯一事实来源，
        不允许任何地方退回默认地图（否则会出现"GUI 显示 offline_room、
        Nav2 实际加载别的图"）。
        """
        if not map_path:
            raise RuntimeError("启动导航需要先选择一张地图")
        map_path = os.path.abspath(map_path)
        if not os.path.isfile(map_path):
            raise RuntimeError(f"地图文件不存在：{map_path}")
        mode = mode or current_mode()
        launch_file = navigation_launch_for(mode)
        args = ["ros2", "launch", "go2_patrol_bringup", launch_file,
                f"map:={map_path}"]
        if mode == OFFLINE:
            # 离线链路自己带 demo.rviz；这里显式跟随 GUI 的开关。
            args.append(f"rviz:={'true' if rviz else 'false'}")
        # 一目了然的对照日志：GUI 选的是哪张、真正传给 Nav2 的是哪张。
        self.log(f"GUI_SELECTED_MAP={map_path}")
        self.log(f"NAV_MAP={map_path}")
        self.log(f"[gui] 导航模式={mode} launch={launch_file}")
        return self._spawn(NAVIGATION, args)

    # ---------- 停止 ----------
    def stop(self, mode, grace=15.0):
        """整组收掉某个 launch（SIGTERM -> 有限等待 -> SIGKILL）。"""
        pgid = self._groups.pop(mode, None)
        popen = self._procs.pop(mode, None)
        self._logs.pop(mode, None)
        if pgid is None and popen is None:
            return False
        self._exit_reason[mode] = "user"
        self.log(f"[gui] 正在停止 {mode}（pgid={pgid}，用户主动停止）")
        proc.terminate_group(pgid, grace=grace, log=self.log,
                             reason=f"LaunchManager.stop({mode}, user)")
        if popen is not None:
            try:
                popen.wait(timeout=5)
            except Exception:
                pass
        return True

    def stop_all(self, grace=15.0):
        for mode in list(self._groups) + [m for m in self._procs if m not in self._groups]:
            try:
                self.stop(mode, grace=grace)
            except Exception as exc:  # 清理阶段不允许再抛异常
                self.log(f"[gui] 停止 {mode} 出错：{exc}")


def save_map(ws_root, map_name, timeout=60.0, run=None, logger=print):
    """调用项目现有 scripts/save_map.sh 保存地图；返回 (ok, 输出文本)。

    只做"跑一次脚本"，不是长跑进程组，因此不需要 setsid。
    """
    runner = run or subprocess.run
    script = os.path.join(ws_root, "scripts", "save_map.sh")
    if not os.path.isfile(script):
        return False, f"找不到 {script}"
    cmd = ["bash", script, map_name]
    logger(f"[gui] 保存地图：{shlex.join(cmd)}")
    try:
        completed = runner(cmd, cwd=ws_root, capture_output=True, text=True,
                           timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, f"保存地图超时（>{timeout:.0f}s）"
    output = (completed.stdout or "") + (completed.stderr or "")
    return completed.returncode == 0, output.strip()


def map_files_exist(ws_root, map_name):
    """检查 save_map.sh 是否真的产出了非空的 <name>.yaml 与 <name>.pgm。

    真实机出现过的情况：map_saver 打印
        [ERROR] [map_saver]: Failed to spin map subscription
    但脚本仍以 0 退出，于是 GUI 显示“已保存”——典型的假成功。
    因此保存成功必须同时满足：脚本返回 0 **且** 两个文件确实存在且非空。
    返回 (ok, 说明)。
    """
    if not map_name:
        return False, "地图名为空"
    map_dir = os.path.join(ws_root, "src", "go2_patrol_bringup", "maps")
    missing = []
    empty = []
    for suffix in ("yaml", "pgm"):
        target = os.path.join(map_dir, f"{map_name}.{suffix}")
        if not os.path.isfile(target):
            missing.append(f"{map_name}.{suffix}")
        elif os.path.getsize(target) == 0:
            empty.append(f"{map_name}.{suffix}")
    if missing:
        return False, "未生成文件：" + "、".join(missing)
    if empty:
        return False, "文件为空：" + "、".join(empty)
    return True, "地图文件已确认存在"
