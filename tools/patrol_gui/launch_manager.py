"""GUI 启动/停止项目现有的 ros2 launch（建图 / 导航）。

**不重写任何 launch**：建图用 `mapping.launch.py`，导航用 `patrol.launch.py`
（真机链路，AMCL + driver + patrol_manager + voice）。GUI 只负责把它们跑在
独立进程组里，并在需要时整组收掉。

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
        os.makedirs(self.log_dir, exist_ok=True)

    # ---------- 查询 ----------
    def is_running(self, mode):
        popen = self._procs.get(mode)
        return popen is not None and popen.poll() is None

    def running_modes(self):
        return [m for m in (MAPPING, NAVIGATION) if self.is_running(m)]

    def log_path(self, mode):
        return self._logs.get(mode)

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
        self.log(f"[gui] 已启动 {mode}（pid={popen.pid} pgid={pgid}），日志：{log_path}")
        return pgid

    def start_mapping(self):
        """启动项目现有 mapping.launch.py（SLAM 建图）。"""
        return self._spawn(
            MAPPING,
            ["ros2", "launch", "go2_patrol_bringup", "mapping.launch.py"],
        )

    def start_navigation(self, map_path):
        """用指定地图启动项目现有 patrol.launch.py（REAL：AMCL + 真机 driver）。"""
        if not map_path:
            raise RuntimeError("启动导航需要先选择一张地图")
        if not os.path.isfile(map_path):
            raise RuntimeError(f"地图文件不存在：{map_path}")
        return self._spawn(
            NAVIGATION,
            ["ros2", "launch", "go2_patrol_bringup", "patrol.launch.py",
             f"map:={map_path}"],
        )

    # ---------- 停止 ----------
    def stop(self, mode, grace=15.0):
        """整组收掉某个 launch（SIGTERM -> 有限等待 -> SIGKILL）。"""
        pgid = self._groups.pop(mode, None)
        popen = self._procs.pop(mode, None)
        self._logs.pop(mode, None)
        if pgid is None and popen is None:
            return False
        self.log(f"[gui] 正在停止 {mode}（pgid={pgid}）")
        proc.terminate_group(pgid, grace=grace, log=self.log)
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
