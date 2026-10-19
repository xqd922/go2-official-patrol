#!/usr/bin/env python3
import os
import shutil
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from go2_patrol_patrol.speech_policy import DROP, QUEUE, decide, pending_is_fresh


class VoicePlayer(Node):
    """Play pre-recorded waypoint speech reliably without a camera or TTS.

    占用中的请求进单槽等待位、播完补播（策略见 speech_policy.py）：
    原来"正在播 / 冷却期内直接丢"会让转弯提醒和到点语音互吞，
    而离线验收只断言"收到过播报"，这种漏播测不出来。
    """

    def __init__(self):
        super().__init__("voice_player")
        self.declare_parameter("audio_dir", os.path.join(os.path.dirname(__file__), "..", "audio"))
        self.declare_parameter("default_text", "到达巡检点")
        # 等待位里那句最多等多久；超过就丢掉并留日志（晚播的到点语音会让人以为在说上一个点）
        self.declare_parameter("pending_ttl", 2.0)
        self.audio_dir = os.path.abspath(self.get_parameter("audio_dir").value)
        self.default_text = self.get_parameter("default_text").value
        self.pending_ttl = float(self.get_parameter("pending_ttl").value)
        self.create_subscription(String, "/robot/say", self.on_say, 10)
        self.busy = False
        self.playing = None
        self.pending = None
        self.pending_at = None
        self.cooldown_until = 0
        self.lock = threading.Lock()
        # 等待位兜底（见 drain_tick）：只靠播放结束触发补播会漏掉"冷却期内排进来"的那句
        self.create_timer(0.2, self.drain_tick)

    # ---------------------------------------------------------------- 播放器选择
    def resolve_audio(self, speech_id: str):
        path = os.path.join(self.audio_dir, f"{speech_id}.wav")
        return path if os.path.isfile(path) else None

    def player_command(self, path: str):
        if shutil.which("aplay"):
            return ["aplay", "-q", path]
        if shutil.which("afplay"):
            return ["afplay", path]
        if shutil.which("ffplay"):
            return ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", path]
        return self.windows_fallback_command(path)

    def windows_fallback_command(self, path: str):
        """WSL 里没有任何音频后端时，借 Windows 的扬声器播（WSLg 实测最稳）。

        WSL 默认不带 aplay/ffplay，之前到点只打一行 [VOICE] 日志，谁都听不见；
        这里把 wav 复制到 Windows 临时目录再用 PowerShell 播。真机（非 WSL）上
        powershell.exe 不存在，自动退回"只打日志"，不影响生产链路。
        """
        if not shutil.which("powershell.exe"):
            return None
        try:
            probe = subprocess.run(
                ["cmd.exe", "/c", "echo", "%TEMP%"],
                capture_output=True,
                timeout=5.0,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        # cmd.exe 按 Windows 本地代码页输出（中文系统是 GBK），不能用 text=True 按
        # UTF-8 解码，否则直接抛 UnicodeDecodeError 把节点搞崩；忽略坏字节即可。
        win_tmp = probe.stdout.decode("utf-8", errors="ignore").strip()
        if len(win_tmp) < 3 or win_tmp[1] != ":":
            return None
        drive = win_tmp[0].lower()
        rest = win_tmp[2:].strip("\\").replace("\\", "/")
        host_dir = f"/mnt/{drive}/{rest}"
        if not os.path.isdir(host_dir):
            return None
        name = f"go2_voice_{os.path.basename(path)}"
        try:
            shutil.copyfile(path, os.path.join(host_dir, name))
        except OSError:
            return None
        win_file = f"{drive.upper()}:\\{rest.replace('/', chr(92))}\\{name}"
        script = f"(New-Object Media.SoundPlayer '{win_file}').PlaySync()"
        return ["powershell.exe", "-NoProfile", "-Command", script]

    # ---------------------------------------------------------------- 调度
    def in_cooldown(self) -> bool:
        return self.get_clock().now().nanoseconds < self.cooldown_until

    def on_say(self, message: String):
        speech_id = message.data.strip()
        action = decide(speech_id, self.playing, self.busy, self.in_cooldown())
        if action == DROP:
            if speech_id:
                self.get_logger().debug(f"忽略 {speech_id}（正在播这一句）")
            return
        if action == QUEUE:
            with self.lock:
                replaced = self.pending if self.pending not in (None, speech_id) else None
                self.pending = speech_id
                self.pending_at = self.get_clock().now().nanoseconds
            if replaced:
                self.get_logger().warn(
                    f"播报排队：{replaced} 被 {speech_id} 顶掉（等待位只有一格）"
                )
            else:
                self.get_logger().info(f"播报排队：{speech_id}（当前在播 {self.playing or '冷却中'}）")
            return
        self.start_playback(speech_id)

    def start_playback(self, speech_id: str):
        now = self.get_clock().now().nanoseconds
        audio = self.resolve_audio(speech_id)
        if audio:
            command = self.player_command(audio)
        else:
            command = None
            self.get_logger().warn(f"audio not found for '{speech_id}'; check /robot/say in logs")

        if not command:
            self.get_logger().info(f"[VOICE] {speech_id or self.default_text}")
            self.cooldown_until = now + 2_000_000_000
            return

        self.busy = True
        self.playing = speech_id
        threading.Thread(target=self.play_blocking, args=(command,), daemon=True).start()

    def play_blocking(self, command):
        try:
            subprocess.run(command, check=False, timeout=20.0)
        finally:
            # 顺序要紧：**先**设冷却，再把 busy 放开。
            # busy/playing/cooldown_until 都是工作线程在 self.lock 之外写的，on_say 与
            # drain_tick 在 executor 线程并发读。若先清 busy，在"busy 已假、冷却还没设"
            # 的窗口里 decide() 会看到 busy=False + 冷却已过期 → 认为可以开播，
            # 于是另起一个播放线程，两路音频同时出声（drain_tick 命中也一样）。
            self.cooldown_until = self.get_clock().now().nanoseconds + 1_000_000_000
            self.playing = None
            self.busy = False
            self.drain_pending()

    def drain_pending(self):
        """播完一句后把等待位里那句补上；过期就丢弃，但一定留日志。"""
        with self.lock:
            speech_id = self.pending
            requested_at = self.pending_at
            self.pending = None
            self.pending_at = None
        if not speech_id:
            return
        if not pending_is_fresh(requested_at, self.get_clock().now().nanoseconds, self.pending_ttl):
            self.get_logger().warn(
                f"补播 {speech_id} 已过期（排队超过 {self.pending_ttl:.1f}s），丢弃；"
                "播报排队太密时检查 speech_lead_distance / turn_speech_min_interval"
            )
            return
        self.start_playback(speech_id)

    def drain_tick(self):
        """等待位兜底：只在"冷却中"排进来的那句，没有任何播放线程会来触发补播。

        原来只有 play_blocking 的 finally 调 drain_pending，于是"设备忙/刚播完还在冷却"
        期间进来的请求会一直滞留在等待位——等下一次别的播报播完才被取走，
        那时通常已超过 pending_ttl 而被丢弃：这正是本次改动要消除的那种静默漏播。
        """
        if self.busy or not self.pending or self.in_cooldown():
            return
        self.drain_pending()


def main(args=None):
    rclpy.init(args=args)
    node = VoicePlayer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
