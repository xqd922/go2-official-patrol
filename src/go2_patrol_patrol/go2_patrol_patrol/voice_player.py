#!/usr/bin/env python3
import os
import shutil
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoicePlayer(Node):
    """Play pre-recorded waypoint speech reliably without a camera or TTS."""

    def __init__(self):
        super().__init__("voice_player")
        self.declare_parameter("audio_dir", os.path.join(os.path.dirname(__file__), "..", "audio"))
        self.declare_parameter("default_text", "到达巡检点")
        self.audio_dir = os.path.abspath(self.get_parameter("audio_dir").value)
        self.default_text = self.get_parameter("default_text").value
        self.create_subscription(String, "/robot/say", self.on_say, 10)
        self.create_service_from_parameter = None
        self.busy = False
        self.cooldown_until = 0.0

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

    def on_say(self, message: String):
        speech_id = message.data.strip()
        now = self.get_clock().now().nanoseconds
        if self.busy or now < self.cooldown_until:
            return

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

        def play():
            try:
                subprocess.run(command, check=False, timeout=20.0)
            finally:
                self.cooldown_until = self.get_clock().now().nanoseconds + 1_000_000_000
                self.busy = False

        threading.Thread(target=play, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = VoicePlayer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
