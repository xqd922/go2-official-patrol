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
        return None

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
