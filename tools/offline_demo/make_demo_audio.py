#!/usr/bin/env python3
"""为离线演示站点生成到点语音（标准库合成提示音，不依赖 ffmpeg/TTS/网络）。

每个 speech id 用一段"可听出区别"的双音提示（不同音高/节奏），
便于在扬声器或耳机里确认"到点确实播音了、播的是哪个点"。

用法:
  python3 tools/offline_demo/make_demo_audio.py
  python3 tools/offline_demo/make_demo_audio.py --out-dir <站点>/audio --ids home zone_ne
"""
from __future__ import annotations

import argparse
import math
import os
import struct
import wave

import yaml

SAMPLE_RATE = 48000
DEFAULT_SITE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "site-demo")

# speech id → (基频 Hz, 每段节拍数, 每拍时长 秒)
TONES = {
    "home": (523.25, 2, 0.18),          # C5
    "zone_ne": (659.25, 3, 0.15),       # E5
    "zone_se": (587.33, 3, 0.15),       # D5
    "zone_sw": (493.88, 3, 0.15),       # B4
    "zone_nw": (440.00, 3, 0.15),       # A4
    "equipment_zone": (392.00, 4, 0.20),  # G4
}
FALLBACK = (349.23, 3, 0.15)  # F4：未登记的 id


def tone_samples(frequency: float, seconds: float, amplitude: float = 0.35):
    total = int(SAMPLE_RATE * seconds)
    fade = max(1, int(SAMPLE_RATE * 0.01))  # 10ms 淡入淡出，避免爆音
    for index in range(total):
        envelope = 1.0
        if index < fade:
            envelope = index / fade
        elif index > total - fade:
            envelope = max(0.0, (total - index) / fade)
        value = amplitude * envelope * math.sin(2.0 * math.pi * frequency * index / SAMPLE_RATE)
        yield int(max(-1.0, min(1.0, value)) * 32767)


def silence_samples(seconds: float):
    for _ in range(int(SAMPLE_RATE * seconds)):
        yield 0


def write_wav(path: str, speech_id: str) -> None:
    frequency, beats, beat_seconds = TONES.get(speech_id, FALLBACK)
    samples = []
    for _ in range(beats):
        samples.extend(tone_samples(frequency, beat_seconds))
        samples.extend(silence_samples(beat_seconds * 0.35))
    with wave.open(path, "wb") as handle:
        handle.setnchannels(1)
        handle.setsampwidth(2)
        handle.setframerate(SAMPLE_RATE)
        handle.writeframes(b"".join(struct.pack("<h", value) for value in samples))


def speech_ids(site_dir: str) -> list[str]:
    path = os.path.join(site_dir, "waypoints.yaml")
    if not os.path.isfile(path):
        raise SystemExit(f"[make_demo_audio] 找不到 {path}")
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    ids = []
    for item in data.get("waypoints", []):
        speech = str(item.get("speech") or item.get("name") or "").strip()
        if speech and speech not in ids:
            ids.append(speech)
    return ids


def main() -> int:
    parser = argparse.ArgumentParser(description="生成离线演示到点语音")
    parser.add_argument("--site-dir", default=DEFAULT_SITE, help="站点目录（读 waypoints.yaml）")
    parser.add_argument("--out-dir", default=None, help="语音输出目录，默认 <站点>/audio")
    parser.add_argument("--ids", nargs="*", default=None, help="只生成指定 id（默认按点位全生成）")
    args = parser.parse_args()

    site_dir = os.path.abspath(args.site_dir)
    out_dir = os.path.abspath(args.out_dir or os.path.join(site_dir, "audio"))
    os.makedirs(out_dir, exist_ok=True)

    ids = args.ids if args.ids else speech_ids(site_dir)
    if not ids:
        print("[make_demo_audio] 没有可生成的 id（waypoints.yaml 里 speech/name 都为空）")
        return 0

    for speech_id in ids:
        path = os.path.join(out_dir, f"{speech_id}.wav")
        write_wav(path, speech_id)
        frequency, beats, beat_seconds = TONES.get(speech_id, FALLBACK)
        print(
            f"[make_demo_audio] {os.path.basename(path)}"
            f"（{frequency:.0f}Hz x{beats} 拍 x{beat_seconds:.2f}s）"
        )
    print(f"[make_demo_audio] 共 {len(ids)} 个，输出目录 {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
