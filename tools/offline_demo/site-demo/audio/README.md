# 离线演示到点语音

文件名 = `waypoints.yaml` 里 `speech` 字段 + `.wav`。

本目录的 wav 是**中文播报**，用 Windows 自带语音合成离线生成（话术见
`deploy/site.example/voice_lines.json`）：

```powershell
powershell -ExecutionPolicy Bypass -File deploy/tools/make_voice_wavs.ps1 -OutDir tools/offline_demo/site-demo/audio
```

没有 Windows 语音环境时，可退回标准库合成提示音（不依赖 ffmpeg / TTS）：

```bash
python3 tools/offline_demo/make_demo_audio.py --out-dir tools/offline_demo/site-demo/audio --force
```

`run_patrol_demo.sh` 会自动调用 `make_demo_audio.py`，但它**默认不覆盖已存在的 wav**，
所以这里的人声文件会原样带到演示站点；生产站点请换成现场录制/合成的人声提示。
