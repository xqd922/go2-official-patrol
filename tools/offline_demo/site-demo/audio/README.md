# 离线演示到点语音

文件名 = `waypoints.yaml` 里 `speech` 字段 + `.wav`。

本目录的 wav 由 `tools/offline_demo/make_demo_audio.py` 生成（标准库合成提示音，
不依赖 ffmpeg / TTS）：

```bash
python3 tools/offline_demo/make_demo_audio.py --out-dir tools/offline_demo/site-demo/audio
```

`run_patrol_demo.sh` 会自动调用它。生产站点请换成现场录制/合成的人声提示。
