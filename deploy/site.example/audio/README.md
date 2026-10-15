# 到点语音

- 文件名 = `waypoints.yaml` 里该点的 `speech` 字段 + `.wav`，例如 `equipment_zone.wav`。
- 播放优先级：`aplay`（Linux）> `afplay`（macOS）> `ffplay`；都没有时只把文本打进日志。
- 建议 48 kHz / 16 bit / 单声道，时长 2–6 秒。

```bash
# 已有 mp3 时转成 wav
ffmpeg -i equipment_zone.mp3 -ar 48000 -ac 1 equipment_zone.wav

# 临时静音占位（只想验证链路，不想出声）
ffmpeg -f lavfi -i anullsrc=r=48000:cl=mono -t 1 equipment_zone.wav
```

> 生产机上缺少对应 wav 不会报错，只会在 `journalctl -u go2-patrol@<站点>` 里看到
> `audio not found for '<id>'`，巡检继续。交付前按点位清单逐个核对。
