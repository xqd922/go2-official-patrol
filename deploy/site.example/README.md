# 站点资产模板

一个「站点」就是一台生产机器狗要跑的现场配置：地图 + 点位 + 语音 + 站点参数。
复制本目录改名为你的现场（例如 `site-office-3f`），把建图产物和标定结果填进去，
再用 `deploy/apply_site.sh` 落到生产机。

> 本目录里的 `map.yaml` / `map.pgm` 是 **10m × 10m 的空房间占位图**，只为让
> `apply_site.sh` 和 systemd 链路在没有真机、没有建图产物时也能跑通（台架自检）。
> 现场交付必须换成 `scripts/save_map.sh` 建出来的真实地图。

## 文件说明

| 文件 | 必需 | 说明 |
|---|---|---|
| `map.yaml` | 是 | 地图描述（`image:` 指向 `.pgm`），用 `scripts/save_map.sh <名字>` 生成；此处为占位图 |
| `map.pgm` | 是 | 地图数据，文件名以 `map.yaml` 的 `image:` 为准；此处为占位图 |
| `waypoints.yaml` | 是 | 巡检点位；缺省会填仓库示例，但**必须现场重新标定** |
| `site.yaml` | 建议 | 固定起点初始位姿、巡检任务参数、网卡等站点差异 |
| `audio/*.wav` | 建议 | 到点语音，文件名 = 点位的 `speech` + `.wav` |

## 用法

```bash
# 开发机（站点落在 <工作空间>/sites/<名字>）
deploy/apply_site.sh --site-dir deploy/site.example --name demo

# 生产机（站点落在 /opt/go2/sites/<名字>，并装成 systemd 实例）
sudo deploy/apply_site.sh --site-dir /media/usb/site-office --name office
sudo systemctl enable --now go2-patrol@office
```

`apply_site.sh` 做三件事：复制资产 → 用 `site.yaml` 生成 `nav2_params.yaml` / `site.env` → 跑一遍启动前自检。

## 现场标定要点

1. 建图：`ros2 launch go2_patrol_bringup mapping.launch.py`，遥控慢走闭环，`./scripts/save_map.sh <名字>`。
2. 点位：`ros2 run go2_patrol_patrol record_waypoint <点位名>` 逐点采集，粘进 `waypoints.yaml`。
3. 固定起点：把机器狗摆到每次开机的固定位置（充电桩/停机位），在 RViz 读 `map` 下的位姿，填进 `site.yaml` 的 `initial_pose`。
4. 语音：按点位 `speech` 命名 wav，放进 `audio/`。
5. 自检：`deploy/preflight.sh <名字>` 全绿后再 `systemctl enable --now`。
