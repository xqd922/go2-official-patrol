# 地图目录

建图完成后用 `scripts/save_map.sh <名字>` 把地图保存在此处，例如：

```bash
./scripts/save_map.sh office_floor
# 生成 maps/office_floor.pgm + maps/office_floor.yaml
```

导航时通过 `map:=...` 指定，默认使用 `maps/map.yaml`：

```bash
ros2 launch go2_patrol_bringup navigation.launch.py map:=$(ros2 pkg prefix go2_patrol_bringup)/share/go2_patrol_bringup/maps/office_floor.yaml
```

> 建好的地图建议提交到仓库，方便同事直接使用。
