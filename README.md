# Go2 Official Stack Patrol Workspace

官方优先的 Go2 巡检工作空间：使用 Unitree 官方 `unitree_ros2` 消息、ROS 2 Humble、Nav2、SLAM Toolbox，实现建图、自主导航、航点巡检和到点语音。反光条已新增独立测试项，用于验证 `intensity` 点云识别，但未接入避障。

完整部署/操作/排错说明见 [`docs/完整部署与使用手册.md`](docs/完整部署与使用手册.md)。

## 组件

| 包 | 职责 |
|---|---|
| `unitree_ros2` | Unitree 官方消息与 CycloneDDS 环境 |
| `go2_patrol_driver` | `/lf/sportmodestate` → TF/odom/imu；`/cmd_vel` → `/api/sport/request`；电量；运动命令 |
| `go2_patrol_bringup` | L1 点云转 `/scan`、SLAM、Nav2、RViz |
| `go2_patrol_patrol` | 航点巡检、到点触发 `/robot/say`、预录语音播放 |
| `go2_patrol_reflective` | 反光条虚拟墙测试检测器，只输出 Marker/状态，不接入 Nav2 |

## Ubuntu / ROS 2

Ubuntu 22.04、ROS 2 Humble。先关闭机器人内不公平的其他 DDS 环境，然后在 WSL/Linux Ubuntu 中执行：

```bash
cd ~/go2_official_ws
./scripts/bootstrap.sh
source scripts/setup.sh
```

`GO2_IFACE` 默认 `enp3s0`，请改成连接 Go2 的网卡名。

## 建图

```bash
source scripts/setup.sh
ros2 launch go2_patrol_bringup mapping.launch.py
```

用遥控器慢速走完整闭环后：

```bash
./scripts/save_map.sh office_floor
```

保存后把地图放到 `src/go2_patrol_bringup/maps/map.yaml`，或启动时传 `map:=...`。

## 导航

```bash
ros2 launch go2_patrol_bringup navigation.launch.py
```

在 RViz 中使用 `2D Pose Estimate` 给初始位姿，再用 `2D Goal Pose` 单目标测试。

## 巡检和语音

1. 编辑 `src/go2_patrol_bringup/config/waypoints.yaml`；
2. 每个点的 `speech` 填语音 ID；
3. 把对应 `speech_id.wav` 放进 `src/go2_patrol_patrol/audio/`；
4. 启动：

```bash
ros2 launch go2_patrol_bringup patrol.launch.py
```

服务：

```bash
ros2 service call /patrol/start std_srvs/srv/Trigger
ros2 service call /patrol/pause std_srvs/srv/Trigger
ros2 service call /patrol/resume std_srvs/srv/Trigger
ros2 service call /patrol/stop std_srvs/srv/Trigger
```

## 采集航点

```bash
ros2 run go2_patrol_patrol record_waypoint entrance
```

输出片段后粘贴进 `waypoints.yaml`，再补 `speech` 和 `dwell`。

## 官方依赖

- Unitree `unitree_ros2` / `unitree_api` / `unitree_go`
- ROS 2 Humble
- Nav2
- SLAM Toolbox
- `pointcloud_to_laserscan`

## 可选扩展：反光条虚拟墙

本工作空间已加入一个独立测试包 `go2_patrol_reflective`，用于实时检查 L1 点云里的高反射线状点，并在 RViz 显示测试用虚拟墙 Marker。它当前只做测试项，**不会让机器人自动避让**，不直接接入 Nav2 costmap。

启动：

```bash
ros2 launch go2_patrol_reflective reflective.launch.py
```

状态话题：

```bash
ros2 topic echo /reflective_wall_status
ros2 topic echo /reflective_wall_active
```

RViz 中添加 `MarkerArray` 后订阅 `/reflective_wall_markers`。
