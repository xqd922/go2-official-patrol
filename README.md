# Go2 Official Stack Patrol Workspace

![CI](https://github.com/xqd922/go2-official-patrol/actions/workflows/ci.yml/badge.svg)

官方优先的 Go2 巡检工作空间：使用 Unitree 官方 `unitree_ros2` 消息、ROS 2 Humble、Nav2、SLAM Toolbox，实现建图、自主导航、航点巡检和到点语音。当前路线使用**机载 4D 激光雷达（L1/L2，以实机为准）**，不要求外置雷达。反光条已新增独立测试项，用于验证 `intensity` 点云识别，但未接入避障。

> **EDU / 官方 SLAM 服务提醒**
>
> 机身自带的新款雷达是 **L2**（旧批次是 L1），SDK 话题接口相同。
> L2 周向约 5.55Hz、单圈约 11500 点；L1 周向约 11Hz、单圈约 2000 点。
> 自动巡检建议先从 0.3～0.6 m/s 验证急停距离。
> 宇树官方 `slam_operate` 导航服务**不使用机载 L2**，只支持
> **EDU + 拓展坞 + 官方外置 MID-360/XT16**。两种路线的差异、限制和
> “反光条能否当墙”的结论见
> [`docs/EDU与L2-官方SLAM导航服务对比.md`](docs/EDU与L2-官方SLAM导航服务对比.md)。

**不知道从哪下手先看 [`docs/使用指南.md`](docs/使用指南.md)**（按"我现在要干什么"分三条路线：
离线演示 / 接真机 / 交付生产机）。完整背景与排错见
[`docs/完整部署与使用手册.md`](docs/完整部署与使用手册.md)。

## 组件

| 包 | 职责 |
|---|---|
| `unitree_ros2` | Unitree 官方消息与 CycloneDDS 环境 |
| `go2_patrol_driver` | `/lf/sportmodestate` → TF/odom/imu；`/cmd_vel` → `/api/sport/request`；电量；运动命令 |
| `go2_patrol_bringup` | 机载 4D 雷达点云转 `/scan`、SLAM、Nav2、RViz |
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
默认加载巡检视图 [`rviz/patrol.rviz`](src/go2_patrol_bringup/rviz/README.md)
（地图/代价地图/激光/规划/定位/巡检点位一屏看完），换视图加 `rviz_config:=<自己的.rviz>`。

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

## 离线闭环演示（没有机器狗也能验收全链路）

`tools/offline_demo/` 用假数据把**生产链路整条跑通**：造地图/造语音 → 站点资产校验 →
`apply_site.sh` 落地站点 → `run_site.sh` 起巡检栈 → 逐点自动验收（顺序/精度/行程/播报）。

```bash
# 无头跑一轮（台架或 CI 用），通过会打印「验收通过 ✅」
tools/offline_demo/run_patrol_demo.sh

# 带 RViz 肉眼确认巡检过程（需要有显示环境）
tools/offline_demo/run_patrol_demo.sh --rviz

tools/offline_demo/run_patrol_demo.sh --battery-drop-after 60      # 验证低电量返航
tools/offline_demo/run_patrol_demo.sh --obstacles "0.5,0,0.6,0.6,0.6"  # 验证绕障
tools/offline_demo/run_patrol_demo.sh --loops 0 --timeout 600     # 无限巡检
tools/offline_demo/run_patrol_demo.sh --clean                     # 只清残留进程
```

和 `run_demo.sh` 的区别：`run_demo.sh` 是**建图**演示（假狗 + SLAM Toolbox + RViz 看地图长出来，
依赖 `$HOME/go2_official_ws`）；`run_patrol_demo.sh` 是**巡检闭环**演示，跑的是生产同一套
`run_site.sh` 与站点参数，并且自带断言——能不能交付看它。

## 生产交付（部署到另一台机器狗）

交付到生产机走 [`deploy/`](deploy/README.md) 工具包，流程、现场标定与验收清单见 [`docs/生产交付手册.md`](docs/生产交付手册.md)。

```bash
# 生产机：装到 /opt/go2/releases/<版本>，切 current，装 systemd 单元
sudo deploy/install.sh --ref v0.1.0 --iface enp3s0

# 落地站点资产（地图/点位/语音/site.yaml），生成站点参数并自检
sudo deploy/apply_site.sh --site-dir /media/usb/site-office --name office

# 开机自启 + 崩溃自动重启
sudo systemctl enable --now go2-patrol@office
```

要点：站点差异（固定起点初始位姿、巡检任务参数、网卡）集中在站点 `site.yaml`；
升级与回滚只切 `/opt/go2/current` 软链，站点资产不受影响。

## 官方依赖

- Unitree `unitree_ros2` / `unitree_api` / `unitree_go`
- ROS 2 Humble
- Nav2
- SLAM Toolbox
- `pointcloud_to_laserscan`

## 真机联调须知

离线闭环演示已把整条链路跑通并断言过，但以下几处上真机时仍要实测确认：

- **TF `base_footprint → base_link`**：机器人自身若已发布这条 TF，就不要再用
  `static_transform_publisher` 补链（见 `tools/offline_demo/run_demo.sh` 注释），重复发布会让 Nav2 报 TF 冲突。
- **Nav2 参数三处坑**（本仓库已修，改站点参数时别改回去）：
  1. `nav2_params.yaml` 必须有 `map_server` 的 `yaml_filename` 段——`nav2_bringup` 的 `RewrittenYaml`
     只改写**已存在**的 key，缺段会让地图路径永远注入不进去，localization 启动即
     `parameter 'yaml_filename' is not initialized`；
  2. `controller_server` 的 `controller_plugins` / `goal_checker_plugins` 必须是**字符串数组**
     （`[FollowPath]` / `[goal_checker]`），写成标量会 `has invalid type` 让组件加载失败；
  3. `local_costmap` 的 `plugins` 里声明了 `inflation_layer`，就**必须**有对应的 `inflation_layer:`
     配置块，否则 costmap 插件加载 FATAL、`nav2_container` 段错误，整栈全崩（真机同样崩）。
- **`bt_navigator.default_nav_to_pose_bt_xml` 不要写成空串**：空串不等于"用默认值"，而是加载空文件 →
  `Empty Tree` → 所有导航目标必然失败。要用内置默认行为树就**删掉这一行**。
- **急停阈值与速度上限按现场地面/负载复核**，见 [`docs/生产交付手册.md`](docs/生产交付手册.md) 第 11 节。

## 可选扩展：反光条虚拟墙

本工作空间已加入一个独立测试包 `go2_patrol_reflective`，用于实时检查机载 4D 雷达点云里的高反射线状点，并在 RViz 显示测试用虚拟墙 Marker。它当前只做测试项，**不会让机器人自动避让**，不直接接入 Nav2 costmap。

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
