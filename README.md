# Go2 Official Stack Patrol Workspace

![CI](https://github.com/xqd922/go2-official-patrol/actions/workflows/ci.yml/badge.svg)

面向宇树 Go2 的自主巡检工作空间：官方 `unitree_ros2` 消息 + ROS 2 Humble + Nav2 + SLAM Toolbox，
一条链路打通**建图 → 自主导航 → 航点巡检 → 到点语音**，并自带离线闭环验收与生产交付工具包。
感知使用**机载 4D 激光雷达（L1/L2，以实机为准）**，不要求外置雷达。

> **EDU / 官方 SLAM 服务提醒**
>
> 机身自带的新款雷达是 **L2**（旧批次是 L1），SDK 话题接口相同。
> L2 周向约 5.55Hz、单圈约 11500 点；L1 周向约 11Hz、单圈约 2000 点。
> 自动巡检建议先从 0.3～0.6 m/s 验证急停距离。
> 宇树官方 `slam_operate` 导航服务**不使用机载 L2**，只支持
> **EDU + 拓展坞 + 官方外置 MID-360/XT16**。两种路线的差异、限制和
> "反光条能否当墙"的结论见
> [`docs/EDU与L2-官方SLAM导航服务对比.md`](docs/EDU与L2-官方SLAM导航服务对比.md)。

## 从哪开始

| 我现在要干什么 | 看这里 |
|---|---|
| 手上没有机器狗，先看效果、跑自动验收 | 本文「离线闭环演示」一节 |
| 接真机：连线自检 → 建图 → 导航 → 巡检 | [`docs/使用指南.md`](docs/使用指南.md) 路线 B |
| 完全没接触过 ROS、第一次上真机 | [`docs/小白上机实操手册.md`](docs/小白上机实操手册.md)（从"交付到底交什么"开始手把手） |
| 交付到另一台生产机器狗 | [`deploy/README.md`](deploy/README.md) + [`docs/生产交付手册.md`](docs/生产交付手册.md) |
| 从零装环境（含踩坑记录） | [`docs/开发环境搭建与安装记录.md`](docs/开发环境搭建与安装记录.md) |
| 完整背景、接口细节、排错大全 | [`docs/完整部署与使用手册.md`](docs/完整部署与使用手册.md) |

## 组件

| 包 / 目录 | 职责 |
|---|---|
| `unitree_ros2` | Unitree 官方消息与 CycloneDDS 环境（`repos/` 钉版本，`bootstrap.sh` 自动拉取） |
| `go2_patrol_driver` | `/lf/sportmodestate` → TF/odom/imu；`/cmd_vel` → `/api/sport/request`；电量；运动命令 |
| `go2_patrol_bringup` | 机载 4D 雷达点云 → `/scan`、SLAM、Nav2、RViz 巡检视图 |
| `go2_patrol_patrol` | 航点巡检调度、到点语音播放、航点采集 |
| `go2_patrol_reflective` | 反光条虚拟墙测试检测器，只输出 Marker/状态，不接入 Nav2 |
| `tools/offline_demo/` | 离线闭环演示：造数据 + 假狗 + 逐点自动验收 |
| `deploy/` | 生产交付工具包：安装、站点落地、systemd、发布包、预检 |

## 环境准备（一次性）

Ubuntu 22.04 / WSL2 + ROS 2 Humble。Windows 侧仓库只做提交，ROS 在 WSL/Linux 里跑：

```bash
cd ~/go2_official_ws
./scripts/bootstrap.sh     # 装系统依赖 + vcs 拉官方 unitree_ros2 + colcon build
source scripts/setup.sh    # 每个新终端 source 一次
```

`setup.sh` 会按网卡生成 `CYCLONEDDS_URI` 并打印一行摘要。`GO2_IFACE` 默认 `enp3s0`，
接机器狗时显式指定**接狗那张网卡**；同时避免同机其它 DDS 环境（别的 ROS 发行版 / 域）抢占话题。

## 离线闭环演示（没有机器狗也能跑通全链路）

`tools/offline_demo/` 用假数据把**生产同一条链路**整跑一遍：造地图/造语音 → 站点资产校验 →
`apply_site.sh` 落地站点 → `run_site.sh` 起巡检栈 → 逐点自动验收（到点顺序 / 精度 ≤0.40m / 行程 / 播报）。

```bash
# 无头跑一轮（台架或 CI 用），约 2 分钟，通过会打印「验收通过 ✅」，退出码 0
tools/offline_demo/run_patrol_demo.sh

# 带 RViz 持续巡检，肉眼确认全过程
tools/offline_demo/run_patrol_demo.sh --rviz --loops 0 --keep

tools/offline_demo/run_patrol_demo.sh --battery-drop-after 60          # 验证低电量返航
tools/offline_demo/run_patrol_demo.sh --obstacles "0.5,0,0.6,0.6,0.6"  # 验证绕障
tools/offline_demo/run_patrol_demo.sh --clean                          # 收工清理残留进程
```

报告写到 `/tmp/go2_offline_demo/report.json`。RViz 点位配色：蓝=未访问、黄=当前目标、
绿=已到达、红=返航点（详见 [`rviz/README.md`](src/go2_patrol_bringup/rviz/README.md)）。

> `run_demo.sh` 是**建图**演示（假狗 + SLAM Toolbox 看地图长出来）；
> `run_patrol_demo.sh` 是**巡检闭环**验收，跑的是生产同一套 `run_site.sh` 与站点参数——
> 能不能交付看它。

## 接真机

### 1. 连通性自检

```bash
GO2_IFACE=<接狗的网卡名> source scripts/setup.sh

ros2 topic hz /lf/sportmodestate        # 运动状态，约 50Hz
ros2 topic hz /utlidar/cloud_deskewed   # 机载点云，L2 约 5.5Hz / L1 约 11Hz
```

两个都有频率，才说明 DDS 选网卡选对了、能收到机器狗数据。

### 2. 建图（每个站点做一次）

```bash
ros2 launch go2_patrol_bringup mapping.launch.py

# 用遥控器慢速走完巡检闭环后存图：
./scripts/save_map.sh office_floor
```

地图存到 `src/go2_patrol_bringup/maps/`。作为默认地图就改名成 `map.yaml`/`map.pgm`，
或启动时 `map:=<路径>` 指定。

### 3. 导航单点测试

```bash
ros2 launch go2_patrol_bringup navigation.launch.py
```

RViz 里先用 `2D Pose Estimate` 给初始位姿，等粒子云收敛到机器人身上，再用 `2D Goal Pose`
发一个近处目标确认能规划能走。默认加载巡检视图 `patrol.rviz`
（地图/代价地图/激光/规划/定位/巡检点位一屏看完），换视图加 `rviz_config:=<自己的.rviz>`。

### 4. 巡检 + 语音

1. 编辑 `src/go2_patrol_bringup/config/waypoints.yaml`（顺序就是巡检顺序）：

```yaml
waypoints:
  - name: entrance
    x: 0.0             # map 坐标系下的位置（米）
    y: 0.0
    yaw: 0.0           # 目标朝向（弧度）
    dwell: 4.0         # 到点停留秒数
    speech: entrance   # 语音 ID → audio/entrance.wav
    home: true         # 标记为返航点（只能有一个）
```

2. 把语音文件放进 `src/go2_patrol_patrol/audio/<speech_id>.wav`（`.wav` 不进 git）；
   没有录音时可用 Windows 自带中文语音离线生成：
   `powershell -ExecutionPolicy Bypass -File deploy/tools/make_voice_wavs.ps1 -OutDir <audio 目录>`
   （话术在 `deploy/site.example/voice_lines.json`）；
3. 启动并控制：

```bash
ros2 launch go2_patrol_bringup patrol.launch.py

ros2 service call /patrol/start  std_srvs/srv/Trigger   # 开始
ros2 service call /patrol/pause  std_srvs/srv/Trigger   # 暂停
ros2 service call /patrol/resume std_srvs/srv/Trigger   # 继续
ros2 service call /patrol/stop   std_srvs/srv/Trigger   # 停止
```

常用 launch 参数：`loops`（圈数，0=无限）、`return_home`（默认 true）、
`battery_low`（低电量阈值，默认 20%）、`arrival_tolerance`（到点容差，默认 0.30m）、
`yaw_mode`（到点朝向：`fixed`/`travel`/`next`，路口用 `travel` 可显著减少原地转）、
`rviz`、`map`、`params_file`、`waypoints_file`、`audio_dir`。

> 走到角落"原地转半天"多半不是定位问题，而是点位 `yaw` 与行进方向无关，
> 详见 [`docs/完整部署与使用手册.md`](docs/完整部署与使用手册.md) §9.1.1。

### 5. 采集航点

懒得手填坐标时，把狗开到位置再采：

```bash
ros2 run go2_patrol_patrol record_waypoint entrance
```

它打印一段可直接粘进 `waypoints.yaml` 的 YAML，再补 `speech` 和 `dwell` 即可。

## 生产交付（部署到另一台机器狗）

```bash
# 生产机：装到 /opt/go2/releases/<版本>，切 current，装 systemd 单元
sudo deploy/install.sh --ref v0.1.0 --iface enp3s0

# 落地站点资产（地图/点位/语音/site.yaml），生成站点参数并自检
sudo deploy/apply_site.sh --site-dir /media/usb/site-office --name office

# 开机自启 + 崩溃自动重启
sudo systemctl enable --now go2-patrol@office
```

站点差异（固定起点初始位姿、巡检任务参数、网卡）全部集中在站点 `site.yaml`；
升级与回滚只切 `/opt/go2/current` 软链（`sudo deploy/install.sh --rollback <旧版本>`），
站点资产不受影响。现场标定与验收清单见 [`docs/生产交付手册.md`](docs/生产交付手册.md)。

## 常用命令速查

| 我想… | 命令 |
|---|---|
| 加载环境 | `source scripts/setup.sh` |
| 无机器狗看效果 | `tools/offline_demo/run_patrol_demo.sh --rviz --loops 0 --keep` |
| 建图 / 存图 | `ros2 launch go2_patrol_bringup mapping.launch.py` + `./scripts/save_map.sh <名字>` |
| 导航 / 巡检 | `ros2 launch go2_patrol_bringup navigation.launch.py` / `patrol.launch.py` |
| 采集航点 | `ros2 run go2_patrol_patrol record_waypoint <名字>` |
| 编译 / 单测 / 端到端验收 | `colcon build --symlink-install` / `python3 -m pytest src/go2_patrol_patrol/test/ -q` / `tools/offline_demo/run_patrol_demo.sh` |

CI：push 到 `main` 或开 PR 时会跑 GitHub Actions（Humble 上 `colcon build` + `colcon test`），
见 [`.github/workflows/ci.yml`](.github/workflows/ci.yml)。

## 真机联调须知（本仓库已修，改参数时别改回去）

- **TF `base_footprint → base_link` 只能有一处发布**：机器人自身若已发这条 TF，就不要再用
  `static_transform_publisher` 补链，重复发布会让 Nav2 报 TF 冲突。
- **`nav2_params.yaml` 的四个坑**：
  1. 必须有 `map_server` 的 `yaml_filename` 段——`RewrittenYaml` 只改写**已存在**的 key，
     缺段会让地图路径永远注入不进去，localization 启动即 `parameter 'yaml_filename' is not initialized`；
  2. `controller_server` 的 `controller_plugins` / `goal_checker_plugins` 必须是**字符串数组**
     （`[FollowPath]` / `[goal_checker]`），写成标量会 `has invalid type` 让组件加载失败；
  3. `local_costmap` 的 `plugins` 里声明了 `inflation_layer`，就**必须**有对应的 `inflation_layer:`
     配置块，否则 costmap 插件加载 FATAL、`nav2_container` 段错误，整栈全崩（真机同样崩）；
  4. `bt_navigator.default_nav_to_pose_bt_xml` 不要写成空串——空串不等于"用默认值"，
     而是加载空文件 → `Empty Tree` → 所有导航目标必然失败。要用内置默认行为树就**删掉这一行**。
- **急停阈值与速度上限按现场地面/负载复核**，见 [`docs/生产交付手册.md`](docs/生产交付手册.md) 第 11 节。

## 可选扩展：反光条虚拟墙（测试项）

独立测试包 `go2_patrol_reflective`：实时检查机载 4D 雷达点云里的高反射线状点，
在 RViz 显示测试用虚拟墙 Marker。当前**只做测试项，不接入 Nav2 costmap、不影响避障**。

```bash
ros2 launch go2_patrol_reflective reflective.launch.py

ros2 topic echo /reflective_wall_status   # 也支持 /reflective_wall_active
# RViz 中添加 MarkerArray 订阅 /reflective_wall_markers
```

## 官方依赖

- Unitree `unitree_ros2` / `unitree_api` / `unitree_go`（`repos/go2_official_ws.repos` 钉版本）
- ROS 2 Humble
- Nav2
- SLAM Toolbox
- `pointcloud_to_laserscan`

## 文档导航

| 文档 | 内容 |
|---|---|
| [`docs/使用指南.md`](docs/使用指南.md) | 按"我现在要干什么"分三条路线，含真机确认清单与排错速查 |
| [`docs/小白上机实操手册.md`](docs/小白上机实操手册.md) | 零基础第一篇：交付概念、名词表、第一次上机逐步操作与排错 |
| [`docs/完整部署与使用手册.md`](docs/完整部署与使用手册.md) | 完整背景、设计取舍、接口细节、排错大全 |
| [`docs/生产交付手册.md`](docs/生产交付手册.md) | 交付流程、现场标定与验收清单 |
| [`docs/交付清单.md`](docs/交付清单.md) | 交付物料、自带工具、现场验收与签字栏（可打印） |
| [`docs/开发环境搭建与安装记录.md`](docs/开发环境搭建与安装记录.md) | 从零装环境与踩坑记录 |
| [`docs/EDU与L2-官方SLAM导航服务对比.md`](docs/EDU与L2-官方SLAM导航服务对比.md) | 官方 SLAM 服务 vs 本方案（机载 L2 路线） |
| [`deploy/README.md`](deploy/README.md) | 生产交付工具包说明 |
