# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 每次改动必须写开发日志

任何代码、脚本、参数、launch 或文档改动，**必须在同一次提交里**往
[`docs/开发日志.md`](docs/开发日志.md) 追加一条记录，并在文首索引表里加一行。

每条的固定结构：

1. **背景**：为什么要改，当时的现象是什么。
2. **改动**：改了哪些文件、关键代码/命令。
3. **证据**：实测输出、日志、频率、测试结果（贴原始片段，不要只写"已验证"）。
4. **踩坑**：试过但失败的路，以及失败原因。
5. **遗留**：没做完的、需要现场确认的。

只写文档的提交也要记一条。没有证据的"应该没问题"不允许写进日志。

## 项目概览

宇树 Go2 自主巡检工作空间（ROS 2 Humble）：机载 4D 雷达 L1/L2 → `pointcloud_to_laserscan` → `/scan` → SLAM Toolbox 建图 / AMCL 定位 → Nav2 导航 → 航点巡检 + 到点/转弯语音。仓库同时包含离线闭环验收（假狗 + 假地图）和生产交付工具链（站点落地、systemd、升级回滚）。

## 运行环境

- **ROS 只在 Ubuntu 22.04 / WSL2 里跑**。Windows 侧仓库只用于编辑与提交；不要在 PowerShell 里起节点或 `colcon`。
- 每个新终端先 `source scripts/setup.sh`。它会转调 `deploy/go2_env.sh`：source ROS、嵌套的 `src/unitree_ros2/cyclonedds_ws/install`、根工作空间 `install`，并生成 `CYCLONEDDS_URI`。
- 接真机时显式指定接狗网卡：`GO2_IFACE=enp3s0 source scripts/setup.sh`。网卡选错的表现是话题完全没数据，不是报错。
  WSL2 镜像网络下网卡名是动态的（本次实测为 `eth7`），按 MAC 识别，不要写死；
  完整配置见 [`docs/WSL2连接Go2完整排错与联调实录.md`](docs/WSL2连接Go2完整排错与联调实录.md)。
- 网络层固定 CycloneDDS（`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`），避免同机其它 DDS/ROS 域抢话题。

### Windows 上验证 Python/脚本改动（本机已验证）

- 不要用 PATH 上的 `python` / `python3`（msys 版没有 pip / PyYAML）。
- 可用 venv：`"$TEMP/go2check2/Scripts/python.exe" -m pytest src -q`（含 pyyaml / pytest / numpy）。
- `deploy/*.sh`、`tools/offline_demo/*.sh` 内部硬调 `python3`。在 Git Bash 里验证这些脚本时需要给 venv 做 shim 并放到 PATH 最前：
  ```bash
  mkdir -p /tmp/shim
  printf '#!/usr/bin/env bash\nexec "%s" "$@"\n' "$TEMP/go2check2/Scripts/python.exe" > /tmp/shim/python3
  chmod +x /tmp/shim/python3
  export PATH="/tmp/shim:$PATH"
  ```
  不要直接复制 `python.exe` 改名（venv 会找不到 `pyvenv.cfg`）。
- 中文控制台输出可能是乱码（cp936/UTF-8），判结果优先看退出码或 `grep -a`，不要用中文关键字匹配。

## 常用命令

```bash
# 一次性环境准备（装 apt 依赖 + vcs 拉钉版 unitree_ros2 + 先编译 cyclonedds_ws 再编译根工作空间）
./scripts/bootstrap.sh
source scripts/setup.sh

# 编译
colcon build --symlink-install
colcon build --symlink-install --packages-select go2_patrol_patrol

# 纯逻辑单测（不需要 ROS，秒级；当前 45 个）
python3 -m pytest src -q
python3 -m pytest src/go2_patrol_patrol/test/test_patrol_sequence.py -q
python3 -m pytest src/go2_patrol_patrol/test/test_patrol_sequence.py::test_following_waypoint_is_the_next_point_in_order -q

# ROS 集成测试（CI 用；跳过上游 unitree_* 的 lint）
colcon test --packages-skip-regex '^unitree_' --event-handlers console_direct+
colcon test-result --verbose

# 离线闭环验收：假狗 + 假地图，走生产同一套 apply_site/run_site，逐点断言顺序/精度/行程/播报
tools/offline_demo/run_patrol_demo.sh                 # 无头，约 2 分钟；通过打印「验收通过 ✅」，退出码 0
tools/offline_demo/run_patrol_demo.sh --rviz --loops 0 --keep
tools/offline_demo/run_patrol_demo.sh --battery-drop-after 60
tools/offline_demo/run_patrol_demo.sh --obstacles "0.5,0,0.6,0.6,0.6"
tools/offline_demo/run_patrol_demo.sh --clean         # 清理残留进程
tools/offline_demo/run_demo.sh                        # 只演示建图（不是巡检验收）

# 真机
ros2 launch go2_patrol_bringup mapping.launch.py
./scripts/save_map.sh office_floor
ros2 launch go2_patrol_bringup navigation.launch.py
ros2 launch go2_patrol_bringup patrol.launch.py       # 起完自动开跑
ros2 run go2_patrol_patrol record_waypoint entrance   # 从 /amcl_pose（map 帧）采点

# 巡检服务
ros2 service call /patrol/pause  std_srvs/srv/Trigger
ros2 service call /patrol/resume std_srvs/srv/Trigger
ros2 service call /patrol/stop   std_srvs/srv/Trigger

# 生产交付
sudo deploy/install.sh --ref v0.1.0 --iface enp3s0
sudo deploy/apply_site.sh --site-dir /media/usb/site-office --name office
sudo systemctl enable --now go2-patrol@office
sudo /opt/go2/current/deploy/install.sh --rollback v0.1.0
```

## 架构

### 主数据流

```text
/lf/sportmodestate ──odom_bridge──► /odom + odom→base_footprint TF + /lf/imu
/utlidar/cloud_deskewed ──pointcloud_to_laserscan──► /scan（best_effort）
/scan + 静态地图 ──AMCL──► /amcl_pose + map→odom TF
Nav2 ──/cmd_vel──► twist_bridge（限速/前置急停/看门狗）──► /api/sport/request（MOVE/STOP_MOVE）
/lf/lowstate ──battery_bridge──► /battery_state
patrol_manager ──NavigateToPose──► Nav2
patrol_manager ──/robot/say──► voice_player ──► audio/<id>.wav
```

TF 链：

```text
map ──(AMCL)──► odom ──(odom_bridge)──► base_footprint ──(*只允许一个发布者*)──► base_link
                                                              └─(static z=0.13)─► utlidar_lidar
```

`base_footprint → base_link` 由 `tf_chain_guard` 在 `mode=auto` 下探测 5s、没人发才补；机器狗自身若已在发就完全不插手，避免 Nav2 报 TF 冲突。现场用站点 `env.GO2_BASE_FOOTPRINT_TF=always|never` 钉死。

### 包与职责

| 包 | 职责 |
|---|---|
| `go2_patrol_driver` | `odom_bridge`（状态→odom/IMU/TF）、`battery_bridge`、`twist_bridge`（`/cmd_vel`→官方 Sport 请求，含前置急停与两路断流看门狗）、`tf_chain_guard`、`sport_command` |
| `go2_patrol_bringup` | 纯 launch/config 包：感知转换、SLAM/Nav2 参数、巡检 RViz 视图 |
| `go2_patrol_patrol` | `patrol_manager`（ROS 交互）、`patrol_sequence`（纯逻辑状态机）、`voice_player`、`record_waypoint` |
| `go2_patrol_reflective` | 反光条虚拟墙测试项，只发 Marker/状态，**不接 Nav2 costmap** |
| `tools/offline_demo/` | 假数据 + 闭环假狗 + 逐点验收 |
| `deploy/` | 站点落地、systemd、安装升级回滚、预检、离线交付包 |

### launch 组合（不要重复 include driver）

- `perception.launch.py`：静态 TF `base_link → utlidar_lidar`（z=0.13）+ `pointcloud_to_laserscan`。
- `mapping.launch.py` = perception + driver + `slam_toolbox`。
- `navigation.launch.py` = perception + driver + Nav2 bringup（延迟 3s）+ RViz。
- `patrol.launch.py` = `navigation.launch.py` + `voice_player` + `patrol_manager`。**它不再单独 include driver**，否则同名节点起两次、TF/odom 重复发布。

### 纯逻辑模块（可单测，别塞回 ROS 节点）

这些模块刻意不依赖 ROS，单测与运行时共用同一套规则：

- `driver/safety.py`：扫描新鲜度 + 前置距离 → go/slow/stop/emergency/no_scan。
- `driver/frame_chain.py`：TF 补链的 auto/always/never 决策与自帧识别。
- `patrol/patrol_sequence.py`：圈数、收工返航、低电量返航状态机。离线验收器 `demo_verify.py` 直接复用它预演预期访问顺序。
- `patrol/speech_policy.py`：播放/排队/丢弃策略。
- `offline_demo/geometry.py`：假点云、静态地图、点位净空校验的唯一几何来源。

新增行为时优先在纯逻辑模块实现 + 加 pytest，ROS 节点只做参数、订阅和调用。

### 站点参数链路（生产核心）

```text
站点 site.yaml（唯一手工入口）
  └─ deploy/tools/render_site.py
       ├─ nav2_params.yaml  （生成：AMCL 初始位姿、robot_radius、goal_checker 容差）
       └─ site.env          （生成：GO2_IFACE/GO2_DOG_IP/GO2_PATROL_*/GO2_MAX_LINEAR…）
            └─ deploy/run_site.sh（set -a 导出）→ patrol.launch.py → driver.launch.py
                 （driver 用 EnvironmentVariable 读 GO2_<参数名大写>）
```

- `apply_site.sh` 先用 `render_site.py --check-only` 校验**源目录**，不通过就一个文件都不动；校验通过后才覆盖站点目录并生成参数。
- `nav2_params.yaml` / `site.env` 是生成文件，**不要手改**；改 `site.yaml` 后重跑 `apply_site.sh`。
- `safety:` 三个距离必须递增（急停 < 停车 < 减速）；`nan/inf`、非数字、负值、`safety:` 与 `env:` 重名都会被拒绝落地。
- 站点参数在 `site.env` 里写成 `VAR=${VAR:-值}`，所以命令行/环境变量优先：`GO2_MAX_LINEAR=0.4 deploy/run_site.sh office` 可临时降速。
- `sites/`、`build/`、`install/`、`log/`、`src/unitree_ros2/` 都不进 git。

如果给 `patrol_manager` 或 `twist_bridge` 加站点可调参数，要同步改：节点 `declare_parameter` → launch `DeclareLaunchArgument`/`env_arg` → `run_site.sh` 的 `LAUNCH_ARGS` → `render_site.py` 的 `patrol`/`SAFETY_KEYS` 映射 → `site.example/site.yaml` → 文档。只改一处会出现“参数写了但不生效”。

### 离线闭环演示

`run_patrol_demo.sh` 依次跑：`make_demo_map.py` / `make_demo_audio.py` → `check_site.py` → 生产 `apply_site.sh` → `run_site.sh`（真起感知/驱动/Nav2/巡检/播报）→ `demo_verify.py`。假狗 `fake_go2_loop.py` 订阅 `/api/sport/request` 积分运动模型，再按同一份 `geometry.py` 射线投射点云，因此地图、扫描、定位自洽。报告在 `/tmp/go2_offline_demo/report.json`。

离线演示**测不出**真机专属问题：真机 TF 是否断、DDS/网卡、实际急停距离。它的价值是“任何节点起不来 → 狗不动 → 验收超时 → 非 0 退出”，所以 CI 专门有一个真起 ROS 图的 job。

## 必须保持的约束（改前先读）

- **没有新扫描 ≠ 前方畅通**：`safety.is_fresh(None)` 必须为假；`require_scan=true` 时扫描断流要停车告警。
- **官方 `MOVE` 会保持最后一次速度**：`/cmd_vel` 断流必须补 `STOP_MOVE`；退出路径也要补刹车。`twist_bridge` 必须用 `SignalHandlerOptions.NO`（不是 `NONE`）自己接管 SIGINT/SIGTERM，让 context 活到刹车发完；不要退回 rclpy 默认处理器。
- **QoS 不匹配会静默失效**：`/scan` 用 sensor data（best effort）订阅；`/amcl_pose` 用 `qos_profile_sensor_data`；`/tf_static` 用 `TRANSIENT_LOCAL`。
- **Nav2 参数的四个坑**（`config/nav2_params.yaml`）：
  1. `map_server` 必须有 `yaml_filename` 段（`RewrittenYaml` 只改写已存在的 key）；
  2. `controller_plugins` / `goal_checker_plugins` 必须是字符串数组；
  3. `plugins` 里声明了哪个层，就必须有对应的配置块（尤其 `inflation_layer`），否则整栈崩溃；
  4. 不要写 `default_nav_to_pose_bt_xml: ""`（空串 = 加载空文件），要用默认行为树就删掉该行。
- **巡检结束语义**：`/patrol/stop` 或跑完一圈后节点继续活着但不动作，`/patrol/start` 返回 `success=false`；要重新巡检只能重启服务。无人值守站点把 `loops` 设 0。
- **坐标帧**：航点与 Nav2 目标都在 `map` 帧。`record_waypoint` 默认只信 `/amcl_pose`；`--allow-odom-fallback` 采到的是 odom 帧，必须自行核对。
- **姿态策略**：`yaw_mode=fixed` 才要求到点转到点位 `yaw`；`travel/next` 用于减少路口原地转，验收器也只对 `fixed` 点位判朝向。
- **中文注释是设计记录**：大量注释在解释“为什么不能这样写”的失败模式，修改相关代码时保留并更新，不要当噪声删掉。

## 测试与 CI

`.github/workflows/ci.yml` 两个 job：

1. `build-and-test`：`colcon build` + `colcon test`（跳过 `^unitree_`）。
2. `offline-closed-loop`（依赖第一个）：额外装 `ros-humble-rmw-cyclonedds-cpp`，先编译 `src/unitree_ros2/cyclonedds_ws`（与 `bootstrap.sh` 顺序一致），再跑 `run_patrol_demo.sh --timeout 900`，始终上传 `/tmp/go2_offline_demo/`。

改完代码至少跑：`python3 -m pytest src -q`；涉及 launch/节点启动/Nav2 参数时再跑离线闭环演示。

## 关键文档

- `docs/真机部署与使用手册.md`：真机安装、联网、建图、导航、巡检、安全验收与生产部署。
- `docs/WSL2连接Go2完整排错与联调实录.md`：Windows + WSL2 直连 Go2 的排错过程、四层修复与实测判据。
- `docs/开发日志.md`：每次改动的背景、证据、踩坑与遗留，排错先看这里。
- `docs/仿真与离线验收手册.md`：假狗离线闭环、自动验收、测试与仿真边界。
- `deploy/README.md`：目录布局、站点参数表、升级回滚。
- `README.md`：真机联调须知（TF 补链、Nav2 坑、看门狗、阈值调整）。
