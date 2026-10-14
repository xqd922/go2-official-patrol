# 13 - EDU、机载 L2 与官方 SLAM 导航服务

> 结论速览
>
> - 现在新款 Go2 机身自带的是 **Unitree 4D LiDAR L2**，不是老资料里的 L1。
> - 但宇树官方的 **SLAM 导航服务接口（`slam_operate`）不使用机身 L2**。
>   官方文档明确要求：**EDU + 拓展坞 + 宇树官方购入的外置雷达（MID-360 或 XT16）**。
> - 只有 EDU + 机载 L2、没有拓展坞和外置雷达时，本仓库现有方案不变：
>   `unitree_ros2 + slam_toolbox + Nav2`，反光条虚拟墙可以正式接入 costmap。
> - EDU + 拓展坞 + MID-360/XT16 时，可以改成官方 `slam_operate` 后端；
>   巡检航点和到点语音仍由我们自己的程序负责，但反光条最多只能“暂停/告警”，
>   不能作为虚拟墙参与官方导航的绕行决策。
> - 如果“反光条必须当墙并自动绕行”是硬需求：**不要切官方 SLAM 服务**，
>   继续使用我们自己的 Nav2 方案。

---

## 1. 先纠正：机身自带的是 L2

用户引用的官方产品页原文是：

> “Go2 搭载 Unitree 自研 4D 激光雷达 L2，360°×96° 半球形超广角感知能力，
> 拥有超低盲区，最小探测距离低至 0.05m……”

> “通过使用 Go2 搭载的自研激光雷达 L2 与专属 APP，即可实现一定区域内的
> 点云地图构建，并可指定行进路径控制 Go2 自主移动。”

这与当前产品页一致。L1 和 L2 的主要差异如下：

| 项目 | L1（旧批次/旧资料） | L2（当前产品页） |
| --- | --- | --- |
| 型号 | Unitree 4D LiDAR L1 | Unitree 4D LiDAR L2 |
| 视场角 | 360° × 90° | 360° × 96° |
| 周向扫描频率 | 11 Hz | 5.55 Hz（可定制调节） |
| 竖直扫描频率 | 180 Hz | 216 Hz |
| 有效点频 | 21600 点/秒 | 64000 点/秒 |
| 单圈点数（估算） | 约 2000 点/圈 | 约 11500 点/圈 |
| 近处盲区 | 0.05 m | 0.05 m |
| 扫描距离 | 30 m @90% 反射率（RM 版） | 30 m @90% 反射率 |
| 测距分辨率 | 约 4.5 mm（L2 产品页口径） | 4.5 mm |
| 数据形式 | 3D 位置 + 1D 灰度（intensity） | 3D 位置 + 1D 灰度（intensity） |

注意：宇树部分 Go2 SDK 文档里的“关于 Go2 → 雷达”章节仍写着
“每秒 21600 次采样”，这是 L1 时代的旧参数；产品页和 L2 单品页已经明确
当前款是 L2。真机以实际固件和话题数据为准，不要只凭旧文档判断。

### 1.1 机载雷达的数据接口

无论机身是 L1 还是 L2，SDK 暴露的话题接口基本一致：

```text
rt/utlidar/cloud               # 原始点云，frame_id=utlidar_lidar
rt/utlidar/cloud_deskewed      # 去运动畸变点云，frame_id=odom
rt/utlidar/height_map_array    # 机载高度地图，128×128，默认 0.06m
rt/utlidar/range_info          # 前/左/右三个方向的距离
rt/utlidar/switch              # 打开/关闭雷达，ON/OFF
```

在 ROS 2 里订阅时，`rt/` 前缀会消失，对应：

```text
/utlidar/cloud
/utlidar/cloud_deskewed
/utlidar/height_map_array
/utlidar/range_info
```

### 1.2 到现场先确认三件事

```bash
# 1. 话题是否存在
ros2 topic list | grep utlidar

# 2. 点云实际频率
ros2 topic hz /utlidar/cloud_deskewed

# 3. 点云字段和每帧点数
ros2 topic echo --no-arr /utlidar/cloud_deskewed --once
```

看输出里的 `width`、`height` 和 `fields`：

- `fields` 必须有 `x`、`y`、`z`；
- 反光条算法还需要 `intensity`；
- L2 约 11500 点/圈，且周向约 5.55Hz；
- L1 约 2000 点/圈，且周向约 11Hz；
- 点云话题的实际发布频率以 `ros2 topic hz` 为准，不要只抄产品页。

> **注意：L2 的单圈点更多，但转得更慢。**
>
> L2 的有效点频是 L1 的约 3 倍，但周向扫描从 11Hz 降到 5.55Hz。
> 也就是说，两圈之间的时间间隔变长了。巡检速度必须重新保守标定，
> 不要因为“L2 更清晰”就默认可以跑得更快。建议先按 0.3～0.6 m/s
> 验证急停距离，再逐步提高。

雷达型号本身**不改变代码架构**。无论 L1 还是 L2，都走
`pointcloud_to_laserscan → /scan → slam_toolbox/AMCL/Nav2` 这条链路，
差异只在点云密度、处理开销和少量参数标定。

---

## 2. 别把两个“SLAM”混为一谈

### 2.1 APP 里的“3D 激光建图”

这是消费级功能：

- 使用机身自带的 L2；
- 通过 Go2 专属 APP 建点云地图；
- 在 APP 里指定行进路径，让 Go2 沿路径自主移动；
- 地图、路径和任务都在 APP/机身闭环里，**没有开放 SDK 接口**给
  我们做“到点播语音”“反光条告警”“虚拟墙注入”这类二次开发
  （至少官方公开文档里没有）。

它适合做演示，不适合接入我们的自动巡检业务。

### 2.2 官方“SLAM 导航服务接口”（`slam_operate`）

这是本文讨论的、可以在 EDU 上二次开发的导航服务：

- 运行在狗背拓展坞 PC 上（默认 `192.168.123.18`）；
- 使用外置 MID-360 或 XT16（默认雷达 IP `192.168.123.20`）；
- 通过 Unitree SDK2 的 JSON 服务调用；
- 官方文档明确写着：

> “SLAM 导航服务接口仅支持带拓展坞和 Unitree 官方购入激光雷达
> （MID-360/XT16）版本的 EDU 机器狗！！！”

也就是说：**机载 L2 可以给 APP 建图，也可以给我们自己的 ROS 2 程序做
SLAM/Nav2，但官方 `slam_operate` 不消费机载 L2。**

---

## 3. 按硬件配置分三种方案

| 配置 | 建图/定位/导航 | 反光条当墙并绕行 | 说明 |
| --- | --- | --- | --- |
| A. EDU，仅有内置 L2 | 我们自己的 `slam_toolbox + AMCL + Nav2` | ✅ 可以 | 就是当前仓库方案，只需把文档里的 L1 改成 L2/L1 兼容写法 |
| B. EDU + 拓展坞 + 外置 MID-360/XT16，走官方服务 | 官方 `slam_operate`，PCD 地图，1102 位姿导航 | ❌ 最多只能暂停/告警 | 巡检和语音我们自己写，官方只负责定位和走路 |
| C. EDU + 拓展坞 + 外置雷达，但坚持自己的 Nav2 | 自己跑外置雷达驱动 + Nav2 | ✅ 可以 | 自由度最高，工程量也最大 |

### 方案 A：只有 EDU + 内置 L2（当前实现）

```text
机载 L2（rt/utlidar/cloud_deskewed）
   │
   ▼
pointcloud_to_laserscan
   │
   ▼
/scan
   ├── slam_toolbox           建图
   └── AMCL + Nav2            定位/规划
              │
              ▼
        /cmd_vel
              │
              ▼
   /api/sport/request  →  Go2 行走
              │
              ▼
   patrol_manager → /robot/say → voice_player
```

这条路线**不需要外置雷达，也不需要拓展坞**（EDU 的网口直连即可）。
反光条检测结果可以转成 `PointCloud2`，作为 Nav2 `obstacle_layer`
的 observation source，真正进入代价地图，从而让 Nav2 把它当成墙来绕行。

需要做的调整：

1. 文档和注释里把“L1”改成“机载 4D 雷达（L1/L2，以实机为准）”；
2. 现场用 `ros2 topic hz` 确认真实频率；
3. 根据 L2 的点数和量程，检查 `perception.yaml` 里的
   `range_max`、`min_height`、`max_height` 和 `angle_increment`；
4. 点云密度变高后，控制电脑 CPU 占用会上升，必要时降采样；
5. L2 周向仅 5.55Hz，自动巡检速度先限制在 0.3～0.6 m/s，
   确认急停距离后再提速。

### 方案 B：EDU + 拓展坞 + 外置 MID-360/XT16，走官方服务

硬件和固件前提：

- Go2 EDU；
- 拓展坞（Orin Nano 8GB 或 Orin NX 16GB）；
- 宇树官方购入的 MID-360 或 XT16；
- 固件 ≥ 1.1.7；
- 拓展坞里的 `unitree_slam` 模块更新到官方 SLAM 版本；
- 使用前按要求修改 `param.yaml` 的 `lidar_type`、`lidar_ysn` 和
  `gridmap_config/config.yaml` 的 `T_Dog2lidar` 外参；
- 拓展坞电源模式设为 `0: MAXN`，并按文档调整 DDS 收发缓冲区。

官方服务的已开放接口：

| API ID | 功能 | 关键输入 |
| --- | --- | --- |
| 1801 | 开始建图 | `slam_type: indoor`（固定值） |
| 1802 | 结束建图 | PCD 保存地址 |
| 1804 | 初始化位姿 | x/y/z、四元数、PCD 地址 |
| 1102 | 位姿导航 | `targetPose`、`mode`、`speed` |
| 1201 | 暂停导航 | 无 |
| 1202 | 恢复导航 | 无 |
| 1901 | 关闭 SLAM | 无 |

导航约束（官方文档原文口径）：

- 适用场景：**小于 25m×25m、特征丰富、静态室内平地**；
- 目标点与当前位置距离**不超过 10m**；
- 机器人沿直线行走，不会像 Nav2 那样做任意全局路径规划；
- 速度范围：Go2 为 **0.2～0.8 m/s**；
- 障碍物高度不要低于 **20cm**；
- `mode=1` 停障；`mode=0` 绕障，但要求可行驶宽度 ≥ 0.8m，
  且障碍物宽度 < 0.5m；
- 官方建议连续使用不超过 30 分钟，避免拓展坞 CPU 降频导致定位异常；
- `unitree_slam` 与 ROS 2 环境存在 DDS 二进制冲突，
  同一进程/同一终端不要先 source ROS 2。

官方服务发布的 ROS 2/DDS 话题：

```text
rt/unitree/slam_mapping/points      # 建图实时点云
rt/unitree/slam_mapping/odom        # 建图实时里程计
rt/unitree/slam_relocation/points   # 定位实时点云
rt/unitree/slam_relocation/odom     # 定位实时里程计
rt/slam_info                        # 状态广播（位姿、障碍、进度等）
rt/slam_key_info                    # 单次任务结果（is_arrived 等）
```

我们的巡检层需要这样改：

1. 新增一个独立进程 `unitree_slam_client`：
   - 不 source ROS 2；
   - 用 Unitree SDK2 调用 `slam_operate` 的 1801/1802/1804/1102/1201/1202/1901；
   - 把“开始/结束/成功/失败/到达”转换成我们内部事件；
2. `patrol_manager` 增加 `backend=official|nav2` 参数：
   - `official`：每个航点调 1102，等 `rt/slam_key_info` 的 `is_arrived=true`；
   - `nav2`：保持当前 `BasicNavigator` 流程；
3. 语音层不变：
   - 收到到达事件后仍然发布 `/robot/say`；
   - `voice_player` 继续播放预录 WAV；
4. 地图管理改成 PCD 文件路径，不再是 `map.yaml`；
5. 航点必须重新设计：
   - 长走廊拆成多个 ≤10m 的短航点；
   - 窄于 0.8m 的通道不要依赖 `mode=0` 绕障；
   - 起始定位点要回到建图起点附近。

反光条在方案 B 里的能力边界：

- 机载 L2 是否在官方 SLAM 服务运行时仍持续发布点云，需要现场确认；
  如果仍发布，我们可以在控制电脑上用 ROS 2 并行做反光条检测；
- 但官方 `slam_operate` 的内部规划器**不接受我们的 costmap 或虚拟墙**；
- 所以检测到反光条后只能：
  1. 调 1201 暂停导航；
  2. 播放告警语音；
  3. 等待人工确认后调 1202 恢复；
  4. 或者直接终止当前任务。
- **做不到**“把它当墙，然后让官方导航自动绕过去”。

还有一个容易忽略的坐标系问题：

- `/utlidar/cloud_deskewed` 的 `frame_id` 是机身里程计 `odom`；
- 官方 SLAM 服务输出的定位/地图在它自己的
  `rt/unitree/slam_relocation/...` 体系里；
- 两者不是自动对齐的。即使检测到了反光条，想把它标到官方 PCD 地图上，
  也要额外做坐标变换和一致性校验，不能直接把 `odom` 下的线段塞进去。

### 方案 C：EDU + 拓展坞 + 外置雷达，但坚持自己的 Nav2

如果既想要外置雷达的稳定性，又必须保留反光条虚拟墙：

- 不要在拓展坞上启动官方 `unitree_slam`；
- 自己在外置雷达上跑驱动（MID-360 用 Livox 驱动，XT16 用对应驱动）；
- 点云 → `/scan` 或 3D 代价地图 → `slam_toolbox`/AMCL/Nav2；
- 反光条检测结果照常注入 `obstacle_layer`；
- 拓展坞只当普通 Linux 电脑用，或者干脆把雷达接到控制电脑上。

这条路线的工程量和风险最高，但导航自由度也最高。

---

## 4. 推荐决策

1. **只要“到点巡检 + 到点语音”，并且场地在 25m×25m 静态室内**：
   有拓展坞和官方雷达的话，方案 B 最省事，定位/导航交给官方服务。
2. **必须“反光条当墙 + 自动绕行”**：方案 A 或方案 C，不要用官方
   `slam_operate`。
3. **目前只有 EDU + 机载 L2**：方案 A 完全可做，不需要买外置雷达；
   先按现有仓库把建图、导航、巡检、语音跑通，反光条作为可选增强。
4. **不要在同一台拓展坞上同时跑官方 `unitree_slam` 和我们自己的
   ROS 2/Nav2 全套**：DDS 环境冲突，现场很难保证稳定。

---

## 5. 需要向宇树或销售确认的问题

1. 你手上的 EDU 具体包含哪些配件：拓展坞？MID-360/XT16？还是只有机载 L2？
2. 官方 `slam_operate` 的 1102 是纯位姿导航，还是必须先用官方工具
   生成拓扑点/路径文件？
3. 官方服务运行时，机载 L2 是否仍然持续发布点云？能否与
   外置雷达同时工作？
4. 反光条的实际 `intensity` 数值范围是多少？需要现场采样标定。
5. 拓展坞的散热和续航能否支持你们的巡检时长？官方建议连续使用
   不超过 30 分钟。

---

## 6. 官方资料索引

- Go2 产品页（L2、APP 3D 建图）：
  https://www.unitree.com/cn/go2
- L2 单品页（64000 点/秒等参数）：
  https://www.unitree.com/cn/L2
- Go2 SDK 开发指南：
  https://support.unitree.com/home/zh/developer
- LiDAR 服务接口（`rt/utlidar/cloud` 等）：
  https://support.unitree.com/home/zh/developer/LiDAR_service
- SLAM 服务简介（要求外置 MID-360/XT16）：
  https://support.unitree.com/home/zh/developer/SLAM%20Application
- SLAM 导航服务接口（`slam_operate` API）：
  https://support.unitree.com/home/zh/developer/SLAM%20and%20Navigation_service
- 拓展坞配置与模块更新：
  https://support.unitree.com/home/zh/developer/module_update

---

## 附录：官方文档核对记录（2026-10-14）

通过宇树官方文档中心（support.unitree.com）逐项核对以下正文：

| 官方文档 | 核对内容 |
| --- | --- |
| Go2 SDK · 快速开始 | 电脑网卡设 123 网段，建议 `192.168.123.222`；**禁止**设为 `192.168.123.161`（机载电脑地址） |
| Go2 SDK · ROS2 服务接口 | 话题 `lf/sportmodestate` / `sportmodestate`；网线直连 + 电脑 IP `192.168.123.99`；Ubuntu 22.04 + Humble 可用 |
| Go2 SDK · LiDAR 服务接口 | `rt/utlidar/cloud`（`utlidar_lidar` 系）、`rt/utlidar/cloud_deskewed`（`odom` 系）；ROS2 下 `rt/` 前缀自动消失 |
| Go2 SDK · SLAM 导航服务接口 | "**仅支持带拓展坞和 Unitree 官方购入激光雷达（MID-360/XT16）版本的 EDU 机器狗**"；拓展坞 PC `192.168.123.18`、雷达 `192.168.123.20`；场景限制 ≤25m×25m 静态室内平地；导航速度 0.2–0.8 m/s |
| L2 SDK 开发指南 | L2：周向 5.55Hz、竖直 216Hz、有效 64000 点/秒（≈11500 点/圈）、0.05–30m、360°×96°（负角度模式默认开） |
| Go2 SDK · 网络服务 | Wi-Fi STA 与 Wi-Fi AP **只会开启一个**；AP 名称/密码在 Go2 App 里配置；4G 自动兜底 |

核对结论：

1. **本仓库全部关键参数与官方一致**：网络段、话题名、L2 参数、slam_operate 硬件门槛均有官方出处（见上表）。
2. **官方文档自身有一处旧数据**：Go2 SDK「关于 Go2 → 雷达」章节仍写 L2 为"每秒 21600 次采样"（L1 旧参数），与 L2 单品文档的 64000 点/秒矛盾。以 L2 文档/产品页为准，现场以话题实测为准。
3. **IP 建议**：`192.168.123.99` 与 `192.168.123.222` 均可（不同官方文档口径），共同禁令是不得用 `192.168.123.161`。
4. **WiFi AP 直连**（本仓库默认接法）：官方未给出 AP 模式下客户端 IP 分配细节，现场实测（连上热点后 `ip addr` 看是否自动获得 123 网段地址，没有就手动设）。
5. 官方强调 "Unitree SDK2（C++ 程序）与 ROS2 环境存在二进制冲突" —— 只影响在拓展坞 PC 上运行 `slam_operate` 等 SDK2 程序的场景；本仓库全程 ROS2，不受影响。


