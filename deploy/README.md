# deploy/ —— 生产交付工具包

把开发机上的工作空间，变成「生产机器狗上开机自启、崩了自动拉起、能一键升级/回滚」的部署。

## 交付链路

```text
开发机(WSL)                          生产机(机器狗配套工控机 / 笔记本)
────────────────────────────          ────────────────────────────────────────
main 合并 → 打 tag v0.1.0
  │
  ├─ 在线: git push ──────────────────►  sudo deploy/install.sh --repo <URL> --ref v0.1.0
  │
  └─ 离线: make_release_bundle.sh ────►  sudo deploy/install.sh --bundle …tar.gz --ref v0.1.0 --no-apt
                                            │
                                            ├─ /opt/go2/releases/v0.1.0  代码 + 编译产物
                                            ├─ /opt/go2/current → 软链，回滚只切它
                                            └─ systemd: go2-patrol@<站点>
                                            │
                                        站点标定产物 ──►  sudo deploy/apply_site.sh --site-dir <站点目录> --name <站点名>
                                                          sudo systemctl enable --now go2-patrol@<站点名>
```

生产机目录布局：

```text
/opt/go2/
├── releases/v0.1.0/       每个版本一份完整工作空间（含 install/ 编译产物）
├── releases/v0.2.0/
├── current -> releases/v0.2.0
└── sites/
    └── office/            站点资产（升级不动它）
        ├── map.yaml  map.pgm
        ├── waypoints.yaml
        ├── site.yaml        现场参数（人手编辑）
        ├── site.env         生成文件，run_site.sh 会 source
        ├── nav2_params.yaml 生成文件，含 AMCL 初始位姿
        └── audio/*.wav
/etc/go2/site.env          机器级配置：GO2_SITES / GO2_IFACE
/etc/systemd/system/go2-patrol@.service
/var/log/go2/              巡检日志（ROS 日志目录）
```

## 文件清单

| 文件 | 作用 | 谁用 |
|---|---|---|
| `apt-deps.txt` | 系统依赖清单（唯一来源） | `install.sh` / `scripts/bootstrap.sh` |
| `go2_env.sh` | 环境定义：自动推导工作空间、网卡、`CYCLONEDDS_URI` | 开发机 `scripts/setup.sh` 与生产机共用 |
| `preflight.sh` | 启动前自检（ROS/编译/站点资产/网卡/狗连通） | systemd `ExecStartPre`、人工 |
| `run_site.sh` | 启动某站点巡检（拼接站点参数后 `ros2 launch`） | systemd `ExecStart` |
| `apply_site.sh` | 落地站点资产 + 生成参数 + 自检 | 交付 / 换地图时 |
| `install.sh` | 目标机安装、升级、回滚、装 systemd | 生产机（sudo） |
| `uninstall.sh` | 停服务、删单元（`--purge` 才删数据） | 生产机（sudo） |
| `make_release_bundle.sh` | 打离线交付包（含官方 `unitree_ros2` 源码） | 开发机 |
| `systemd/go2-patrol@.service` | 站点实例服务模板 | `install.sh` 落地 |
| `tools/render_site.py` | 由 `site.yaml` 生成 `nav2_params.yaml` / `site.env` + 校验 | `apply_site.sh` |
| `tools/make_blank_map.py` | 生成空白站点骨架，台架自检用 | 开发机 |
| `site.example/` | 站点资产模板（照它准备现场资产） | 交付准备 |

## 站点参数（现场只改 `site.yaml`）

`site.yaml` 是站点差异的唯一入口，`apply_site.sh` 会据此生成
`nav2_params.yaml`（AMCL 固定起点、`robot_radius`、到点容差）与 `site.env`（其余参数）。

| `site.yaml` 字段 | 落到哪里 | 说明 |
|---|---|---|
| `initial_pose` | `nav2_params.yaml` 的 `amcl.set_initial_pose/initial_pose` | 开机自启能自动定位的关键，写错会发散 |
| `robot_radius` / `patrol.arrival_tolerance` | 代价地图 / Nav2 `goal_checker` | 按机身与现场留余量 |
| `patrol.loops` | `patrol_manager.loops` | `1`=跑完一圈收工（服务仍 active）；`0`=无限循环 |
| `patrol.battery_low` / `return_home` | 低电量返航与收工返航 | 返航点取 `home: true` 的那个点位 |
| `patrol.yaw_mode` | 到点朝向策略 | `fixed`/`travel`/`next`，路口用 `travel` 少原地转 |
| `patrol.goal_retry_limit` / `goal_retry_delay` / `on_goal_failure` | 导航失败策略 | 同一目标连续失败 3 次（含首次，即最多尝试 3 次）后按 `on_goal_failure` 处理，每次间隔 5s |
| `safety.*` | `twist_bridge` 的限速与急停阈值 | 速度上限、急停/停车/减速距离、`cmd_vel_timeout`；三个距离必须递增，写反会被自检拦下 |
| `env.GO2_BASE_FOOTPRINT_TF` | `tf_chain_guard.mode` | `auto`（默认，探测后按需补 `base_footprint → base_link`）/ `always` / `never` |
| `env.GO2_REQUIRE_SCAN` | `twist_bridge.require_scan` | 默认 `true`：`/scan` 断流即停车保护；无雷达台架才设 `false` |
| `env.*`（其它） | 直接写进 `site.env` 并 export | 例如 `ROS_DOMAIN_ID` |

改完执行 `sudo deploy/apply_site.sh --site-dir <站点目录> --name <站点名>`，
它会重生成参数并跑一次 `preflight.sh` 自检。

> 落地是**先预检源目录、再覆盖站点目录**的：`apply_site.sh` 第一步用
> `render_site.py --check-only` 把源目录的 `site.yaml` 查一遍（安全阈值、未知键、
> 段类型），不通过就直接退出、**一个文件都不动**。以前是先把资产拷过去再校验，
> `site.yaml` 写错时会把已上线站点的配置换成坏的那份，而参数文件还是旧的——
> 目录进入半成品状态，现场排查方向全错。
>
> 顺便提醒 `iface` 的优先级：站点 `site.yaml` 的 `iface` 写进站点 `site.env` 后，
> `preflight.sh` 也会按它校验网卡（不再只看机器级的 `/etc/go2/site.env`），
> 所以网卡名写错会当场 `[FAIL]`，而不是"自检通过但收不到狗的数据"。

> `site.env` 由 `run_site.sh` 以 `set -a` 读入后再 `exec ros2 launch`，所以里面的变量
> （包括 `env:` 段的自定义变量）确实会进到 launch 子进程的环境里——
> `driver.launch.py` 读的就是这些 `GO2_*` 变量——规则是 `GO2_` + launch 参数名大写
> （`max_linear` ← `GO2_MAX_LINEAR`、`require_scan` ← `GO2_REQUIRE_SCAN`、
> `base_footprint_tf` ← `GO2_BASE_FOOTPRINT_TF`），所以 `set -a` 这一步不能少。
>
> 覆盖优先级：巡检与安全阈值（`patrol.*` / `safety.*`）在 `site.env` 里写成
> `GO2_X=${GO2_X:-值}`，所以**命令行/环境变量优先**——现场临时降速验证可以直接
> `GO2_MAX_LINEAR=0.4 deploy/run_site.sh office`；网卡与狗地址是裸赋值，**站点 site.yaml 优先**
> 于机器级 `/etc/go2/site.env`。改 `site.yaml` 后仍需重跑 `apply_site.sh` 才生效。

## 一次性验收（裸机跑通）

```bash
# 0. 准备一台 Ubuntu 22.04 + ROS 2 Humble
# 1. 在线安装（会自动编译；离线包见手册 3.2）
git clone <你的仓库> && cd go2_official_ws
sudo deploy/install.sh --iface enp3s0

# 2. 台架验证：site.example 自带 10m 空地占位图，不需要真机/建图产物
sudo deploy/apply_site.sh --site-dir deploy/site.example --name demo
sudo systemctl enable --now go2-patrol@demo
journalctl -u go2-patrol@demo -f

# 3. 真机现场：换成标定好的站点资产
sudo deploy/apply_site.sh --site-dir /media/usb/site-office --name office
sudo systemctl enable --now go2-patrol@office
```

> 想换房间尺寸再自测时用 `python3 deploy/tools/make_blank_map.py <目录> --size 15` 生成骨架。

> 台架骨架用空白房间地图，AMCL 与 Nav2 会正常起来；机器狗不在线时只有驱动桥接报错，
> 属预期行为，用来验证「systemd → 环境 → launch → 参数」这条链路。

## 升级与回滚

```bash
# 升级（新版本目录 + 切 current + 重启在跑的站点服务）
sudo /opt/go2/current/deploy/install.sh --ref v0.2.0

# 回滚（秒级，站点资产不动）
sudo /opt/go2/current/deploy/install.sh --rollback v0.1.0
```

编译在版本目录里独立完成，回滚只是切软链，所以「升级失败」不会影响正在跑的那一版。

## 生产机常用命令

```bash
systemctl status go2-patrol@office                 # 状态
journalctl -u go2-patrol@office -f                 # 实时日志
sudo systemctl restart go2-patrol@office           # 重启
sudo /opt/go2/current/deploy/preflight.sh office   # 手动自检
ros2 service call /patrol/start std_srvs/srv/Trigger   # 取消暂停（启动时是自动开跑的；巡检结束后调它无效）
```

详细交付流程、现场标定与验收清单见
[`docs/真机部署与使用手册.md`](../docs/真机部署与使用手册.md)。
