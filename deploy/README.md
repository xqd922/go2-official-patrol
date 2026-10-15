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

## 一次性验收（裸机跑通）

```bash
# 0. 准备一台 Ubuntu 22.04 + ROS 2 Humble
# 1. 在线安装
git clone <你的仓库> && cd go2_official_ws
sudo deploy/install.sh --iface enp3s0

# 2. 台架验证：造一份空白站点骨架（不需要真机）
python3 deploy/tools/make_blank_map.py /tmp/site-demo --name demo
sudo deploy/apply_site.sh --site-dir /tmp/site-demo --name demo
sudo systemctl enable --now go2-patrol@demo
journalctl -u go2-patrol@demo -f

# 3. 真机现场：换成标定好的站点资产
sudo deploy/apply_site.sh --site-dir /media/usb/site-office --name office
sudo systemctl enable --now go2-patrol@office
```

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
ros2 service call /patrol/start std_srvs/srv/Trigger   # 手动触发巡检（需先 source 环境）
```

详细交付流程、现场标定与验收清单见 [`docs/生产交付手册.md`](../docs/生产交付手册.md)。
