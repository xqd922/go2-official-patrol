# RViz 配置目录

自定义的 `.rviz` 配置文件放在这里，由 `navigation.launch.py` 通过 `rviz -d <path>` 引用。

## patrol.rviz

巡检栈默认视图（`navigation.launch.py` 的 `rviz_config` 参数默认值），显示：

| 显示项 | 话题 | 用途 |
| --- | --- | --- |
| Grid / TF | — | 世界网格与坐标树（Fixed Frame = `map`） |
| Map | `/map` | 静态地图 |
| GlobalCostmap / LocalCostmap | `/global_costmap/costmap`、`/local_costmap/costmap` | 代价地图（看膨胀层是否合理） |
| LaserScan | `/scan` | 感知输出 |
| PointCloud2 | `/utlidar/cloud_deskewed` | 原始点云（默认关，排查感知时打开） |
| GlobalPlan / LocalPlan | `/plan`、`/local_plan` | 规划路径与控制器轨迹 |
| AMCLPose / ParticleCloud | `/amcl_pose`、`/particle_cloud` | 定位结果与粒子分布 |
| Odometry | `/odom` | 里程计轨迹 |
| WaypointMarkers | `/waypoints/markers` | 巡检点位与其状态（见下） |

### 点位标记配色

`patrol_manager` 每 0.5s 刷新一次 `/waypoints/markers`（`MarkerArray`，球体 + 悬浮名字），
一眼能看出巡检走到哪一步：

| 颜色 | 含义 |
| --- | --- |
| 绿 | 本圈已到达 |
| 黄 | 当前正在前往的目标 |
| 蓝 | 本圈还没访问 |
| 红 | 返航点（低电量/收工返航时高亮） |

跨圈后会按 `sequence.loop_index` 变化自动清空“已到达”，重新变蓝。

现场用 `deploy/run_site.sh <站点>`（站点 `site.yaml` 里 `rviz: true`）即可看到这个视图；
换视图直接改本文件，或给 launch 传 `rviz_config:=<自己的.rviz>`。
