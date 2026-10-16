"""离线定点巡检链路（无需 Go2 实机）。

与真机 patrol.launch.py 的差别全部集中在“让 Nav2 在没有实机、没有 AMCL
的情况下也能真正闭环跑起来”，真机链路本身不受影响：

1. 定位：离线不用 AMCL，改为发布一条静态 map->odom 变换。
   fake_go2 的 /odom 是仿真真值（由 /cmd_vel 积分得来），odom_bridge 把它
   变成 odom->base_footprint；再补一条 map->odom，就得到唯一一条完整链：

       map -> odom -> base_footprint -> base_link -> utlidar_lidar

   之所以不用 AMCL：AMCL 会拿离线雷达去和地图做匹配，而离线房间与保存的
   地图并不完全一致，于是 AMCL 会不断“修正” map->odom，与 odom_bridge
   发布的真值互相打架，RViz 里表现为点云/地图持续抖动。离线既然已经有真值
   里程计，直接固定 map->odom 即可，定位链只有一个写者。

2. 运动隔离：driver 以 sport_output:=false 启动，twist_bridge 既不订阅
   /cmd_vel，也不创建 /api/sport/request 发布者，确保离线运行绝不会向真实
   Go2 发指令。
"""
import os
import shutil
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _repo_root(share_dir):
    """从 install/share/<pkg> 反推仓库根，不写死用户名或绝对路径。"""
    return os.path.abspath(os.path.join(share_dir, "..", "..", "..", ".."))


def _python_for_ros():
    """挑一个能 import rclpy 的解释器，优先系统 python3。

    常见坑：conda 的 python3 在 PATH 前面，但它没有 ROS 的 rclpy，
    fake_go2 会启动即崩。这里优先 /usr/bin/python3（ROS 2 Humble 的
    系统解释器），不存在时回退到 PATH 里的 python3。
    """
    for candidate in ("/usr/bin/python3", shutil.which("python3")):
        if candidate and os.path.exists(candidate):
            return candidate
    return "python3"


def _default_map(share_dir):
    """离线默认地图：优先 maps/map.yaml，否则挑第一张已保存的地图。

    save_map.sh 生成的是 <名字>.yaml（例如 offline_room.yaml），并不会改名成
    map.yaml。若默认值写死 maps/map.yaml，而仓库里只有 offline_room.yaml，
    map_server 会因 “bad file .../map.yaml” 直接 configure 失败，整套
    lifecycle 起不来。这里在启动时按实际存在的文件解析，避免写死地图名。
    """
    maps_dir = os.path.join(share_dir, "maps")
    preferred = os.path.join(maps_dir, "map.yaml")
    if os.path.isfile(preferred):
        return preferred
    if os.path.isdir(maps_dir):
        for name in sorted(os.listdir(maps_dir)):
            if name.endswith(".yaml"):
                return os.path.join(maps_dir, name)
    # 都不存在时仍返回惯用名，让 map_server 给出清晰的缺失路径报错。
    return preferred


def _offline_params_file(bringup_share):
    """在共享 nav2_params.yaml 基础上生成"离线专用"参数文件。

    只改一处：收紧 controller_server 的 goal_checker 容差。
    原因：共享参数里 xy_goal_tolerance=0.30，离线 RPP 控制器会在距航点约
    0.28m 处就判定到达（真实观测 x=1.719，而 equipment_zone 在 x=2.0），
    视觉上像是"没走到就报成功"。离线演示希望机器人真正贴近航点，故把容差
    收紧到 0.10。真机链路不经过本函数，仍使用原始共享文件，行为不变。

    单一事实来源：以共享文件为基准，程序化派生，避免手抄整份 YAML 造成漂移。
    """
    src = os.path.join(bringup_share, "config", "nav2_params.yaml")
    with open(src, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    controller = data.get("controller_server", {}).get("ros__parameters", {})
    goal_checker = controller.get("goal_checker")
    if isinstance(goal_checker, dict):
        goal_checker["xy_goal_tolerance"] = 0.10
        goal_checker["yaw_goal_tolerance"] = 0.20
    fd, path = tempfile.mkstemp(prefix="nav2_params_offline_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
    return path


def generate_launch_description():
    bringup = get_package_share_directory("go2_patrol_bringup")
    driver_share = get_package_share_directory("go2_patrol_driver")
    fake_script = os.path.join(_repo_root(bringup), "tools", "offline_demo", "fake_go2.py")
    offline_params = _offline_params_file(bringup)

    fake_go2 = ExecuteProcess(
        cmd=[_python_for_ros(), fake_script],
        name="fake_go2",
        output="screen",
    )

    # 离线补链 TF：odom_bridge 只发布 odom -> base_footprint，
    # 真机上 base_footprint -> base_link 由机器人自身提供。离线没有机器人，
    # 缺这一环会让 utlidar_lidar -> base_footprint 断链，
    # pointcloud_to_laserscan 因查不到变换而丢弃所有点云（/scan 为空）。
    base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_footprint_to_base_link",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"],
        output="screen",
    )

    # 离线定位：静态 map -> odom。odom 系与 map 系在离线场景约定重合，
    # 机器人真实位姿完全由 fake_go2 的仿真里程计提供。
    # 该变换只有一个发布者（本节点），AMCL 在离线链路里不启动。
    map_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    # 离线驱动链：odom_bridge / battery_bridge 仍需要（把假数据转成 /odom + TF），
    # 但 twist_bridge 必须静音，避免污染真实运动接口。
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(driver_share, "launch", "driver.launch.py")),
        launch_arguments={"sport_output": "false"}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "navigation.launch.py")),
        # driver:=false：本文件已统一启动一套 driver，navigation 内不能再起第二套。
        # localization:=false：离线用静态 map->odom 顶替 AMCL，避免两套定位互相打架。
        launch_arguments={
            "map": LaunchConfiguration("map"),
            # 离线专用参数文件：仅收紧 goal 容差，其余与共享文件一致。
            "params_file": offline_params,
            "driver": "false",
            # 离线：不起 AMCL，由本文件发布静态 map->odom；navigation 内部
            # 只起 Nav2 导航栈 + map_server。
            "localization": "false",
            "no_localization": "true",
            # 本文件用自带 demo.rviz 启动 RViz，抑制 navigation 内的默认 RViz。
            "rviz": "false",
            # 离线链路不做 3s 延迟：nav2_starter 与当前 launch 同时启动，
            # 避免它先发现/操作同 domain 里上一次残留的 Nav2 实例。
            "startup_delay": "0.0",
        }.items(),
    )

    # 离线演示用 RViz 配置：Map / LaserScan / TF / 机器人位置 / Nav2 路径与目标。
    # 注意条件用的是 offline_demo_rviz 而不是 rviz：下面 include navigation.launch.py
    # 时传了 rviz:=false（用于抑制 navigation 自带的 RViz），而
    # IncludeLaunchDescription 的 launch_arguments 会通过 SetLaunchConfiguration
    # **写回外层 launch configuration**，把外层的 rviz 覆盖成 false。若这里直接读
    # rviz，父 launch 自己的 RViz 就永远不会启动（即使命令行给了 rviz:=true）。
    # 因此在 include 之前先把用户的选择快照到 offline_demo_rviz，再据此判断。
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(_repo_root(bringup), "tools", "offline_demo", "demo.rviz")],
        condition=IfCondition(LaunchConfiguration("offline_demo_rviz")),
        output="screen",
    )

    # 等 Nav2 节点服务就绪后再统一触发 bringup，避免 DDS 发现慢导致的
    # 间歇性 "Failed to change state / Aborting bringup"。
    nav2_starter = ExecuteProcess(
        cmd=[_python_for_ros(), os.path.join(_repo_root(bringup), "tools", "offline_demo", "nav2_starter.py")],
        name="nav2_starter",
        output="screen",
    )

    voice = Node(package="go2_patrol_patrol", executable="voice_player", output="screen")
    patrol = Node(
        package="go2_patrol_patrol",
        executable="patrol_manager",
        parameters=[
            {
                "waypoints_file": LaunchConfiguration("waypoints_file"),
                # 离线没有 AMCL：跳过对 amcl/get_state 的等待，只等导航栈就绪。
                "localizer": "",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=_default_map(bringup)),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=os.path.join(bringup, "config", "waypoints.yaml"),
            ),
            # rviz:=false 用于无显示器的 CI / 沙箱：离线巡检链路本身不需要可视化。
            DeclareLaunchArgument("rviz", default_value="true",
                                  description="是否启动演示用 RViz（无显示器环境传 false）。"),
            # 在 include navigation.launch.py 之前快照用户的 rviz 选择（见上面注释）。
            SetLaunchConfiguration("offline_demo_rviz", LaunchConfiguration("rviz")),
            fake_go2,
            base_link_tf,
            map_odom_tf,
            driver,
            navigation,
            nav2_starter,
            rviz,
            voice,
            patrol,
        ]
    )

