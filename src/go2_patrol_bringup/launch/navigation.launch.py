import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("go2_patrol_bringup")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "perception.launch.py"))
    )
    # driver.launch.py 只在这里 include 一次；外层（patrol.launch.py）通过
    # driver:=false 抑制这次 include，避免 odom_bridge/TF/twist_bridge 起两套。
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("go2_patrol_driver"), "launch", "driver.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("driver")),
    )
    map_yaml = LaunchConfiguration("map")
    # params_file 默认仍是共享的 config/nav2_params.yaml（真机行为不变）。
    # OFFLINE 链路通过 launch 参数传入"离线专用"参数文件：在共享文件基础上
    # 只收紧 goal_checker 容差，使离线巡检真正走到航点附近，而不是踩在
    # 0.30m 容差边缘就报到达。真机链路不传该参数，仍用原文件。
    params_file = LaunchConfiguration("params_file")

    # localization:=true（真机默认）走 nav2 标准 bringup：map_server + AMCL + 导航栈。
    # localization:=false（离线）只起导航栈 + 自行启动 map_server，不起 AMCL：
    # 离线已经由 fake_go2 提供真值里程计，静态 map->odom 由 outer launch 发布。
    # nav2 的 bringup_launch/localization_launch 把 map_server 与 amcl 写死在同一
    # lifecycle 组里，无法只去掉 AMCL，因此这里按开关选择 include 的目标。
    use_localization = LaunchConfiguration("localization")
    localization_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": "false",
        }.items(),
        condition=IfCondition(use_localization),
    )
    navigation_only_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, "launch", "navigation_launch.py")),
        launch_arguments={
            "params_file": params_file,
            "use_sim_time": "false",
            # use_composition:=False（首字母大写）：把 8 个 Nav2 lifecycle 节点
            # 拆成独立进程，逐个被 lifecycle manager 发现后再激活。合成到一个
            # 容器时，节点与生命周期管理器在同一瞬间互相发现，本机 CycloneDDS 下
            # change_state 服务调用会偶发超时（bt_navigator / smoother_server
            # 激活失败，整个 bringup abort）。独立进程只影响离线链路的启动顺序，
            # 真机链路仍走 nav2 默认的合成模式。
            #
            # 注意大小写：nav2 的 navigation_launch.py 内部用
            # IfCondition(PythonExpression(['not ', use_composition])) 求值，
            # 传小写 "false"/"true" 会被当成 Python 标识符而抛
            # NameError: name 'false' is not defined。必须传 Python 字面量形式。
            "use_composition": "False",
            # autostart:=false：不在节点刚起来时立刻激活。离线链路改由
            # offline_patrol.launch.py 里的 nav2_starter 在确认所有节点服务
            # 就绪后再触发 STARTUP，避免本机 CycloneDDS 发现慢导致的间歇失败。
            "autostart": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("no_localization")),
    )
    # 离线专用 map_server：仍走标准 map_server 生命周期节点，只把
    # yaml_filename 从 launch 参数传进去，不在参数文件里写死绝对路径。
    offline_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[params_file, {"yaml_filename": map_yaml, "use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("no_localization")),
        output="screen",
    )
    offline_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        parameters=[
            # bond_timeout 加大：本机 CycloneDDS 发现较慢，默认 4s 的 bond
            # 心跳在节点刚启动时容易误判超时，导致 map_server 在激活中途被
            # 判为失败、整个 localization bringup abort。
            {"autostart": False, "node_names": ["map_server"], "bond_timeout": 20.0}
        ],
        condition=IfCondition(LaunchConfiguration("no_localization")),
        output="screen",
    )

    navigation = TimerAction(
        period=LaunchConfiguration("startup_delay"),
        actions=[
            localization_include,
            navigation_only_include,
            offline_map_server,
            offline_lifecycle_manager,
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=os.path.join(bringup, "maps", "map.yaml")),
            DeclareLaunchArgument(
                "driver",
                default_value="true",
                description="是否在本 launch 内启动 go2_patrol_driver；由 patrol.launch.py 置为 false 以避免重复启动。",
            ),
            DeclareLaunchArgument(
                "localization",
                default_value="true",
                description="是否使用 AMCL 做定位（真机默认 true）。",
            ),
            DeclareLaunchArgument(
                "no_localization",
                default_value="false",
                description="离线链路置为 true：不起 AMCL，只起导航栈与 map_server。",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup, "config", "nav2_params.yaml"),
                description="Nav2 参数文件；OFFLINE 传离线专用（收紧 goal 容差）版本，真机默认不变。",
            ),
            DeclareLaunchArgument(
                "startup_delay",
                default_value="3.0",
                description="Nav2 延迟启动秒数；OFFLINE 传 0，避免 nav2_starter 先连到同 domain 的旧实例。",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="是否在本 launch 内启动 RViz；离线链路需要自带配置的 RViz，故置为 false。",
            ),
            perception,
            driver,
            navigation,
            rviz,
        ]
    )
