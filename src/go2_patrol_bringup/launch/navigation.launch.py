from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("go2_patrol_bringup")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "perception.launch.py"))
    )
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("go2_patrol_driver"), "launch", "driver.launch.py")
        )
    )
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    navigation = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_yaml,
                    "params_file": params_file,
                    "use_sim_time": "false",
                }.items(),
            )
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=os.path.join(bringup, "maps", "map.yaml")),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup, "config", "nav2_params.yaml"),
                description="Nav2 参数文件（站点可用 apply_site.sh 生成自己的副本）",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="是否启动 RViz；无显示器的生产机设 false"
            ),
            perception,
            driver,
            navigation,
            rviz,
        ]
    )
