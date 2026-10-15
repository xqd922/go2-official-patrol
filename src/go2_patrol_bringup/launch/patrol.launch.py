from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = get_package_share_directory("go2_patrol_bringup")
    patrol = get_package_share_directory("go2_patrol_patrol")

    # 感知与驱动桥接（odom/battery/twist）统一由 navigation.launch.py 引入，
    # 这里不再重复 include driver，否则同名节点会启动两次（odom/TF 重复发布）。
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "navigation.launch.py")),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )
    voice = Node(
        package="go2_patrol_patrol",
        executable="voice_player",
        parameters=[{"audio_dir": LaunchConfiguration("audio_dir")}],
        output="screen",
    )
    timer = Node(
        package="go2_patrol_patrol",
        executable="patrol_manager",
        parameters=[
            {
                "waypoints_file": LaunchConfiguration("waypoints_file"),
                "loops": ParameterValue(LaunchConfiguration("loops"), value_type=int),
                "return_home": ParameterValue(LaunchConfiguration("return_home"), value_type=bool),
                "battery_low": ParameterValue(LaunchConfiguration("battery_low"), value_type=float),
                "arrival_tolerance": ParameterValue(
                    LaunchConfiguration("arrival_tolerance"), value_type=float
                ),
            }
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=os.path.join(bringup, "maps", "map.yaml")),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup, "config", "nav2_params.yaml"),
                description="Nav2 参数文件；生产站点由 apply_site.sh 生成站点副本后传入",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=os.path.join(bringup, "config", "waypoints.yaml"),
            ),
            DeclareLaunchArgument(
                "audio_dir",
                default_value=os.path.join(patrol, "audio"),
                description="到点语音目录，文件名 <speech_id>.wav（站点可覆盖）",
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="是否启动 RViz；生产机建议 false"
            ),
            DeclareLaunchArgument("loops", default_value="1"),
            DeclareLaunchArgument("return_home", default_value="true"),
            DeclareLaunchArgument("battery_low", default_value="20.0"),
            DeclareLaunchArgument("arrival_tolerance", default_value="0.30"),
            navigation,
            voice,
            timer,
        ]
    )
