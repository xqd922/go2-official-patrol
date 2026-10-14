from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("go2_patrol_bringup")
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("go2_patrol_driver"), "launch", "driver.launch.py")
        )
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "navigation.launch.py")),
        launch_arguments={"map": LaunchConfiguration("map")}.items(),
    )
    voice = Node(package="go2_patrol_patrol", executable="voice_player", output="screen")
    timer = Node(
        package="go2_patrol_patrol",
        executable="patrol_manager",
        parameters=[{"waypoints_file": LaunchConfiguration("waypoints_file")}],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=os.path.join(bringup, "maps", "map.yaml")),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=os.path.join(bringup, "config", "waypoints.yaml"),
            ),
            driver,
            navigation,
            voice,
            timer,
        ]
    )
