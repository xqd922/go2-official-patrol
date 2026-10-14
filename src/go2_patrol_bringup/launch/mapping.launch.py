from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("go2_patrol_bringup")
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "perception.launch.py"))
    )
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("go2_patrol_driver"), "launch", "driver.launch.py")
        )
    )
    slam = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        parameters=[{"use_sim_time": False}],
        output="screen",
    )
    return LaunchDescription([perception, driver, slam])
