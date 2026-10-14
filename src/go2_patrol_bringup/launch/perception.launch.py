from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    params = os.path.join(get_package_share_directory("go2_patrol_bringup"), "config", "perception.yaml")
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_lidar",
                arguments=["0", "0", "0.13", "0", "0", "0", "base_link", "utlidar_lidar"],
                output="screen",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                remappings=[("cloud_in", "/utlidar/cloud_deskewed"), ("scan", "/scan")],
                parameters=[params],
                output="screen",
            ),
        ]
    )
