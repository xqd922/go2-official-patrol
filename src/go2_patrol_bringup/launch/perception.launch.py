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
            # 机载点云带的是机器狗自己的时钟，起机后常与电脑差几分钟。不改时间戳的话
            # pointcloud_to_laserscan 会把每帧都当成"比 TF 缓存还旧"丢掉，表现为
            # /utlidar/cloud_deskewed 有频率但 /scan 一帧没有。原话题保留作旁路。
            Node(
                package="go2_patrol_driver",
                executable="cloud_time_sync",
                name="cloud_time_sync",
                parameters=[
                    {
                        "input_topic": "/utlidar/cloud_deskewed",
                        "output_topic": "/utlidar/cloud_deskewed_ros",
                    }
                ],
                output="screen",
            ),
            # L1 是非重复扫描：单帧只覆盖 242/360 个方位，直接转 LaserScan 会让
            # slam_toolbox 每帧看到"随机缺 1/3 方位"的扫描，位姿来回跳、地图糊成
            # 放射状。这里把最近 10 帧按 TF 变换到同一时刻再合并，方位覆盖补满。
            Node(
                package="go2_patrol_driver",
                executable="cloud_accumulator",
                name="cloud_accumulator",
                parameters=[
                    {
                        "input_topic": "/utlidar/cloud_deskewed_ros",
                        "output_topic": "/utlidar/cloud_accumulated",
                        "base_frame": "base_footprint",
                    }
                ],
                output="screen",
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                remappings=[("cloud_in", "/utlidar/cloud_accumulated"), ("scan", "/scan")],
                parameters=[params],
                output="screen",
            ),
        ]
    )
