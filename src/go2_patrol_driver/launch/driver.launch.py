from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            Node(
                package="go2_patrol_driver",
                executable="odom_bridge",
                output="screen",
            ),
            Node(
                package="go2_patrol_driver",
                executable="battery_bridge",
                output="screen",
            ),
            Node(
                package="go2_patrol_driver",
                executable="twist_bridge",
                parameters=[{"scan_topic": LaunchConfiguration("scan_topic")}],
                output="screen",
            ),
        ]
    )
