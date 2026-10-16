from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            # 离线 demo 用 false 彻底关闭真机运动输出（twist_bridge 不订阅
            # /cmd_vel、不建 /api/sport/request 发布者）。默认 true 保持真机行为不变。
            DeclareLaunchArgument(
                "sport_output",
                default_value="true",
                description="是否允许 twist_bridge 向 /api/sport/request 输出真机运动命令。",
            ),
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
                parameters=[
                    {
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "enable_sport_output": LaunchConfiguration("sport_output"),
                    }
                ],
                output="screen",
            ),
        ]
    )
