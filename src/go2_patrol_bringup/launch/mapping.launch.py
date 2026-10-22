from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("go2_patrol_bringup")
    slam_params = os.path.join(bringup, "config", "slam_toolbox.yaml")
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "perception.launch.py"))
    )
    # 建图只靠用户手里的遥控器走位，软件侧不允许下发运动指令：
    # driver.launch.py 里除 twist_bridge 外的节点（odom_bridge / battery_bridge /
    # tf_chain_guard）建图要用，所以这里不整包去掉，而是把运动闸门关死。
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("go2_patrol_driver"), "launch", "driver.launch.py")
        ),
        launch_arguments={"allow_cmd_vel": "false"}.items(),
    )
    slam = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        # 参数文件在 config/slam_toolbox.yaml，注释里写清了与官方默认值的差异
        # （激光量程、更新间隔、最小位移阈值），现场调参改那一份就够。
        parameters=[slam_params, {"use_sim_time": False}],
        output="screen",
    )
    return LaunchDescription([perception, driver, slam])
