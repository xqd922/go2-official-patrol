from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="go2_patrol_reflective",
                executable="reflective_wall_detector",
                parameters=[
                    {
                        "cloud_topic": "/utlidar/cloud_deskewed",
                        "intensity_min": 30.0,
                        "height_min": -0.10,
                        "height_max": 0.10,
                        "max_range": 6.0,
                        "cluster_eps": 0.18,
                        "cluster_min_points": 3,
                        "min_line_points": 5,
                        "inlier_distance": 0.08,
                        "required_hits": 3,
                        "wall_half_width": 0.10,
                        "wall_height": 0.7,
                        "max_wall_length": 4.0,
                    }
                ],
                output="screen",
            )
        ]
    )
