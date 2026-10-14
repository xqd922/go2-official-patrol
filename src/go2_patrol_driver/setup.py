from glob import glob
import os
from setuptools import setup

package_name = "go2_patrol_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Go2 Patrol",
    maintainer_email="dev@local",
    description="Official-Unitree-to-ROS 2 navigation bridges.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odom_bridge = go2_patrol_driver.odom_bridge:main",
            "twist_bridge = go2_patrol_driver.twist_bridge:main",
            "battery_bridge = go2_patrol_driver.battery_bridge:main",
            "sport_command = go2_patrol_driver.sport_command:main",
        ],
    },
)
