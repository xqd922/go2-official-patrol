from glob import glob
import os
from setuptools import setup

package_name = "go2_patrol_patrol"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "audio"), glob("audio/*.wav")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Go2 Patrol",
    maintainer_email="dev@example.com",
    description="Waypoint inspection and speech playback.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "patrol_manager = go2_patrol_patrol.patrol_manager:main",
            "voice_player = go2_patrol_patrol.voice_player:main",
            "record_waypoint = go2_patrol_patrol.record_waypoint:main",
        ],
    },
)
