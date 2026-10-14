from glob import glob
import os
from setuptools import setup

package_name = "go2_patrol_reflective"

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
    description="Reflective tape virtual wall detector for testing.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "reflective_wall_detector = "
            "go2_patrol_reflective.reflective_wall_detector:main",
        ],
    },
)
