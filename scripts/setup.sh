#!/usr/bin/env bash
set -euo pipefail

IFACE="${GO2_IFACE:-enp3s0}"
source /opt/ros/humble/setup.bash
source "$HOME/go2_official_ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash"
source "$HOME/go2_official_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
