#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosidl-generator-dds-idl ros-humble-tf2-tools \
  python3-colcon-common-extensions python3-vcstool

cd "$(dirname "$0")/.."
vcs import src < repos/go2_official_ws.repos
cd src/unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
cd ../..
colcon build --symlink-install
