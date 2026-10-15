#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
ROOT="$(pwd)"

# 依赖清单统一在 deploy/apt-deps.txt，开发机与生产机安装脚本共用一份。
# 生产机由 deploy/install.sh 装好依赖后以 GO2_SKIP_APT=1 调用这里。
if [ "${GO2_SKIP_APT:-0}" != "1" ]; then
  mapfile -t GO2_APT_PKGS < <(grep -vE '^[[:space:]]*(#|$)' "${ROOT}/deploy/apt-deps.txt")
  if [ "${#GO2_APT_PKGS[@]}" -eq 0 ]; then
    echo "[bootstrap] 错误：deploy/apt-deps.txt 里没有可用依赖项" >&2
    exit 1
  fi
  sudo apt update
  sudo apt install -y "${GO2_APT_PKGS[@]}"
else
  echo "[bootstrap] 跳过 apt 依赖安装（GO2_SKIP_APT=1）"
fi

# 官方 unitree_ros2（已钉版本）；离线交付包已带源码时跳过联网拉取
if [ -d src/unitree_ros2/cyclonedds_ws ]; then
  echo "[bootstrap] 已存在 src/unitree_ros2，跳过 vcs import"
else
  vcs import < repos/go2_official_ws.repos
fi

set +u
source /opt/ros/humble/setup.bash
set -u

if [ -d src/unitree_ros2/cyclonedds_ws ]; then
  cd src/unitree_ros2/cyclonedds_ws
  colcon build --symlink-install
  cd "${ROOT}"
else
  echo "[bootstrap] 错误：缺少 src/unitree_ros2/cyclonedds_ws" >&2
  echo "[bootstrap] 检查 vcs import 是否成功，或离线包是否带 offline/unitree_ros2.tar.gz" >&2
  exit 1
fi

colcon build --symlink-install
