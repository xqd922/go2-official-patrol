#!/usr/bin/env bash
# 由脚本自身位置推导工作空间根目录，不依赖用户名或仓库目录名。
#
# 用法（在其它脚本里）：
#   source "$(dirname "$(realpath "${BASH_SOURCE[0]}")")/repo_root.sh"
# 之后可使用：
#   $GO2_WS_ROOT            # 工作空间根目录（含 src/、scripts/、install/）
#   $GO2_INSTALL_SETUP      # $GO2_WS_ROOT/install/setup.bash
#   $GO2_CYCLONEDDS_SETUP   # $GO2_WS_ROOT/src/unitree_ros2/cyclonedds_ws/install/setup.bash
# 也可通过 GO2_WS_ROOT 环境变量显式覆盖自动推导结果。

# GO2_WS_ROOT 环境变量只做覆盖，不做提前返回：下面的派生变量
# （GO2_INSTALL_SETUP / GO2_CYCLONEDDS_SETUP）每次都要重新计算并导出，
# 否则调用方会拿到空变量，静默跳过 install / CycloneDDS 环境。
if [ -z "${GO2_WS_ROOT:-}" ] || [ ! -d "${GO2_WS_ROOT}/src" ]; then
  _go2_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
  GO2_WS_ROOT="$(cd "${_go2_script_dir}/.." && pwd -P)"
  unset _go2_script_dir
fi

GO2_INSTALL_SETUP="${GO2_WS_ROOT}/install/setup.bash"
GO2_CYCLONEDDS_SETUP="${GO2_WS_ROOT}/src/unitree_ros2/cyclonedds_ws/install/setup.bash"
export GO2_WS_ROOT GO2_INSTALL_SETUP GO2_CYCLONEDDS_SETUP
