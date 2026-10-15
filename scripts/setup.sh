#!/usr/bin/env bash
set -euo pipefail

# 环境定义已统一到 deploy/go2_env.sh（开发机 ~/go2_official_ws 与生产机
# /opt/go2/current 共用同一份，路径按脚本位置自动推导，不再写死 $HOME）。
_go2_setup_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../deploy/go2_env.sh
source "${_go2_setup_dir}/../deploy/go2_env.sh"
unset _go2_setup_dir
