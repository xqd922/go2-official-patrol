#!/usr/bin/env bash
# 卸载 Go2 巡检服务：默认只停服务、删 systemd 单元，保留 /opt/go2 数据便于回滚/排查。
#   sudo deploy/uninstall.sh [--root /opt/go2] [--purge]
set -euo pipefail

GO2_ROOT="/opt/go2"
PURGE=0

usage() {
  cat <<'EOF'
用法: sudo deploy/uninstall.sh [选项]

  --root <目录>   部署根目录，默认 /opt/go2
  --purge         连 /opt/go2 与 /etc/go2 一起删除（不可恢复）
  -h, --help      显示本帮助
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --root)
      [ "$#" -ge 2 ] || { echo "--root 缺少取值" >&2; exit 2; }
      GO2_ROOT="$2"
      shift 2
      ;;
    --purge)
      PURGE=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [ "$(id -u)" -ne 0 ]; then
  echo "[uninstall] 请用 sudo 运行" >&2
  exit 1
fi

{
  for dir in "${GO2_ROOT}/sites"/*/; do
    if [ -d "$dir" ]; then
      basename "$dir"
    fi
  done
  systemctl list-units --all --plain --no-legend 'go2-patrol@*.service' 2>/dev/null |
    awk '{print $1}' | sed -e 's/^go2-patrol@//' -e 's/\.service$//'
} | sort -u | while read -r site; do
  if [ -n "$site" ]; then
    systemctl disable --now "go2-patrol@${site}.service" >/dev/null 2>&1 || true
    echo "[uninstall] 已停止 go2-patrol@${site}.service"
  fi
done || true

rm -f /etc/systemd/system/go2-patrol@.service
systemctl daemon-reload || true
echo "[uninstall] 已移除 systemd 单元 go2-patrol@.service"

if [ "$PURGE" -eq 1 ]; then
  case "$GO2_ROOT" in
    ""|"/"|"/opt"|"/usr"|"/etc"|"/home")
      echo "[uninstall] 拒绝删除受保护路径: ${GO2_ROOT}" >&2
      exit 1
      ;;
  esac
  rm -rf -- "$GO2_ROOT" /etc/go2
  echo "[uninstall] 已删除 ${GO2_ROOT} 与 /etc/go2"
else
  echo "[uninstall] 已保留 ${GO2_ROOT} 与 /etc/go2（需要彻底清理时加 --purge）"
fi
