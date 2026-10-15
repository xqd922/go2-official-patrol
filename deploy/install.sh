#!/usr/bin/env bash
# Go2 巡检系统 —— 生产机安装 / 升级 / 回滚入口（需 sudo）
#
# 在线安装：
#   sudo deploy/install.sh --repo https://github.com/xqd922/go2-official-patrol.git --ref v0.1.0
# 离线安装（U 盘交付包）：
#   sudo deploy/install.sh --bundle /media/usb/go2-official-ws-v0.1.0-bundle.tar.gz --ref v0.1.0 --no-apt
# 回滚到已安装版本：
#   sudo deploy/install.sh --rollback <旧版本目录名>
#
# 目录布局（--root 可改，默认 /opt/go2）：
#   /opt/go2/releases/<版本>/            代码 + 编译产物（一版一份，便于回滚）
#   /opt/go2/current -> releases/<版本>  当前生效版本
#   /opt/go2/sites/<站点>/               地图/点位/语音/站点参数（升级不动它）
#   /etc/go2/site.env                    机器级环境（GO2_SITES / GO2_IFACE）
#   /etc/systemd/system/go2-patrol@.service
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_REPO_URL="$(git -C "${REPO_ROOT}" remote get-url origin 2>/dev/null || true)"

GO2_ROOT="/opt/go2"
TARGET_USER="${SUDO_USER:-$(id -un)}"
REF="main"
REPO_URL=""
BUNDLE=""
SITE_SRC=""
SITE_NAME=""
IFACE=""
ROLLBACK=""
SKIP_BUILD=0
DO_APT=1
DO_SYSTEMD=1
FORCE=0

info() { echo "[install] $*"; }
warn() { echo "[install] 警告: $*" >&2; }
die() { echo "[install] 错误: $*" >&2; exit 1; }

usage() {
  cat <<'EOF'
用法: sudo deploy/install.sh [选项]

  --repo <URL>         仓库地址（默认取本机 clone 的 origin）
  --ref <分支|tag|sha> 安装版本，默认 main；用离线包时建议写成交付包版本名
  --bundle <文件>      从离线交付包安装（与 --repo 二选一）
  --root <目录>        部署根目录，默认 /opt/go2
  --user <用户名>      服务运行用户，默认执行 sudo 的用户
  --iface <网卡>       写入 /etc/go2/site.env 的 GO2_IFACE（接机器狗的网卡）
  --site <目录>        安装后立即落地一份站点资产（等价于 apply_site.sh）
  --site-name <名称>   站点名，默认取站点源目录名
  --no-apt             跳过系统依赖安装（离线机 / 依赖已装好）
  --no-build           只拉代码不编译
  --no-systemd         不安装 systemd 单元（容器或测试环境用）
  --force              版本目录已存在时覆盖重装
  --rollback <版本>    把 current 切回已安装版本并重启相关站点服务
  -h, --help           显示本帮助
EOF
}

opt_value() {
  local opt="$1"
  shift
  if [ "$#" -lt 1 ] || [ -z "${1:-}" ]; then
    die "${opt} 缺少取值"
  fi
  printf '%s' "$1"
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --repo)
      REPO_URL="$(opt_value "$1" "${2:-}")"
      shift 2
      ;;
    --ref)
      REF="$(opt_value "$1" "${2:-}")"
      shift 2
      ;;
    --bundle)
      BUNDLE="$(opt_value "$1" "${2:-}")"
      shift 2
      ;;
    --root)
      GO2_ROOT="$(opt_value "$1" "${2:-}")"
      shift 2
      ;;
    --user)
      TARGET_USER="$(opt_value "$1" "${2:-}")"
      shift 2
      ;;
    --iface)
      IFACE="$(opt_value "$1" "${2:-}")"
      shift 2
      ;;
    --site)
      SITE_SRC="$(opt_value "$1" "${2:-}")"
      shift 2
      ;;
    --site-name)
      SITE_NAME="$(opt_value "$1" "${2:-}")"
      shift 2
      ;;
    --rollback)
      ROLLBACK="$(opt_value "$1" "${2:-}")"
      shift 2
      ;;
    --no-apt)
      DO_APT=0
      shift
      ;;
    --no-build)
      SKIP_BUILD=1
      shift
      ;;
    --no-systemd)
      DO_SYSTEMD=0
      shift
      ;;
    --force)
      FORCE=1
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
  die "请用 sudo 运行（需要写 ${GO2_ROOT}、/etc/systemd/system）"
fi

restart_site_services() {
  local dir site
  for dir in "${GO2_ROOT}/sites"/*/; do
    if [ -d "$dir" ]; then
      site="$(basename "$dir")"
      if systemctl is-active --quiet "go2-patrol@${site}.service"; then
        info "重启 go2-patrol@${site}.service"
        systemctl restart "go2-patrol@${site}.service"
      fi
    fi
  done
}

# ---------------- 回滚 ----------------
if [ -n "$ROLLBACK" ]; then
  case "$ROLLBACK" in
    */*) die "--rollback 只接受版本目录名（见 ${GO2_ROOT}/releases）" ;;
  esac
  if [ ! -d "${GO2_ROOT}/releases/${ROLLBACK}" ]; then
    die "未找到版本目录: ${GO2_ROOT}/releases/${ROLLBACK}"
  fi
  ln -sfn "releases/${ROLLBACK}" "${GO2_ROOT}/current"
  info "current 已切到 ${ROLLBACK}"
  restart_site_services
  exit 0
fi

# ---------------- 参数校验 ----------------
if [ -z "$BUNDLE" ] && [ -z "$REPO_URL" ]; then
  REPO_URL="${GO2_REPO_URL:-$DEFAULT_REPO_URL}"
fi
if [ -n "$BUNDLE" ]; then
  [ -f "$BUNDLE" ] || die "离线包不存在: $BUNDLE"
  BUNDLE="$(cd "$(dirname "$BUNDLE")" && pwd)/$(basename "$BUNDLE")"
else
  [ -n "$REPO_URL" ] || die "需要 --repo <仓库地址> 或 --bundle <离线包>"
fi
id "$TARGET_USER" >/dev/null 2>&1 || die "用户不存在: $TARGET_USER"

if ! command -v git >/dev/null 2>&1; then
  info "安装 git"
  apt-get update
  apt-get install -y git
fi

REF_SAFE="${REF//\//_}"
RELEASE_DIR="${GO2_ROOT}/releases/${REF_SAFE}"
case "$RELEASE_DIR" in
  "${GO2_ROOT}"/releases/*) ;;
  *) die "版本目录异常: ${RELEASE_DIR}" ;;
esac

mkdir -p "${GO2_ROOT}/releases" "${GO2_ROOT}/sites"
if [ -e "$RELEASE_DIR" ]; then
  [ "$FORCE" -eq 1 ] || die "版本目录已存在: ${RELEASE_DIR}（加 --force 覆盖重装）"
  info "覆盖已存在的版本目录: ${RELEASE_DIR}"
  rm -rf -- "$RELEASE_DIR"
fi
mkdir -p "$RELEASE_DIR"

# ---------------- 取代码 ----------------
if [ -n "$BUNDLE" ]; then
  info "解包离线交付包: ${BUNDLE}"
  tar -xzf "$BUNDLE" -C "$RELEASE_DIR" --strip-components=1
  if [ -f "${RELEASE_DIR}/offline/manifest.txt" ]; then
    cp -f "${RELEASE_DIR}/offline/manifest.txt" "${RELEASE_DIR}/DELIVERY_MANIFEST.txt"
    info "已保留交付清单: DELIVERY_MANIFEST.txt"
  fi
  if [ -f "${RELEASE_DIR}/offline/unitree_ros2.tar.gz" ]; then
    info "展开离线依赖 src/unitree_ros2（跳过联网 vcs 拉取）"
    mkdir -p "${RELEASE_DIR}/src"
    tar -xzf "${RELEASE_DIR}/offline/unitree_ros2.tar.gz" -C "${RELEASE_DIR}/src"
  else
    warn "离线包里没有 offline/unitree_ros2.tar.gz，编译时会联网 vcs import"
  fi
else
  info "克隆 ${REPO_URL} (${REF}) → ${RELEASE_DIR}"
  if ! git clone --quiet --branch "$REF" --depth 1 "$REPO_URL" "$RELEASE_DIR"; then
    info "按分支浅克隆失败（可能是 tag/sha），改为完整克隆后 checkout"
    rm -rf -- "$RELEASE_DIR"
    mkdir -p "$RELEASE_DIR"
    git clone --quiet "$REPO_URL" "$RELEASE_DIR"
    git -C "$RELEASE_DIR" checkout --quiet "$REF"
  fi
fi

chown -R "${TARGET_USER}:${TARGET_USER}" "$RELEASE_DIR" 2>/dev/null || warn "chown 未完成，继续"

# ---------------- 依赖 ----------------
if [ "$DO_APT" -eq 1 ]; then
  info "安装系统依赖（清单: deploy/apt-deps.txt；离线机请用 --no-apt）"
  apt-get update
  mapfile -t GO2_APT_PKGS < <(grep -vE '^[[:space:]]*(#|$)' "${RELEASE_DIR}/deploy/apt-deps.txt")
  if [ "${#GO2_APT_PKGS[@]}" -eq 0 ]; then
    die "依赖清单为空: ${RELEASE_DIR}/deploy/apt-deps.txt"
  fi
  apt-get install -y "${GO2_APT_PKGS[@]}" ||
    die "apt 装依赖失败；离线机请改用 --no-apt（并提前装好 ROS 2 Humble 与清单里的包）"
else
  info "跳过系统依赖安装（--no-apt）"
fi

# ---------------- 编译 ----------------
if [ "$SKIP_BUILD" -eq 1 ]; then
  warn "跳过编译（--no-build）：启用服务前需执行 ${RELEASE_DIR}/scripts/bootstrap.sh"
else
  info "编译工作空间（首次约 10-20 分钟；以 ${TARGET_USER} 身份编译）"
  su - "$TARGET_USER" -c "cd '${RELEASE_DIR}' && GO2_SKIP_APT=1 ./scripts/bootstrap.sh"
fi

ln -sfn "releases/${REF_SAFE}" "${GO2_ROOT}/current"
info "current -> releases/${REF_SAFE}"

# ---------------- 机器级环境 ----------------
mkdir -p /etc/go2 /var/log/go2
{
  echo "# Go2 巡检 - 机器级配置（deploy/install.sh 生成）"
  echo "# 改完执行: sudo systemctl restart go2-patrol@<站点>"
  echo "GO2_SITES=${GO2_ROOT}/sites"
  if [ -n "$IFACE" ]; then
    echo "GO2_IFACE=${IFACE}"
  fi
} >/etc/go2/site.env
info "已写 /etc/go2/site.env"

# ---------------- systemd ----------------
if [ "$DO_SYSTEMD" -eq 1 ]; then
  sed -e "s|@GO2_ROOT@|${GO2_ROOT}|g" -e "s|@GO2_USER@|${TARGET_USER}|g" \
    "${RELEASE_DIR}/deploy/systemd/go2-patrol@.service" >/etc/systemd/system/go2-patrol@.service
  chmod 0644 /etc/systemd/system/go2-patrol@.service
  systemctl daemon-reload
  info "已安装 systemd 单元 go2-patrol@.service（运行用户 ${TARGET_USER}）"
else
  info "已跳过 systemd 安装（--no-systemd）"
fi

# ---------------- 可选：落地站点 ----------------
if [ -n "$SITE_SRC" ]; then
  info "落地站点资产: ${SITE_SRC}"
  APPLY_ARGS=(--site-dir "$SITE_SRC" --sites-dir "${GO2_ROOT}/sites")
  if [ -n "$SITE_NAME" ]; then
    APPLY_ARGS+=(--name "$SITE_NAME")
  fi
  bash "${GO2_ROOT}/current/deploy/apply_site.sh" "${APPLY_ARGS[@]}"
fi

cat <<EOF

[install] 完成
  版本目录 : ${RELEASE_DIR}
  当前版本 : ${GO2_ROOT}/current -> releases/${REF_SAFE}
  站点目录 : ${GO2_ROOT}/sites
  运行用户 : ${TARGET_USER}

下一步：
  1) 落地站点资产（地图 / 点位 / 语音 / site.yaml）：
     sudo bash ${GO2_ROOT}/current/deploy/apply_site.sh --site-dir <站点目录> --name <站点名>
  2) 开机自启并立即启动巡检：
     sudo systemctl enable --now go2-patrol@<站点名>
  3) 看日志 / 停止：
     journalctl -u go2-patrol@<站点名> -f
     sudo systemctl stop go2-patrol@<站点名>

升级与回滚：
  sudo bash ${GO2_ROOT}/current/deploy/install.sh --ref <新版本>
  sudo bash ${GO2_ROOT}/current/deploy/install.sh --rollback <旧版本目录名>
EOF
