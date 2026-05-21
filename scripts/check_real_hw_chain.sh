#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ESP32_PORT="${ESP32_PORT:-/dev/ttyACM0}"
RUN_DASHBOARD_CHECK=false

usage() {
  cat <<EOF
用法：
  $(basename "$0") [选项]

烧录并复位板子后，执行当前实机整链检查：
  1. 检查 ESP32 串口设备是否已枚举
  2. 运行 micro-ROS topic / 下行订阅检查
  3. 可选检查 dashboard MQTT/HTTP 整链

选项：
  --esp32-port PORT   ESP32 串口设备，默认：$ESP32_PORT
  --dashboard         追加运行 dashboard 整链检查
  -h, --help          显示帮助
EOF
}

fail() {
  echo "[real-hw-check] ERROR: $*" >&2
  exit 1
}

log() {
  echo "[real-hw-check] $*"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --esp32-port)
      [[ $# -ge 2 ]] || fail "--esp32-port 需要一个参数"
      ESP32_PORT="$2"
      shift 2
      ;;
    --dashboard)
      RUN_DASHBOARD_CHECK=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "未知参数：$1"
      ;;
  esac
done

[[ -e "$ESP32_PORT" ]] || fail "未找到 ESP32 串口设备：$ESP32_PORT。请确认板子已烧录、接线正常并重新插拔。"

log "检测到 ESP32 设备：$ESP32_PORT"
log "默认启动顺序：先运行 scripts/start_microros_agent_udp.sh，再复位 ESP32，然后执行本脚本。"

"$REPO_ROOT/scripts/check_microros_topics.sh"

if [[ "$RUN_DASHBOARD_CHECK" == true ]]; then
  log "运行 dashboard 整链检查。"
  "$REPO_ROOT/scripts/check_motor_dashboard_loop.sh"
fi

log "实机整链检查完成。"
