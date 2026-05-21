#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-$REPO_ROOT/install/setup.bash}"
MICROROS_SETUP="${MICROROS_SETUP:-/home/ina/microros_ws/install/setup.bash}"
MQTT_BROKER_URL="${MQTT_BROKER_URL:-mqtt://127.0.0.1:1883}"
MQTT_HOST="${MQTT_HOST:-127.0.0.1}"
MQTT_PORT="${MQTT_PORT:-1883}"
IMU_MQTT_TOPIC="${IMU_MQTT_TOPIC:-robot/imu}"
MOTOR_STATUS_MQTT_TOPIC="${MOTOR_STATUS_MQTT_TOPIC:-robot/motor/status}"
BACKEND_BASE_URL="${BACKEND_BASE_URL:-http://127.0.0.1:9000}"

usage() {
  cat <<EOF
用法：
  $(basename "$0")

验证当前主链路：
  micro-ROS/ROS 2 -> MQTT -> dashboard backend/fronted

检查项：
  1. ROS 2 topic 是否可见
  2. dashboard backend /health 与 /api/robot/status 是否可用
  3. robot/imu 是否能从 MQTT 看到实时消息
  4. POST /api/robot/motor/cmd 是否能落到 ROS 2 /motor/cmd
  5. /motor/status 是否已由实机固件发布，并能镜像到 MQTT
EOF
}

log() {
  echo "[motor-dashboard-check] $*"
}

fail() {
  echo "[motor-dashboard-check] ERROR: $*" >&2
  exit 1
}

warn() {
  echo "[motor-dashboard-check] WARN: $*" >&2
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    fail "缺少依赖命令：$1"
  fi
}

parse_mqtt_url() {
  local parsed
  parsed="$(
    python3 - "$MQTT_BROKER_URL" <<'PY'
import sys
from urllib.parse import urlparse

url = sys.argv[1]
if "://" not in url:
    url = f"mqtt://{url}"
parsed = urlparse(url)
print(parsed.hostname or "127.0.0.1")
print(parsed.port or 1883)
PY
  )"
  MQTT_HOST="$(echo "$parsed" | sed -n '1p')"
  MQTT_PORT="$(echo "$parsed" | sed -n '2p')"
}

ensure_paths() {
  require_command curl
  require_command mosquitto_sub
  require_command python3
  require_command timeout

  [[ -f "$ROS_SETUP" ]] || fail "未找到 ROS 2 setup.bash：$ROS_SETUP"
  [[ -f "$WORKSPACE_SETUP" ]] || fail "未找到工作区 setup.bash：$WORKSPACE_SETUP"
  [[ -f "$MICROROS_SETUP" ]] || fail "未找到 micro-ROS setup.bash：$MICROROS_SETUP"
}

http_json() {
  local url=$1
  local output_path=$2
  local http_code

  http_code="$(
    curl \
      --silent \
      --show-error \
      --noproxy '*' \
      --max-time 5 \
      --output "$output_path" \
      --write-out '%{http_code}' \
      "$url"
  )"

  [[ "$http_code" == "200" ]]
}

topic_list() {
  set +u
  source "$ROS_SETUP"
  source "$WORKSPACE_SETUP"
  set -u
  ros2 topic list
}

topic_exists() {
  local topic=$1
  printf '%s\n' "$TOPIC_LIST" | grep -Fxq "$topic"
}

topic_publisher_count() {
  local topic=$1
  set +u
  source "$ROS_SETUP"
  source "$WORKSPACE_SETUP"
  set -u
  ros2 topic info -v "$topic" | sed -n 's/^Publisher count: //p'
}

topic_subscription_count() {
  local topic=$1
  set +u
  source "$ROS_SETUP"
  source "$WORKSPACE_SETUP"
  set -u
  ros2 topic info -v "$topic" | sed -n 's/^Subscription count: //p'
}

check_backend() {
  local health_json
  local status_json
  health_json="$(mktemp)"
  status_json="$(mktemp)"

  if ! http_json "$BACKEND_BASE_URL/health" "$health_json"; then
    rm -f "$health_json" "$status_json"
    fail "dashboard backend health 不可用：$BACKEND_BASE_URL/health"
  fi
  log "backend health OK: $BACKEND_BASE_URL/health"

  if ! http_json "$BACKEND_BASE_URL/api/robot/status" "$status_json"; then
    rm -f "$health_json" "$status_json"
    fail "dashboard backend robot status 不可用：$BACKEND_BASE_URL/api/robot/status"
  fi

  python3 - "$status_json" <<'PY'
import json
import sys

data = json.load(open(sys.argv[1], "r", encoding="utf-8"))
connection = data.get("connection", {})
robot = data.get("robot", {})
print(f"connection_status={connection.get('status')}")
print(f"imu_present={robot.get('imu') is not None}")
print(f"motor_present={robot.get('motor_status') is not None}")
PY

  rm -f "$health_json" "$status_json"
}

check_imu_mqtt() {
  local imu_payload
  imu_payload="$(timeout 5s mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "$IMU_MQTT_TOPIC" -C 1 || true)"
  [[ -n "$imu_payload" ]] || fail "在 MQTT topic $IMU_MQTT_TOPIC 上 5 秒内未收到 IMU 消息。"
  log "MQTT IMU OK: $IMU_MQTT_TOPIC"
}

check_motor_cmd_loop() {
  local echo_file
  local echo_pid
  local status_file
  local status_pid
  local cmd_response
  echo_file="$(mktemp)"
  status_file="$(mktemp)"

  (
    set +u
    source "$ROS_SETUP"
    source "$WORKSPACE_SETUP"
    set -u
    timeout 20s ros2 --use-python-default-buffering topic echo --once /motor/cmd >"$echo_file"
  ) &
  echo_pid=$!

  (
    set +u
    source "$ROS_SETUP"
    source "$WORKSPACE_SETUP"
    set -u
    timeout 20s ros2 --use-python-default-buffering topic echo /motor/status >"$status_file"
  ) &
  status_pid=$!

  # Wait until the echo subscriber is visible on the graph before sending.
  local subscriptions
  subscriptions=0
  for _attempt in {1..12}; do
    subscriptions="$(topic_subscription_count /motor/cmd || echo 0)"
    if [[ "${subscriptions:-0}" =~ ^[0-9]+$ ]] && (( subscriptions >= 2 )); then
      break
    fi
    sleep 1
  done

  for _attempt in 1 2 3; do
    cmd_response="$(
      curl \
        --silent \
        --show-error \
        --noproxy '*' \
        --request POST \
        --header 'Content-Type: application/json' \
        --data '{"target_rpm":123.0,"enabled":true,"closed_loop":true,"max_pwm":0.3,"timeout_ms":700}' \
        "$BACKEND_BASE_URL/api/robot/motor/cmd"
    )"

    for _poll in {1..10}; do
      if grep -q '"target_rpm":123.0' "$echo_file"; then
        kill "$echo_pid" >/dev/null 2>&1 || true
        wait "$echo_pid" >/dev/null 2>&1 || true
        kill "$status_pid" >/dev/null 2>&1 || true
        wait "$status_pid" >/dev/null 2>&1 || true
        log "dashboard -> MQTT -> ROS /motor/cmd OK"
        rm -f "$echo_file" "$status_file"
        return 0
      fi

      if grep -Eq '"target_rpm":123(\.0+)?' "$status_file"; then
        kill "$echo_pid" >/dev/null 2>&1 || true
        wait "$echo_pid" >/dev/null 2>&1 || true
        kill "$status_pid" >/dev/null 2>&1 || true
        wait "$status_pid" >/dev/null 2>&1 || true
        log "dashboard -> MQTT -> ROS -> ESP32 motor status OK"
        rm -f "$echo_file" "$status_file"
        return 0
      fi
      sleep 1
    done
  done

  wait "$echo_pid" || true
  wait "$status_pid" || true
  rm -f "$echo_file" "$status_file"
  fail "未在 ROS 2 /motor/cmd 上观察到 backend 下发的命令。响应：$cmd_response"
}

check_motor_status_loop() {
  if ! topic_exists "/motor/status"; then
    warn "ROS 2 缺少 /motor/status。当前实机很可能仍在旧固件上，只发布 /motor/actual_rpm 和 /motor/state。"
    return 1
  fi

  local publisher_count
  publisher_count="$(topic_publisher_count /motor/status)"
  if [[ "${publisher_count:-0}" == "0" ]]; then
    warn "/motor/status 当前没有 publisher。当前实机很可能仍在旧固件上，只发布 /motor/actual_rpm 和 /motor/state。"
    return 1
  fi

  local motor_payload
  motor_payload="$(timeout 5s mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" -t "$MOTOR_STATUS_MQTT_TOPIC" -C 1 || true)"
  [[ -n "$motor_payload" ]] || fail "在 MQTT topic $MOTOR_STATUS_MQTT_TOPIC 上 5 秒内未收到电机状态消息。"

  python3 - <<'PY' "$motor_payload"
import json
import sys

payload = json.loads(sys.argv[1])
required = ("target_rpm", "measured_rpm", "pwm", "error_rpm", "enabled", "closed_loop", "fault")
missing = [key for key in required if key not in payload]
if missing:
    raise SystemExit(f"缺少字段: {', '.join(missing)}")
PY

  log "ROS /motor/status -> MQTT $MOTOR_STATUS_MQTT_TOPIC OK"
  return 0
}

main() {
  case "${1:-}" in
    -h|--help)
      usage
      exit 0
      ;;
    "")
      ;;
    *)
      fail "未知参数：$1"
      ;;
  esac

  parse_mqtt_url
  ensure_paths

  TOPIC_LIST="$(topic_list)"
  log "当前 ROS 2 topics:"
  printf '%s\n' "$TOPIC_LIST"

  for topic in /imu/data /imu/filtered /robot/state /motor/actual_rpm /motor/state; do
    topic_exists "$topic" || fail "缺少 ROS 2 topic：$topic"
  done

  check_backend
  check_imu_mqtt
  check_motor_cmd_loop

  if ! check_motor_status_loop; then
    fail "未完成实机电机状态上报闭环：请先刷写当前仓库 ESP32 固件，再重跑本脚本。"
  fi

  log "整链检查通过。"
}

main "$@"
