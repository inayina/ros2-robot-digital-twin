#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
MICROROS_SETUP="${MICROROS_SETUP:-/home/ina/microros_ws/install/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-$REPO_ROOT/install/setup.bash}"
DASHBOARD_ROOT="${DASHBOARD_ROOT:-/home/ina/workspace/robot-ops-dashboard}"
DASHBOARD_PYTHON="${DASHBOARD_PYTHON:-$DASHBOARD_ROOT/.venv/bin/python}"
DASHBOARD_VENV_SITE_PACKAGES="${DASHBOARD_VENV_SITE_PACKAGES:-}"

MQTT_BROKER_URL="${MQTT_BROKER_URL:-mqtt://127.0.0.1:1883}"
MQTT_HOST="${MQTT_HOST:-127.0.0.1}"
MQTT_PORT="${MQTT_PORT:-1883}"

ROBOT_ID="${ROBOT_ID:-amr-001}"
IMU_ROS_TOPIC="${IMU_ROS_TOPIC:-/imu/data}"
IMU_MQTT_TOPIC="${IMU_MQTT_TOPIC:-robot/imu}"
MOTOR_STATUS_TOPIC="${MOTOR_STATUS_TOPIC:-/motor/status}"
MOTOR_CMD_TOPIC="${MOTOR_CMD_TOPIC:-/motor/cmd}"
MOTOR_STATUS_MQTT_TOPIC="${MOTOR_STATUS_MQTT_TOPIC:-robot/motor/status}"
MOTOR_CMD_MQTT_TOPIC="${MOTOR_CMD_MQTT_TOPIC:-robot/motor/cmd}"
BRIDGE_RATE_LIMIT_HZ="${BRIDGE_RATE_LIMIT_HZ:-5}"
MICRO_ROS_UDP_PORT="${MICRO_ROS_UDP_PORT:-8888}"

DASHBOARD_HOST="${DASHBOARD_HOST:-127.0.0.1}"
DASHBOARD_PORT="${DASHBOARD_PORT:-9000}"
FRONTEND_HOST="${FRONTEND_HOST:-127.0.0.1}"
FRONTEND_PORT="${FRONTEND_PORT:-8001}"

LOG_DIR="${LOG_DIR:-/tmp/robot_state_monitor_motor_dashboard_stack}"
PID_DIR="$LOG_DIR/pids"

MQTT_LOG="$LOG_DIR/mqtt_broker.log"
AGENT_LOG="$LOG_DIR/micro_ros_agent.log"
IMU_BRIDGE_LOG="$LOG_DIR/microros_imu_to_mqtt_bridge.log"
MOTOR_STATUS_BRIDGE_LOG="$LOG_DIR/motor_status_bridge.log"
MOTOR_CMD_BRIDGE_LOG="$LOG_DIR/motor_cmd_bridge.log"
DASHBOARD_LOG="$LOG_DIR/dashboard_backend.log"
FRONTEND_LOG="$LOG_DIR/frontend_static.log"

MQTT_PID_FILE="$PID_DIR/mqtt_broker.pid"
AGENT_PID_FILE="$PID_DIR/micro_ros_agent.pid"
IMU_BRIDGE_PID_FILE="$PID_DIR/microros_imu_to_mqtt_bridge.pid"
MOTOR_STATUS_BRIDGE_PID_FILE="$PID_DIR/motor_status_bridge.pid"
MOTOR_CMD_BRIDGE_PID_FILE="$PID_DIR/motor_cmd_bridge.pid"
DASHBOARD_PID_FILE="$PID_DIR/dashboard_backend.pid"
FRONTEND_PID_FILE="$PID_DIR/frontend_static.pid"

START_FRONTEND=true
START_DASHBOARD=true
START_IMU_BRIDGE=true
START_MQTT=true
CHECK_ONLY=false

usage() {
  cat <<EOF
用法：
  $(basename "$0") [选项]

一键启动当前主链路：
  ESP32/micro-ROS <-> ROS 2 <-> ROS2-MQTT bridge <-> MQTT <-> dashboard backend <-> frontend

默认启动：
  1. MQTT broker: $MQTT_BROKER_URL
  2. micro-ROS Agent: udp4:$MICRO_ROS_UDP_PORT
  3. IMU ROS2->MQTT bridge: $IMU_ROS_TOPIC -> $IMU_MQTT_TOPIC
  4. Motor status bridge: $MOTOR_STATUS_TOPIC -> $MOTOR_STATUS_MQTT_TOPIC
  5. Motor cmd bridge: $MOTOR_CMD_MQTT_TOPIC -> $MOTOR_CMD_TOPIC
  6. Dashboard backend: http://$DASHBOARD_HOST:$DASHBOARD_PORT
  7. Frontend static page: http://$FRONTEND_HOST:$FRONTEND_PORT/frontend/

选项：
  --check               只做路径与依赖检查，不启动进程
  --status              打印当前脚本记录的服务状态
  --stop                停止本脚本上次启动的服务
  --no-frontend         不启动前端静态页面
  --no-dashboard        不启动 dashboard backend
  --no-imu-bridge       不启动 IMU ROS2->MQTT bridge
  --no-mqtt             不启动本地 mosquitto，只检查 broker 可达性
  --imu-topic TOPIC     IMU ROS topic，默认：$IMU_ROS_TOPIC
  --robot-id ID         robot_id，默认：$ROBOT_ID
  --dashboard-root DIR  robot-ops-dashboard 仓库路径
  -h, --help            显示帮助

环境变量：
  ROS_SETUP、MICROROS_SETUP、WORKSPACE_SETUP、DASHBOARD_ROOT、
  MQTT_BROKER_URL、IMU_ROS_TOPIC、ROBOT_ID、DASHBOARD_PORT、FRONTEND_PORT、LOG_DIR
EOF
}

log() {
  echo "[motor-dashboard-stack] $*"
}

fail() {
  echo "[motor-dashboard-stack] ERROR: $*" >&2
  exit 1
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

resolve_dashboard_site_packages() {
  if [[ -n "$DASHBOARD_VENV_SITE_PACKAGES" ]]; then
    return 0
  fi

  DASHBOARD_VENV_SITE_PACKAGES="$(
    find "$DASHBOARD_ROOT/.venv/lib" -maxdepth 2 -type d -name site-packages | head -n 1
  )"

  [[ -n "$DASHBOARD_VENV_SITE_PACKAGES" ]] || fail "未找到 dashboard .venv site-packages 目录。"
}

ensure_paths() {
  require_command bash
  require_command curl
  require_command python3
  require_command mosquitto

  [[ -f "$ROS_SETUP" ]] || fail "未找到 ROS 2 setup.bash：$ROS_SETUP"
  [[ -f "$MICROROS_SETUP" ]] || fail "未找到 micro-ROS setup.bash：$MICROROS_SETUP"
  [[ -f "$WORKSPACE_SETUP" ]] || fail "未找到工作区 setup.bash：$WORKSPACE_SETUP"
  [[ -d "$DASHBOARD_ROOT" ]] || fail "未找到 dashboard 仓库：$DASHBOARD_ROOT"
  [[ -x "$DASHBOARD_PYTHON" ]] || fail "未找到 dashboard Python：$DASHBOARD_PYTHON"
  [[ -f "$DASHBOARD_ROOT/scripts/microros_imu_to_mqtt_bridge.py" ]] || fail "缺少 dashboard IMU bridge 脚本。"
  resolve_dashboard_site_packages
}

ensure_dirs() {
  mkdir -p "$LOG_DIR" "$PID_DIR"
}

http_ok() {
  local url=$1
  curl --silent --noproxy '*' --max-time 3 --output /dev/null --fail "$url"
}

port_open() {
  local host=$1
  local port=$2
  (echo >/dev/tcp/"$host"/"$port") >/dev/null 2>&1
}

process_running() {
  local pid_file=$1
  [[ -f "$pid_file" ]] || return 1
  local pid
  pid="$(cat "$pid_file")"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" >/dev/null 2>&1
}

write_pid_file() {
  local pid=$1
  local pid_file=$2
  echo "$pid" >"$pid_file"
}

stop_pid_file() {
  local label=$1
  local pid_file=$2

  if ! process_running "$pid_file"; then
    rm -f "$pid_file"
    log "$label 未运行。"
    return 0
  fi

  local pid
  pid="$(cat "$pid_file")"
  log "停止 $label PID=$pid"
  kill "$pid" >/dev/null 2>&1 || true
  sleep 1
  if kill -0 "$pid" >/dev/null 2>&1; then
    kill -9 "$pid" >/dev/null 2>&1 || true
  fi
  rm -f "$pid_file"
}

start_background() {
  local label=$1
  local pid_file=$2
  local log_file=$3
  local command=$4

  if process_running "$pid_file"; then
    log "$label 已在运行，PID=$(cat "$pid_file")"
    return 0
  fi

  log "启动 $label"
  nohup bash -lc "$command" >"$log_file" 2>&1 &
  local pid=$!
  write_pid_file "$pid" "$pid_file"
  sleep 1

  if ! kill -0 "$pid" >/dev/null 2>&1; then
    fail "$label 启动失败，请检查日志：$log_file"
  fi
}

start_mqtt_if_needed() {
  if port_open "$MQTT_HOST" "$MQTT_PORT"; then
    log "检测到 MQTT broker 已就绪：$MQTT_HOST:$MQTT_PORT"
    return 0
  fi

  if [[ "$START_MQTT" != true ]]; then
    fail "MQTT broker 不可达：$MQTT_HOST:$MQTT_PORT"
  fi

  start_background \
    "MQTT broker" \
    "$MQTT_PID_FILE" \
    "$MQTT_LOG" \
    "exec mosquitto -p $MQTT_PORT"

  for _attempt in {1..10}; do
    if port_open "$MQTT_HOST" "$MQTT_PORT"; then
      log "MQTT broker ready: $MQTT_HOST:$MQTT_PORT"
      return 0
    fi
    sleep 1
  done

  fail "MQTT broker 未在 10 秒内就绪：$MQTT_HOST:$MQTT_PORT"
}

start_micro_ros_agent() {
  start_background \
    "micro-ROS Agent" \
    "$AGENT_PID_FILE" \
    "$AGENT_LOG" \
    "source '$ROS_SETUP' && source '$MICROROS_SETUP' && exec ros2 run micro_ros_agent micro_ros_agent udp4 --port $MICRO_ROS_UDP_PORT -v 4"
}

start_imu_bridge() {
  [[ "$START_IMU_BRIDGE" == true ]] || return 0

  start_background \
    "IMU ROS2->MQTT bridge" \
    "$IMU_BRIDGE_PID_FILE" \
    "$IMU_BRIDGE_LOG" \
    "source '$ROS_SETUP' && source '$MICROROS_SETUP' && export PYTHONPATH='$DASHBOARD_VENV_SITE_PACKAGES':\"\$PYTHONPATH\" && cd '$DASHBOARD_ROOT' && exec python3 scripts/microros_imu_to_mqtt_bridge.py --ros-topic '$IMU_ROS_TOPIC' --mqtt-broker '$MQTT_BROKER_URL' --mqtt-topic '$IMU_MQTT_TOPIC' --robot-id '$ROBOT_ID' --rate-limit-hz '$BRIDGE_RATE_LIMIT_HZ'"
}

start_motor_status_bridge() {
  start_background \
    "motor status bridge" \
    "$MOTOR_STATUS_BRIDGE_PID_FILE" \
    "$MOTOR_STATUS_BRIDGE_LOG" \
    "source '$ROS_SETUP' && source '$WORKSPACE_SETUP' && export PYTHONPATH='$DASHBOARD_VENV_SITE_PACKAGES':\"\$PYTHONPATH\" && cd '$REPO_ROOT' && exec ros2 run robot_mqtt_bridge motor_status_bridge --ros-args -p motor_status_topic:='$MOTOR_STATUS_TOPIC' -p mqtt_host:='$MQTT_HOST' -p mqtt_port:=$MQTT_PORT -p mqtt_topic:='$MOTOR_STATUS_MQTT_TOPIC' -p robot_id:='$ROBOT_ID'"
}

start_motor_cmd_bridge() {
  start_background \
    "motor cmd bridge" \
    "$MOTOR_CMD_BRIDGE_PID_FILE" \
    "$MOTOR_CMD_BRIDGE_LOG" \
    "source '$ROS_SETUP' && source '$WORKSPACE_SETUP' && export PYTHONPATH='$DASHBOARD_VENV_SITE_PACKAGES':\"\$PYTHONPATH\" && cd '$REPO_ROOT' && exec ros2 run robot_mqtt_bridge motor_cmd_bridge --ros-args -p ros_cmd_topic:='$MOTOR_CMD_TOPIC' -p mqtt_host:='$MQTT_HOST' -p mqtt_port:=$MQTT_PORT -p mqtt_topic:='$MOTOR_CMD_MQTT_TOPIC' -p robot_id:='$ROBOT_ID'"
}

start_dashboard_backend() {
  [[ "$START_DASHBOARD" == true ]] || return 0

  if http_ok "http://$DASHBOARD_HOST:$DASHBOARD_PORT/health"; then
    log "dashboard backend 已就绪：http://$DASHBOARD_HOST:$DASHBOARD_PORT"
    return 0
  fi

  start_background \
    "dashboard backend" \
    "$DASHBOARD_PID_FILE" \
    "$DASHBOARD_LOG" \
    "source '$DASHBOARD_ROOT/.venv/bin/activate' && export PYTHONPATH='$DASHBOARD_ROOT' && export MQTT_BROKER_URL='$MQTT_BROKER_URL' && export ROBOT_OPS_TASK_SOURCE='mock_json' && cd '$DASHBOARD_ROOT' && exec uvicorn backend.app.main:app --host '$DASHBOARD_HOST' --port '$DASHBOARD_PORT'"

  for _attempt in {1..30}; do
    if http_ok "http://$DASHBOARD_HOST:$DASHBOARD_PORT/health"; then
      log "dashboard backend ready: http://$DASHBOARD_HOST:$DASHBOARD_PORT"
      return 0
    fi
    sleep 1
  done

  fail "dashboard backend 未在 30 秒内就绪。"
}

start_frontend() {
  [[ "$START_FRONTEND" == true ]] || return 0

  if http_ok "http://$FRONTEND_HOST:$FRONTEND_PORT/frontend/"; then
    log "frontend 已就绪：http://$FRONTEND_HOST:$FRONTEND_PORT/frontend/"
    return 0
  fi

  start_background \
    "frontend static server" \
    "$FRONTEND_PID_FILE" \
    "$FRONTEND_LOG" \
    "cd '$DASHBOARD_ROOT' && exec python3 -m http.server '$FRONTEND_PORT' --bind '$FRONTEND_HOST'"

  for _attempt in {1..30}; do
    if http_ok "http://$FRONTEND_HOST:$FRONTEND_PORT/frontend/"; then
      log "frontend ready: http://$FRONTEND_HOST:$FRONTEND_PORT/frontend/"
      return 0
    fi
    sleep 1
  done

  fail "frontend 未在 30 秒内就绪。"
}

print_status() {
  echo "== Process =="
  for entry in \
    "MQTT broker:$MQTT_PID_FILE:$MQTT_LOG" \
    "micro-ROS Agent:$AGENT_PID_FILE:$AGENT_LOG" \
    "IMU bridge:$IMU_BRIDGE_PID_FILE:$IMU_BRIDGE_LOG" \
    "motor status bridge:$MOTOR_STATUS_BRIDGE_PID_FILE:$MOTOR_STATUS_BRIDGE_LOG" \
    "motor cmd bridge:$MOTOR_CMD_BRIDGE_PID_FILE:$MOTOR_CMD_BRIDGE_LOG" \
    "dashboard backend:$DASHBOARD_PID_FILE:$DASHBOARD_LOG" \
    "frontend:$FRONTEND_PID_FILE:$FRONTEND_LOG"; do
    IFS=":" read -r label pid_file log_file <<<"$entry"
    if process_running "$pid_file"; then
      echo "[RUNNING] $label pid=$(cat "$pid_file") log=$log_file"
    else
      echo "[STOPPED] $label log=$log_file"
    fi
  done

  echo
  echo "== HTTP =="
  if http_ok "http://$DASHBOARD_HOST:$DASHBOARD_PORT/health"; then
    echo "[OK] backend http://$DASHBOARD_HOST:$DASHBOARD_PORT/health"
  else
    echo "[WARN] backend http://$DASHBOARD_HOST:$DASHBOARD_PORT/health"
  fi

  if http_ok "http://$FRONTEND_HOST:$FRONTEND_PORT/frontend/"; then
    echo "[OK] frontend http://$FRONTEND_HOST:$FRONTEND_PORT/frontend/"
  else
    echo "[WARN] frontend http://$FRONTEND_HOST:$FRONTEND_PORT/frontend/"
  fi

  echo
  echo "== Broker =="
  if port_open "$MQTT_HOST" "$MQTT_PORT"; then
    echo "[OK] mqtt $MQTT_HOST:$MQTT_PORT"
  else
    echo "[WARN] mqtt $MQTT_HOST:$MQTT_PORT"
  fi
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --check)
        CHECK_ONLY=true
        shift
        ;;
      --status)
        ensure_dirs
        parse_mqtt_url
        ensure_paths
        print_status
        exit 0
        ;;
      --stop)
        ensure_dirs
        stop_pid_file "frontend static server" "$FRONTEND_PID_FILE"
        stop_pid_file "dashboard backend" "$DASHBOARD_PID_FILE"
        stop_pid_file "motor cmd bridge" "$MOTOR_CMD_BRIDGE_PID_FILE"
        stop_pid_file "motor status bridge" "$MOTOR_STATUS_BRIDGE_PID_FILE"
        stop_pid_file "IMU ROS2->MQTT bridge" "$IMU_BRIDGE_PID_FILE"
        stop_pid_file "micro-ROS Agent" "$AGENT_PID_FILE"
        stop_pid_file "MQTT broker" "$MQTT_PID_FILE"
        exit 0
        ;;
      --no-frontend)
        START_FRONTEND=false
        shift
        ;;
      --no-dashboard)
        START_DASHBOARD=false
        shift
        ;;
      --no-imu-bridge)
        START_IMU_BRIDGE=false
        shift
        ;;
      --no-mqtt)
        START_MQTT=false
        shift
        ;;
      --imu-topic)
        IMU_ROS_TOPIC=$2
        shift 2
        ;;
      --robot-id)
        ROBOT_ID=$2
        shift 2
        ;;
      --dashboard-root)
        DASHBOARD_ROOT=$2
        DASHBOARD_PYTHON="$2/.venv/bin/python"
        shift 2
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
}

main() {
  parse_args "$@"
  parse_mqtt_url
  ensure_paths
  ensure_dirs

  if [[ "$CHECK_ONLY" == true ]]; then
    log "检查通过。"
    log "ROS setup: $ROS_SETUP"
    log "micro-ROS setup: $MICROROS_SETUP"
    log "workspace setup: $WORKSPACE_SETUP"
    log "dashboard root: $DASHBOARD_ROOT"
    log "mqtt broker: $MQTT_BROKER_URL"
    exit 0
  fi

  start_mqtt_if_needed
  start_micro_ros_agent
  start_motor_status_bridge
  start_motor_cmd_bridge
  start_imu_bridge
  start_dashboard_backend
  start_frontend

  echo
  log "联调链路已启动。"
  log "backend:  http://$DASHBOARD_HOST:$DASHBOARD_PORT"
  log "frontend: http://$FRONTEND_HOST:$FRONTEND_PORT/frontend/"
  log "logs:     $LOG_DIR"
  log "建议下一步：./scripts/check_motor_dashboard_loop.sh"
}

main "$@"
