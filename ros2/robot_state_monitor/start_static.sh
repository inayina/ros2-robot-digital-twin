#!/usr/bin/env bash
set -euo pipefail

WS_DIR="/home/ina/ros2_ws"
PKG_NAME="robot_state_monitor"
MODEL_NAME="mpu6050"
MODEL_FILE="$WS_DIR/src/$PKG_NAME/models/$MODEL_NAME.sdf"
WORLD_NAME="default"
WORLD_FILE="$WS_DIR/src/$PKG_NAME/worlds/empty.sdf"
GAZEBO_LOG="/tmp/gazebo_static.log"
BRIDGE_LOG="/tmp/gazebo_bridge_static.log"

cleanup() {
  set +e
  echo "[INFO] Cleaning up background processes..."
  [[ -n "${BRIDGE_PID:-}" ]] && kill "$BRIDGE_PID" 2>/dev/null || true
  [[ -n "${SERVICE_BRIDGE_PID:-}" ]] && kill "$SERVICE_BRIDGE_PID" 2>/dev/null || true
  [[ -n "${GAZEBO_PID:-}" ]] && kill "$GAZEBO_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -f "$MODEL_FILE" ]]; then
  echo "[ERROR] Model file not found: $MODEL_FILE"
  exit 1
fi

if [[ ! -f "$WORLD_FILE" ]]; then
  echo "[ERROR] World file not found: $WORLD_FILE"
  exit 1
fi

cd "$WS_DIR"

set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_LOG_DIR=/tmp/ros_logs
mkdir -p "$ROS_LOG_DIR"
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QUICK_BACKEND=software

echo "[1/6] Building $PKG_NAME ..."
colcon build --packages-select "$PKG_NAME"

set +u
source "$WS_DIR/install/setup.bash"
set -u

echo "[2/6] Starting Gazebo ..."
if ! command -v gz >/dev/null 2>&1; then
  echo "[ERROR] 'gz' was not found in PATH."
  echo "[HINT] Install Gazebo Harmonic and ROS bridge packages, then rerun:"
  echo "       sudo apt update"
  echo "       sudo apt install gz-harmonic ros-jazzy-ros-gz ros-jazzy-ros-gz-sim"
  exit 1
fi

# Run Gazebo Harmonic in server-only mode to avoid EGL / GUI driver issues.
gz sim -r -s -v 4 "$WORLD_FILE" >"$GAZEBO_LOG" 2>&1 &
GAZEBO_PID=$!

echo "[3/6] Waiting for Gazebo services ..."
for _ in {1..60}; do
  if gz service -l 2>/dev/null | grep -q "/world/$WORLD_NAME/create" && \
     gz service -l 2>/dev/null | grep -q "/world/$WORLD_NAME/set_pose"; then
    break
  fi
  sleep 1
done

if ! gz service -l 2>/dev/null | grep -q "/world/$WORLD_NAME/create" || \
   ! gz service -l 2>/dev/null | grep -q "/world/$WORLD_NAME/set_pose"; then
  echo "[ERROR] Gazebo Harmonic did not expose /world/$WORLD_NAME/create or /set_pose in time."
  echo "[HINT] Check log: $GAZEBO_LOG"
  exit 1
fi

echo "[4/6] Starting ROS-Gazebo service bridge ..."
ros2 run ros_gz_bridge parameter_bridge \
  "/world/$WORLD_NAME/create@ros_gz_interfaces/srv/SpawnEntity" \
  "/world/$WORLD_NAME/set_pose@ros_gz_interfaces/srv/SetEntityPose" \
  >/tmp/ros_gz_service_bridge.log 2>&1 &
SERVICE_BRIDGE_PID=$!

for _ in {1..30}; do
  if ros2 service list 2>/dev/null | grep -q "/world/$WORLD_NAME/create" && \
     ros2 service list 2>/dev/null | grep -q "/world/$WORLD_NAME/set_pose"; then
    break
  fi
  sleep 1
done

echo "[5/6] Starting gazebo bridge ..."
ros2 run "$PKG_NAME" gazebo_bridge --ros-args \
  -p model_name:="$MODEL_NAME" \
  -p world_name:="$WORLD_NAME" \
  >"$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!

for _ in {1..30}; do
  if ros2 node list 2>/dev/null | grep -q "/gazebo_bridge"; then
    break
  fi
  sleep 1
done

echo "[6/7] Publishing one static IMU sample ..."
ros2 topic pub --once /imu/filtered sensor_msgs/msg/Imu "{orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"

sleep 1
echo "[7/7] Verifying $MODEL_NAME exists in Gazebo pose stream ..."
if timeout 5 gz topic -e -t "/world/$WORLD_NAME/pose/info" 2>/dev/null | grep -q "$MODEL_NAME"; then
  echo "[INFO] $MODEL_NAME found in /world/$WORLD_NAME/pose/info."
else
  echo "[WARN] $MODEL_NAME not found from Gazebo pose check."
  echo "[HINT] Check log: $BRIDGE_LOG"
fi

echo

echo "[OK] Static test stack is running."
echo "     Gazebo log : $GAZEBO_LOG"
echo "     Bridge log : $BRIDGE_LOG"
echo "     (Gazebo is running headless server-only mode)"
echo "     Press Ctrl+C to stop everything."

wait "$GAZEBO_PID"
