#!/usr/bin/env bash
set -euo pipefail

WS_DIR="/home/ina/ros2_ws"
PKG_NAME="robot_state_monitor"
MODEL_FILE="$WS_DIR/src/$PKG_NAME/models/robot_model.sdf"
WORLD_FILE="/usr/share/gazebo-11/worlds/empty.world"
GAZEBO_LOG="/tmp/gazebo_static.log"
BRIDGE_LOG="/tmp/gazebo_bridge_static.log"

cleanup() {
  set +e
  echo "[INFO] Cleaning up background processes..."
  [[ -n "${BRIDGE_PID:-}" ]] && kill "$BRIDGE_PID" 2>/dev/null || true
  [[ -n "${GAZEBO_PID:-}" ]] && kill "$GAZEBO_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -f "$MODEL_FILE" ]]; then
  echo "[ERROR] Model file not found: $MODEL_FILE"
  exit 1
fi

cd "$WS_DIR"

source /opt/ros/jazzy/setup.bash

echo "[1/6] Building $PKG_NAME ..."
colcon build --packages-select "$PKG_NAME"

source "$WS_DIR/install/setup.bash"

echo "[2/6] Starting Gazebo ..."
gazebo --verbose -s libgazebo_ros_factory.so "$WORLD_FILE" >"$GAZEBO_LOG" 2>&1 &
GAZEBO_PID=$!

echo "[3/6] Waiting for Gazebo services ..."
for _ in {1..60}; do
  if ros2 service list 2>/dev/null | grep -q "/spawn_entity"; then
    break
  fi
  sleep 1
done

if ! ros2 service list 2>/dev/null | grep -q "/spawn_entity"; then
  echo "[ERROR] Gazebo did not expose /spawn_entity in time."
  echo "[HINT] Check log: $GAZEBO_LOG"
  exit 1
fi

echo "[4/6] Spawning robot_model ..."
ros2 run gazebo_ros spawn_entity.py -entity robot_model -file "$MODEL_FILE"

echo "[5/6] Starting gazebo bridge ..."
ros2 run "$PKG_NAME" gazebo_bridge >"$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!

for _ in {1..30}; do
  if ros2 node list 2>/dev/null | grep -q "/gazebo_bridge"; then
    break
  fi
  sleep 1
done

echo "[6/6] Publishing one static IMU sample ..."
ros2 topic pub --once /imu/filtered sensor_msgs/msg/Imu "{orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"

echo

echo "[OK] Static test stack is running."
echo "     Gazebo log : $GAZEBO_LOG"
echo "     Bridge log : $BRIDGE_LOG"
echo "     Press Ctrl+C to stop everything."

wait "$GAZEBO_PID"
