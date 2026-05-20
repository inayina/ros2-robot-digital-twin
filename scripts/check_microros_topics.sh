#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
HZ_TIMEOUT_SECONDS="${HZ_TIMEOUT_SECONDS:-6}"
QOS_RELIABILITY="${QOS_RELIABILITY:-best_effort}"

TOPICS=(
  "/imu/data"
  "/imu/filtered"
  "/robot/state"
  "/motor/status"
  "/motor/actual_rpm"
  "/motor/state"
)

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[ERROR] ROS 2 Jazzy setup file not found: $ROS_SETUP" >&2
  exit 1
fi

source "$ROS_SETUP"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 command not found after sourcing: $ROS_SETUP" >&2
  exit 1
fi

echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo "[INFO] Listing ROS 2 topics ..."

TOPIC_LIST="$(ros2 topic list)"
printf '%s\n' "$TOPIC_LIST"
echo

missing=0
for topic in "${TOPICS[@]}"; do
  if printf '%s\n' "$TOPIC_LIST" | grep -Fxq "$topic"; then
    echo "[OK] Found $topic"
  else
    echo "[WARN] Missing $topic"
    missing=1
  fi
done

echo
echo "[INFO] Sampling topic rates with ros2 topic hz."
echo "[INFO] Using QoS reliability: $QOS_RELIABILITY"
echo "[INFO] If a topic is quiet, check that the Agent is running and reset ESP32 after the Agent is ready."

for topic in "${TOPICS[@]}"; do
  if ! printf '%s\n' "$TOPIC_LIST" | grep -Fxq "$topic"; then
    continue
  fi

  echo
  echo "[HZ] $topic"
  hz_cmd=(ros2 topic hz "$topic" --qos-reliability "$QOS_RELIABILITY")
  if command -v timeout >/dev/null 2>&1; then
    timeout "${HZ_TIMEOUT_SECONDS}s" "${hz_cmd[@]}" || true
  else
    echo "[WARN] timeout command not found; press Ctrl+C after a few samples."
    "${hz_cmd[@]}" || true
  fi
done

echo
if [[ "$missing" -eq 0 ]]; then
  echo "[OK] All expected micro-ROS topics are visible."
else
  echo "[WARN] Some expected topics are missing."
  echo "[HINT] Start scripts/start_microros_agent_udp.sh first, then reset ESP32."
  exit 1
fi
