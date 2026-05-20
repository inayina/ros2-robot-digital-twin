#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

find_agent_setup() {
  if [[ -n "${MICRO_ROS_AGENT_SETUP:-}" ]]; then
    if [[ -f "$MICRO_ROS_AGENT_SETUP" ]]; then
      printf '%s\n' "$MICRO_ROS_AGENT_SETUP"
      return 0
    fi
    echo "[ERROR] MICRO_ROS_AGENT_SETUP is set but not found: $MICRO_ROS_AGENT_SETUP" >&2
    return 1
  fi

  local candidates=(
    "$HOME/uros_ws/install/local_setup.bash"
    "$HOME/micro_ros_ws/install/local_setup.bash"
  )

  local candidate
  for candidate in "${candidates[@]}"; do
    if [[ -f "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  echo "[ERROR] micro-ROS Agent setup file not found." >&2
  echo "[HINT] Set MICRO_ROS_AGENT_SETUP, for example:" >&2
  echo "       export MICRO_ROS_AGENT_SETUP=\$HOME/uros_ws/install/local_setup.bash" >&2
  return 1
}

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[ERROR] ROS 2 Jazzy setup file not found: $ROS_SETUP" >&2
  exit 1
fi

AGENT_SETUP="$(find_agent_setup)"

echo "[INFO] Sourcing ROS 2 Jazzy: $ROS_SETUP"
source "$ROS_SETUP"

echo "[INFO] Sourcing micro-ROS Agent workspace: $AGENT_SETUP"
source "$AGENT_SETUP"

echo "[INFO] Starting micro-ROS Agent on UDP port 8888 ..."
exec ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v 4
