#!/usr/bin/env bash
set -e

# Keyboard teleop for Dingo (publishes /cmd_vel)

source /opt/ros/humble/setup.bash

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
  source "${WS_DIR}/install/setup.bash"
else
  echo "ERROR: ${WS_DIR}/install/setup.bash not found."
  echo "Build the workspace first, e.g.:"
  echo "  cd ${WS_DIR} && colcon build --symlink-install"
  exit 1
fi

# teleop_twist_keyboard prints keybindings in the terminal
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
