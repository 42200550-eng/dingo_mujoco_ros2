#!/bin/bash
# Dingo Quadruped Simulation Startup Script
# Starts MuJoCo simulation (no Gazebo/CHAMP packages required)

set -e

# Source ROS environment
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

echo "Starting MuJoCo simulation..."
echo "This will launch:"
echo "  - MuJoCo plant (dingo_mujoco)"
echo "  - /clock, /joint_states, /odom (TF)"
echo "  - robot_state_publisher for TF"
echo ""
echo "Press Ctrl+C to stop the simulation"
echo ""

# Launch the simulation
ros2 launch dingo_config mujoco.launch.py render:=true "$@"

