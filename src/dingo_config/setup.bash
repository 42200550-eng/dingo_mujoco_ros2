#!/bin/bash
# =============================================================
# Dingo Quadruped Workspace Setup Script
# Source this file to configure the environment
# Usage: source setup.bash
# =============================================================

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   Dingo Quadruped Environment Setup   ${NC}"
echo -e "${GREEN}========================================${NC}"

# 1. Source ROS 2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✓ ROS 2 Humble sourced${NC}"
else
    echo -e "${RED}✗ ROS 2 Humble not found at /opt/ros/humble/setup.bash${NC}"
    return 1
fi

# 2. Source workspace (if built)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    echo -e "${GREEN}✓ Workspace sourced: $WS_DIR${NC}"
else
    echo -e "${YELLOW}⚠ Workspace not built yet. Run 'colcon build' first.${NC}"
fi

# 3. Export Gazebo paths
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$SCRIPT_DIR/meshes
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$SCRIPT_DIR/worlds

# 4. Set ROS Domain ID (change if needed for multi-robot)
export ROS_DOMAIN_ID=0

# 5. Useful aliases
alias dingo_gazebo='ros2 launch dingo_config gazebo.launch.py'
alias dingo_bringup='ros2 launch dingo_config bringup.launch.py'
alias dingo_rviz='ros2 launch dingo_config display.launch.py'
alias dingo_teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard'

echo -e "${GREEN}✓ Gazebo paths configured${NC}"
echo -e "${GREEN}✓ Aliases created: dingo_gazebo, dingo_bringup, dingo_rviz, dingo_teleop${NC}"
echo ""
echo -e "${YELLOW}Quick Start Commands:${NC}"
echo "  • dingo_gazebo   - Launch Gazebo simulation"
echo "  • dingo_bringup  - Launch CHAMP controller (standalone)"
echo "  • dingo_rviz     - Launch RViz visualization"
echo "  • dingo_teleop   - Launch keyboard teleoperation"
echo ""
