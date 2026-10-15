#!/usr/bin/env bash
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi
echo "ROS 2 Jazzy & CARLA environment sourced successfully."
