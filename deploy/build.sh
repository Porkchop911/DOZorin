#!/bin/bash
# deploy/build.sh - build the hwr stack ON the robot (~/hwr).
# Installs hwr_core editable, then colcon-builds ros2_ws. Building only
# needs base ROS (setup.bash is fine here; runtime launches use zsh+.robotrc).
set -euo pipefail
cd "$HOME/hwr"

python3 -m pip install -q -e hwr_core

source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select \
    hwr_msgs hwr_driver hwr_base hwr_description hwr_teleop
echo "build OK -> source ~/hwr/ros2_ws/install/setup.zsh (or .bash)"
