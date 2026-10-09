#!/bin/bash
# deploy/build.sh - build the hwr stack ON the robot (~/hwr).
# hwr_core is used via PYTHONPATH (the Jetson's pip/setuptools are too old
# for clean editable installs); deploy/env.zsh wires it up at runtime.
set -eo pipefail  # no -u: ROS setup.bash reads unset vars
cd "$HOME/hwr"

source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select \
    hwr_msgs hwr_driver hwr_base hwr_description hwr_teleop hwr_sensors \
    hwr_odom hwr_slam hwr_loc hwr_nav hwr_arm hwr_explore hwr_gateway hwr_agent
echo "build OK -> source ~/hwr/deploy/env.zsh to run"
