# deploy/env.zsh - source this (zsh, on the robot) before running hwr nodes:
#   source ~/hwr/deploy/env.zsh
# Vendor env first (ROS + DDS config), then our overlay, then hwr_core.
set +u
source $HOME/ros2_ws/.robotrc >/dev/null 2>&1
source $HOME/hwr/ros2_ws/install/setup.zsh
export PYTHONPATH=$HOME/hwr/hwr_core:$PYTHONPATH
if [[ -f $HOME/hwr/config/hwr.env ]]; then
  set -a
  source $HOME/hwr/config/hwr.env
  set +a
fi
