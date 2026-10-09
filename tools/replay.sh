#!/usr/bin/env bash
# Replay a recorded HWR bag with ROS simulated time available on /clock.
set -euo pipefail

bag=${1:?usage: tools/replay.sh BAG_PATH [ros2 bag play options...]}
shift
exec ros2 bag play "$bag" --clock "$@"
