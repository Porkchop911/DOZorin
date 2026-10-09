#!/usr/bin/env bash
# Record the standard motion/sensor bag used by Phases 3-7.
set -euo pipefail

name=${1:?usage: tools/record.sh NAME [OUTPUT_DIR]}
output_dir=${2:-$HOME/hwr/bags}
mkdir -p "$output_dir"

exec ros2 bag record --output "$output_dir/$name" \
    /scan_raw /scan /imu /imu_filtered /battery \
    /cmd_vel_applied /wheel_speeds /odom_model /odom_scan /odom \
    /tf /tf_static
