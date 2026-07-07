#!/usr/bin/env bash
# deploy/remote.sh - run a command on the robot inside the factory ROS env (POSIX twin of remote.ps1).
set -euo pipefail
ROBOT="${ROBOT:-ubuntu@192.168.0.223}"
payload='source $HOME/ros2_ws/.robotrc >/dev/null 2>&1; '"$*"
b64=$(printf '%s' "$payload" | base64 | tr -d '\n')
exec ssh "$ROBOT" "echo $b64 | base64 -d | zsh"
