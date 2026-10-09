#!/usr/bin/env bash
# deploy/sync.sh - push this repo to the robot's ~/hwr (PC -> robot), POSIX twin of sync.ps1.
# Mirrors git-tracked (+ untracked-but-not-ignored) files, minus evidence/.
# Runtime state and colcon outputs are protected from deletion.
set -euo pipefail
ROBOT="${ROBOT:-ubuntu@192.168.0.223}"
DEST="${DEST:-hwr}"
cd "$(dirname "$0")/.."

list=$(mktemp)
trap 'rm -f "$list"' EXIT
git ls-files --cached --others --exclude-standard | grep -v '^evidence/' > "$list"
count=$(wc -l < "$list")

remote_stage=$(ssh "$ROBOT" 'mktemp -d /tmp/hwr-sync.XXXXXX')
case "$remote_stage" in
    /tmp/hwr-sync.*) ;;
    *) echo "invalid remote staging path: $remote_stage" >&2; exit 1 ;;
esac

tar czf - -T "$list" | ssh "$ROBOT" "tar xzf - -C '$remote_stage'"
ssh "$ROBOT" "mkdir -p ~/$DEST && rsync -a --delete \
    --exclude='/.takeover' --exclude='/switchover.log' --exclude='/evidence/***' \
    --exclude='/config/hwr.env' --exclude='/maps/***' --exclude='/state/***' \
    --exclude='/ros2_ws/build/***' --exclude='/ros2_ws/install/***' \
    --exclude='/ros2_ws/log/***' '$remote_stage/' ~/$DEST/ && \
    find '$remote_stage' -depth -delete && \
    chmod +x ~/$DEST/deploy/*.sh ~/$DEST/tools/*.sh ~/$DEST/tools/*.zsh"
echo "synced $count files -> $ROBOT:~/$DEST"

# pull the switchover log back for evidence (phase 0.5)
if ssh "$ROBOT" "test -f ~/$DEST/switchover.log"; then
    scp -q "$ROBOT:$DEST/switchover.log" evidence/phase0/switchover.log
    echo "pulled switchover.log -> evidence/phase0/"
fi
