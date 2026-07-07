#!/usr/bin/env bash
# deploy/sync.sh - push this repo to the robot's ~/hwr (PC -> robot), POSIX twin of sync.ps1.
# Ships exactly the git-tracked (+ untracked-but-not-ignored) files, minus evidence/.
set -euo pipefail
ROBOT="${ROBOT:-ubuntu@192.168.0.223}"
DEST="${DEST:-hwr}"
cd "$(dirname "$0")/.."

list=$(mktemp)
trap 'rm -f "$list"' EXIT
git ls-files --cached --others --exclude-standard | grep -v '^evidence/' > "$list"
count=$(wc -l < "$list")

tar czf - -T "$list" | ssh "$ROBOT" "mkdir -p ~/$DEST && tar xzf - -C ~/$DEST && chmod +x ~/$DEST/deploy/*.sh"
echo "synced $count files -> $ROBOT:~/$DEST"

# pull the switchover log back for evidence (phase 0.5)
if ssh "$ROBOT" "test -f ~/$DEST/switchover.log"; then
    scp -q "$ROBOT:$DEST/switchover.log" evidence/phase0/switchover.log
    echo "pulled switchover.log -> evidence/phase0/"
fi
