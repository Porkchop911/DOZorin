# deploy/sync.ps1 - push this repo to the robot's ~/hwr (PC -> robot).
# Mirrors git-tracked (+ untracked-but-not-ignored) files, minus evidence/.
# Runtime state and colcon outputs are protected from deletion.
param(
    [string]$Robot = 'ubuntu@192.168.0.223',
    [string]$Dest = 'hwr'
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
Push-Location $repo
try {
    $files = (git ls-files --cached --others --exclude-standard) |
        Where-Object { $_ -and ($_ -notlike 'evidence/*') }
    if (-not $files) { throw 'no files to sync?' }

    $list = Join-Path $env:TEMP 'hwr_sync_files.txt'
    $tgz  = Join-Path $env:TEMP 'hwr_sync.tgz'
    $token = [Guid]::NewGuid().ToString('N')
    $remoteArchive = "/tmp/hwr_sync_$token.tgz"
    $remoteStage = "/tmp/hwr_sync_$token"
    $files | Set-Content -Encoding ascii $list
    tar -czf $tgz -T $list
    if ($LASTEXITCODE -ne 0) { throw "tar failed ($LASTEXITCODE)" }
    $kb = [math]::Round((Get-Item $tgz).Length / 1KB, 1)

    scp -q $tgz "${Robot}:$remoteArchive"
    if ($LASTEXITCODE -ne 0) { throw "scp failed ($LASTEXITCODE)" }
    ssh $Robot "set -e; mkdir '$remoteStage'; tar xzf '$remoteArchive' -C '$remoteStage'; mkdir -p ~/$Dest; rsync -a --delete --exclude='/.takeover' --exclude='/switchover.log' --exclude='/evidence/***' --exclude='/config/hwr.env' --exclude='/maps/***' --exclude='/state/***' --exclude='/ros2_ws/build/***' --exclude='/ros2_ws/install/***' --exclude='/ros2_ws/log/***' '$remoteStage/' ~/$Dest/; find '$remoteStage' -depth -delete; rm -f '$remoteArchive'; chmod +x ~/$Dest/deploy/*.sh ~/$Dest/tools/*.sh ~/$Dest/tools/*.zsh"
    if ($LASTEXITCODE -ne 0) { throw "remote unpack failed ($LASTEXITCODE)" }

    Write-Host "synced $($files.Count) files ($kb KB) -> ${Robot}:~/$Dest"

    # pull the switchover log back for evidence (phase 0.5)
    ssh $Robot "test -f ~/$Dest/switchover.log"
    if ($LASTEXITCODE -eq 0) {
        scp -q "${Robot}:$Dest/switchover.log" (Join-Path $repo 'evidence\phase0\switchover.log')
        Write-Host 'pulled switchover.log -> evidence/phase0/'
    }
}
finally {
    Pop-Location
    Remove-Item $list, $tgz -ErrorAction SilentlyContinue
}
