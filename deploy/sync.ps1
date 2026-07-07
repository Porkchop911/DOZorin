# deploy/sync.ps1 - push this repo to the robot's ~/hwr (PC -> robot).
# Ships exactly the git-tracked (+ untracked-but-not-ignored) files, minus evidence/.
# Idempotent. Pulls back switchover.log into evidence/phase0/ if present.
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
    $files | Set-Content -Encoding ascii $list
    tar -czf $tgz -T $list
    if ($LASTEXITCODE -ne 0) { throw "tar failed ($LASTEXITCODE)" }
    $kb = [math]::Round((Get-Item $tgz).Length / 1KB, 1)

    scp -q $tgz "${Robot}:/tmp/hwr_sync.tgz"
    if ($LASTEXITCODE -ne 0) { throw "scp failed ($LASTEXITCODE)" }
    ssh $Robot "mkdir -p ~/$Dest && tar xzf /tmp/hwr_sync.tgz -C ~/$Dest && rm /tmp/hwr_sync.tgz && chmod +x ~/$Dest/deploy/*.sh"
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
