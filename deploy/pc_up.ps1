# PC half of the Soudruhu stack (plan 14, 12.6): start whatever is not already listening.
#   relay :12340 -> Strata :8080   (robot explorer VLM preflight)
#   relay :12341 -> Hermes API :8643 (ears.py on the robot)
#   hermes -p robot gateway         (Hermes API server, :8643)
# Strata itself is the user's to start; this only warns when :8080 is down.
# Usage: deploy\pc_up.ps1 [-Register]   (-Register adds a logon scheduled task running this script)
param([switch]$Register)

$repo = Split-Path -Parent $PSScriptRoot
$python = 'C:\Python314\python.exe'   # the python the Windows firewall already allows for the relays
$robot = '192.168.0.223'
$logs = Join-Path $repo 'evidence\phase10'

if ($Register) {
    $action = New-ScheduledTaskAction -Execute 'powershell.exe' `
        -Argument "-NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File `"$PSCommandPath`""
    $trigger = New-ScheduledTaskTrigger -AtLogOn -User $env:USERNAME
    Register-ScheduledTask -TaskName 'HWR pc_up' -Action $action -Trigger $trigger -Force | Out-Null
    'registered scheduled task "HWR pc_up" (at logon)'
}

function Listening([int]$port) {
    [bool](Get-NetTCPConnection -State Listen -LocalPort $port -ErrorAction SilentlyContinue)
}

function Ensure([string]$name, [int]$port, [string]$exe, [string[]]$arguments) {
    if (Listening $port) { "${name}: already listening on :$port"; return }
    Start-Process -FilePath $exe -ArgumentList $arguments -WindowStyle Hidden `
        -RedirectStandardOutput "$logs\pc_up_$port.out.log" -RedirectStandardError "$logs\pc_up_$port.err.log"
    "${name}: started (:$port)"
}

if (-not (Listening 8080)) { 'WARNING: Strata is not listening on :8080; preflights and Hermes will fail until it is up' }

$relay = Join-Path $repo 'tools\lm_studio_relay.py'
Ensure 'strata relay' 12340 $python @($relay, '--allowed-client', $robot, '--listen-port', '12340', '--target-port', '8080')
Ensure 'hermes relay' 12341 $python @($relay, '--allowed-client', $robot, '--listen-port', '12341', '--target-port', '8643')
Ensure 'hermes gateway' 8643 'hermes.cmd' @('-p', 'robot', 'gateway')
