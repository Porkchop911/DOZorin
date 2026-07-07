# deploy/remote.ps1 - run a command on the robot inside the factory ROS env.
# Base64-armors the payload (survives PowerShell/ssh quoting and CRLF), executes under zsh
# because ~/ros2_ws/.robotrc sources local_setup.zsh files.
param(
    [Parameter(Mandatory, ValueFromRemainingArguments)][string[]]$Cmd,
    [string]$Robot = 'ubuntu@192.168.0.223'
)
$payload = 'source $HOME/ros2_ws/.robotrc >/dev/null 2>&1; ' + ($Cmd -join ' ')
$b64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($payload))
ssh $Robot "echo $b64 | base64 -d | zsh"
exit $LASTEXITCODE
