param(
    [string]$Root = (Resolve-Path ".").Path
)

$ErrorActionPreference = "Stop"

function Invoke-Rg {
    param(
        [string]$Title,
        [string]$Pattern,
        [string[]]$Paths,
        [string[]]$Globs = @("*.py", "*.sh", "*.launch.py", "*.yaml", "*.md", "*.service", "*.rules")
    )

    Write-Output ""
    Write-Output "## $Title"
    $args = @("-n", $Pattern)
    foreach ($glob in $Globs) {
        $args += "--glob"
        $args += $glob
    }
    $args += $Paths

    $output = & rg @args 2>$null
    if ($LASTEXITCODE -eq 0) {
        $output
    } elseif ($LASTEXITCODE -eq 1) {
        "No matches."
    } else {
        throw "rg failed while scanning $Title"
    }
}

Push-Location $Root
try {
    Invoke-Rg `
        -Title "Raw motion publishers" `
        -Pattern "controller/cmd_vel|/cmd_vel|create_publisher\(Twist|ros2 topic pub.*cmd_vel|mecanum_pub|cmd\.publish" `
        -Paths @("vendor", "vendor-modified", "added")

    Invoke-Rg `
        -Title "Hardcoded local endpoints and devices" `
        -Pattern "192\.168\.|127\.0\.0\.1|/home/ubuntu|~/|plughw|/dev/|arecord|aplay" `
        -Paths @("vendor", "vendor-modified", "added", "docs")

    Invoke-Rg `
        -Title "Process killing and detached launches" `
        -Pattern "pkill|kill -9|nohup|timeout [0-9]+ ros2|timeout [0-9]+ python" `
        -Paths @("vendor", "vendor-modified", "added")

    Invoke-Rg `
        -Title "Known suspicious symbols" `
        -Pattern "self\.self|z\.append|pos_rad_to_raw|FollowJointTrajectoryResult|self\.clock\(\)|TODO|FIXME" `
        -Paths @("vendor", "vendor-modified", "added")
}
finally {
    Pop-Location
}

