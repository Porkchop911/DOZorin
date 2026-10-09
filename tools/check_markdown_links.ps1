param(
    [string]$Root = (Resolve-Path ".").Path,
    [string[]]$Files = @(
        "README.md",
        "PROVENANCE.md",
        "docs/GUIDE.md",
        "docs/ARCHITECTURE.md",
        "docs/CODE_WALKTHROUGH.md",
        "docs/HARDWARE_INTERACTIONS.md",
        "docs/OFFLINE_AUDIT.md",
        "docs/REFERENCE.md",
        "docs/LAUNCH_MAP.md",
        "docs/TASKS.md",
        "docs/DOCUMENTATION_PLAN.md",
        "docs/VENDOR_STACK.md",
        "docs/ADDED_SOFTWARE.md",
        "docs/SYSTEM.md",
        "docs/HARDWARE.md",
        "docs/MODIFICATIONS.md",
        "added/guides/OPERATING.md"
    )
)

$ErrorActionPreference = "Stop"
$missing = New-Object System.Collections.Generic.List[string]

foreach ($relative in $Files) {
    $path = Join-Path $Root $relative
    if (-not (Test-Path -LiteralPath $path)) {
        $missing.Add("$relative -> file itself is missing")
        continue
    }

    $text = Get-Content -Raw -LiteralPath $path
    $matches = [regex]::Matches($text, '\[[^\]]+\]\(([^)#][^)]+)\)')
    foreach ($match in $matches) {
        $target = $match.Groups[1].Value
        if ($target -match '^[a-zA-Z][a-zA-Z0-9+.-]*:') {
            continue
        }
        if ($target.StartsWith("mailto:")) {
            continue
        }

        $base = Split-Path $path
        $resolved = Join-Path $base $target
        if (-not (Test-Path -LiteralPath $resolved)) {
            $missing.Add("$relative -> $target")
        }
    }
}

if ($missing.Count -gt 0) {
    $missing | Sort-Object -Unique
    exit 1
}

"All checked local links exist."
