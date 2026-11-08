param(
    [Parameter(Mandatory = $true)][ValidateSet('Desktop', 'Laptop')][string]$Role,
    [string]$EnvironmentRoot = ''
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$settingsPath = Join-Path $projectRoot '.vex-device.json'
$settings = [ordered]@{ role = $Role; autoPull = ($Role -eq 'Desktop') }
if (Test-Path -LiteralPath $settingsPath) {
    $previous = Get-Content -Raw -Encoding UTF8 -LiteralPath $settingsPath | ConvertFrom-Json
    if ($previous.environmentRoot) { $settings.environmentRoot = $previous.environmentRoot }
}
if ($EnvironmentRoot) {
    $environmentPath = (Resolve-Path -LiteralPath $EnvironmentRoot).Path
    $toolchain = Join-Path $environmentPath 'vscode-data/User/globalStorage/vexrobotics.vexcode/tools/cpp/toolchain_win32/clang/bin/clang.exe'
    if (!(Test-Path -LiteralPath $toolchain -PathType Leaf)) { throw 'EnvironmentRoot must point to the extracted VEX Windows migration bundle.' }
    $settings.environmentRoot = $environmentPath
}
$utf8 = New-Object Text.UTF8Encoding($false)
[IO.File]::WriteAllText($settingsPath, ($settings | ConvertTo-Json -Depth 5) + "`n", $utf8)
Write-Output ($settings | ConvertTo-Json -Depth 5)
Write-Output 'Device settings are local and excluded from Git.'
