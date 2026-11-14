param(
    [string]$SdkPath = "$env:APPDATA/Code/User/globalStorage/vexrobotics.vexcode/sdk/cpp/V5/V5_20240802_15_00_00",
    [string]$ToolchainPath = "$env:APPDATA/Code/User/globalStorage/vexrobotics.vexcode/tools/cpp/toolchain_win32",
    [string]$EnvironmentRoot = '',
    [switch]$Rebuild,
    [ValidateSet('Default', 'Test', 'Competition', 'Calibration')]
    [string]$Mode = 'Default'
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$deviceSettingsPath = Join-Path $projectRoot '.vex-device.json'
if (!$EnvironmentRoot -and (Test-Path -LiteralPath $deviceSettingsPath)) {
    $deviceSettings = Get-Content -Raw -Encoding UTF8 -LiteralPath $deviceSettingsPath | ConvertFrom-Json
    $EnvironmentRoot = $deviceSettings.environmentRoot
}
if ($EnvironmentRoot) {
    $storageRoot = Join-Path $EnvironmentRoot 'vscode-data/User/globalStorage/vexrobotics.vexcode'
    $sdkVersion = (Get-Content -Raw -Encoding UTF8 -LiteralPath (Join-Path $projectRoot '.vscode/vex_project_settings.json') | ConvertFrom-Json).project.sdkVersion
    if (!$PSBoundParameters.ContainsKey('SdkPath')) { $SdkPath = Join-Path $storageRoot ('sdk/cpp/V5/' + $sdkVersion) }
    if (!$PSBoundParameters.ContainsKey('ToolchainPath')) { $ToolchainPath = Join-Path $storageRoot 'tools/cpp/toolchain_win32' }
}
$makePath = Join-Path $ToolchainPath 'tools/bin/make.exe'
$compilerPath = Join-Path $ToolchainPath 'clang/bin/clang.exe'
$linkerPath = Join-Path $ToolchainPath 'gcc/bin/arm-none-eabi-ld.exe'
$sizePath = Join-Path $ToolchainPath 'gcc/bin/arm-none-eabi-size.exe'
$objcopyPath = Join-Path $ToolchainPath 'gcc/bin/arm-none-eabi-objcopy.exe'
foreach ($path in @($makePath, $compilerPath, $linkerPath, $sizePath, $objcopyPath,
                    (Join-Path $SdkPath 'vexv5/include/v5.h'))) {
    if (!(Test-Path -LiteralPath $path)) {
        throw "VEX build dependency not found: $path. Supply -SdkPath / -ToolchainPath."
    }
}

$makeArguments = @(
    '-C', $projectRoot, "T=$SdkPath", 'P=Bartmoss',
    ('CXX="{0}"' -f $compilerPath),
    ('LINK="{0}"' -f $linkerPath),
    ('SIZE="{0}"' -f $sizePath),
    ('OBJCOPY="{0}"' -f $objcopyPath)
)
if ($Mode -ne 'Default') {
    $modeFlag = if ($Mode -eq 'Competition') { 1 } else { 0 }
    $calibrationFlag = if ($Mode -eq 'Calibration') { 1 } else { 0 }
    $makeArguments += @("BUILD=build/$($Mode.ToLowerInvariant())",
                        "DEFINES=-DVexV5 -DBARTMOSS_COMPETITION=$modeFlag -DBARTMOSS_CALIBRATION=$calibrationFlag")
}
if ($Rebuild) { $makeArguments = @('-B') + $makeArguments }
$previousOS = $env:OS
try {
    if ([Environment]::OSVersion.Platform -eq [PlatformID]::Win32NT) { $env:OS = 'Windows_NT' }
    & $makePath @makeArguments
    if ($LASTEXITCODE -ne 0) { throw "VEX build failed (exit $LASTEXITCODE)." }
} finally { $env:OS = $previousOS }
