param(
    [string]$SdkPath = "$env:APPDATA/Code/User/globalStorage/vexrobotics.vexcode/sdk/cpp/V5/V5_20240802_15_00_00",
    [string]$ToolchainPath = "$env:APPDATA/Code/User/globalStorage/vexrobotics.vexcode/tools/cpp/toolchain_win32"
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
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
& $makePath @makeArguments
if ($LASTEXITCODE -ne 0) { throw "VEX build failed (exit $LASTEXITCODE)." }
