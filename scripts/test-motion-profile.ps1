param([string]$CompilerPath = '')

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$testBuild = Join-Path $projectRoot 'build/motion-tests'
New-Item -ItemType Directory -Path $testBuild -Force | Out-Null
if (!$CompilerPath) {
    foreach ($name in @('g++', 'clang++', 'zig')) {
        $candidate = Get-Command $name -ErrorAction SilentlyContinue
        if ($candidate) { $CompilerPath = $candidate.Source; break }
    }
}
if (!$CompilerPath) { throw 'Supply -CompilerPath pointing to g++, clang++, or Zig.' }
$previousGlobalCache = $env:ZIG_GLOBAL_CACHE_DIR
$previousLocalCache = $env:ZIG_LOCAL_CACHE_DIR
try {
    $compilerPrefix = @()
    if ([IO.Path]::GetFileNameWithoutExtension($CompilerPath) -eq 'zig') {
        $compilerPrefix = @('c++')
        $env:ZIG_GLOBAL_CACHE_DIR = Join-Path $testBuild 'zig-global-cache'
        $env:ZIG_LOCAL_CACHE_DIR = Join-Path $testBuild 'zig-local-cache'
    }
    foreach ($testName in @('driver-control-test', 'trapezoid-profile-test', 'drive-test-test', 'drive-test-route-test')) {
        $testExecutable = Join-Path $testBuild ($testName + '.exe')
        $compileArguments = $compilerPrefix + @(
            '-std=c++11', '-Wall', '-Wextra', '-Werror', '-pedantic',
            '-I', (Join-Path $projectRoot 'tests/fakes'),
            '-I', (Join-Path $projectRoot 'include'),
            (Join-Path $projectRoot ('tests/' + $testName + '.cpp'))
        )
        if ($testName -eq 'trapezoid-profile-test') {
            $compileArguments += Join-Path $projectRoot 'src/robot-config.cpp'
        }
        $compileArguments += @('-o', $testExecutable)
        & $CompilerPath @compileArguments
        if ($LASTEXITCODE -ne 0) { throw "$testName did not compile (exit $LASTEXITCODE)." }
        & $testExecutable
        if ($LASTEXITCODE -ne 0) { throw "$testName failed (exit $LASTEXITCODE)." }
    }
} finally {
    $env:ZIG_GLOBAL_CACHE_DIR = $previousGlobalCache
    $env:ZIG_LOCAL_CACHE_DIR = $previousLocalCache
}
