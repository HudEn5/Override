param(
    [switch]$Watch,
    [switch]$Auto,
    [ValidateRange(10, 3600)][int]$IntervalSeconds = 60,
    [string]$RepositoryPath = '',
    [string]$ExpectedRemoteUrl = 'https://github.com/HudEn5/Override.git'
)

$ErrorActionPreference = 'Stop'
if (!$RepositoryPath) { $RepositoryPath = Split-Path -Parent $PSScriptRoot }
$repositoryRoot = [IO.Path]::GetFullPath($RepositoryPath).TrimEnd('\', '/')
$gitCommand = (Get-Command git -ErrorAction Stop).Source

function Quote-NativeArgument([string]$Value) {
    return '"' + (($Value -replace '(\\*)"', '$1$1\"') -replace '(\\+)$', '$1$1') + '"'
}

function Invoke-Git([string[]]$GitArguments) {
    $arguments = @('-c', 'http.lowSpeedLimit=1', '-c', 'http.lowSpeedTime=20',
        '-c', 'credential.interactive=never', '-C', $repositoryRoot) + $GitArguments
    $startInfo = New-Object Diagnostics.ProcessStartInfo
    $startInfo.FileName = $gitCommand
    $startInfo.Arguments = (($arguments | ForEach-Object { Quote-NativeArgument $_ }) -join ' ')
    $startInfo.UseShellExecute = $false
    $startInfo.CreateNoWindow = $true
    $startInfo.RedirectStandardOutput = $true
    $startInfo.RedirectStandardError = $true
    $startInfo.EnvironmentVariables['GIT_TERMINAL_PROMPT'] = '0'
    $startInfo.EnvironmentVariables['GCM_INTERACTIVE'] = 'never'
    $process = New-Object Diagnostics.Process
    $process.StartInfo = $startInfo
    try {
        [void]$process.Start()
        $outputTask = $process.StandardOutput.ReadToEndAsync()
        $errorTask = $process.StandardError.ReadToEndAsync()
        if (!$process.WaitForExit(45000)) {
            $process.Kill()
            throw 'Git command timed out; local source was preserved.'
        }
        $outputText = $outputTask.Result
        $errorText = $errorTask.Result
        if ($process.ExitCode -ne 0) { throw ($errorText + $outputText).Trim() }
        return $outputText.Trim()
    } finally { $process.Dispose() }
}

function Normalize-Remote([string]$Url) {
    return (($Url.TrimEnd('/') -replace '^git@github\.com:', 'https://github.com/') -replace '\.git$', '').ToLowerInvariant()
}

function Test-AutomaticPull {
    $settingsPath = Join-Path $repositoryRoot '.vex-device.json'
    if (!(Test-Path -LiteralPath $settingsPath -PathType Leaf)) { return $false }
    $settings = Get-Content -Raw -Encoding UTF8 -LiteralPath $settingsPath | ConvertFrom-Json
    return $settings.role -eq 'Desktop' -and $settings.autoPull -eq $true
}

function Get-PauseReason {
    if ((Invoke-Git @('branch', '--show-current')) -ne 'main') { return 'Not on main; current branch was preserved.' }
    if (Invoke-Git @('status', '--porcelain', '--untracked-files=all')) { return 'Local changes found; commit and push them before syncing.' }
    return ''
}

function Sync-Once {
    $actualRoot = [IO.Path]::GetFullPath((Invoke-Git @('rev-parse', '--show-toplevel'))).TrimEnd('\', '/')
    if (![string]::Equals($actualRoot, $repositoryRoot, [StringComparison]::OrdinalIgnoreCase)) {
        throw 'Refusing to sync a directory that is not the repository root.'
    }
    if ((Normalize-Remote (Invoke-Git @('remote', 'get-url', 'origin'))) -ne (Normalize-Remote $ExpectedRemoteUrl)) {
        throw 'origin does not match the intended Override repository.'
    }
    $reason = Get-PauseReason
    if ($reason) { return [pscustomobject]@{status='paused';message=$reason} }
    $before = Invoke-Git @('rev-parse', '--verify', 'HEAD')
    [void](Invoke-Git @('fetch', '--no-tags', '--no-recurse-submodules', 'origin', '+refs/heads/main:refs/remotes/origin/main'))
    $reason = Get-PauseReason
    if ($reason) { return [pscustomobject]@{status='paused';message=$reason} }
    if ((Invoke-Git @('rev-parse', 'HEAD')) -ne $before) {
        return [pscustomobject]@{status='paused';message='Local HEAD changed during fetch; retry later.'}
    }
    if ((Invoke-Git @('rev-list', '--count', 'origin/main..HEAD')) -ne '0') {
        return [pscustomobject]@{status='paused';message='Local commits or diverged history found; push or resolve them first.'}
    }
    $target = Invoke-Git @('rev-parse', 'origin/main')
    if ($target -ne $before) {
        [void](Invoke-Git @('merge', '--ff-only', '--no-edit', '--no-autostash', '--no-overwrite-ignore', 'origin/main'))
    }
    $status = if ($target -eq $before) { 'current' } else { 'updated' }
    return [pscustomobject]@{status=$status;message=('main @ ' + (Invoke-Git @('rev-parse', '--short', 'HEAD')))}
}

$hashAlgorithm = [Security.Cryptography.SHA256]::Create()
try { $lockId = [BitConverter]::ToString($hashAlgorithm.ComputeHash([Text.Encoding]::UTF8.GetBytes($repositoryRoot.ToLowerInvariant()))).Replace('-', '') }
finally { $hashAlgorithm.Dispose() }
$syncMutex = New-Object Threading.Mutex($false, ('Local\VexGitSync-' + $lockId))
$hasLock = $false
try {
    try { $hasLock = $syncMutex.WaitOne(0) }
    catch [Threading.AbandonedMutexException] { $hasLock = $true }
    if (!$hasLock) { Write-Output '[busy] Another sync worker is already running for this repository.'; exit 0 }
    $continuous = $Watch -or $Auto
    do {
    try {
        if ($Auto -and !(Test-AutomaticPull)) {
            Write-Output '[disabled] Automatic pull is disabled on this device. Use configure-device.ps1 -Role Desktop to enable it.'
            exit 0
        }
        $result = Sync-Once
        Write-Output ('[{0}] [{1}] {2}' -f (Get-Date -Format 'HH:mm:ss'), $result.status, $result.message)
        if (!$continuous) {
            if ($result.status -eq 'paused') { exit 2 }
            exit 0
        }
    } catch {
        Write-Output ('[{0}] [error] {1}' -f (Get-Date -Format 'HH:mm:ss'), $_.Exception.Message)
        if (!$continuous) { exit 1 }
    }
    Start-Sleep -Seconds $IntervalSeconds
    } while ($continuous)
} finally {
    if ($hasLock) { $syncMutex.ReleaseMutex() }
    $syncMutex.Dispose()
}
