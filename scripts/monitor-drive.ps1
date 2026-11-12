param(
    [string]$Port = 'Auto',
    [ValidateRange(1, 600)][int]$DurationSeconds = 60
)

# 只读主控回传，不发送任何电机指令；使用前关闭占用同一串口的终端。
$ErrorActionPreference = 'Stop'
# V5 主控的控制台是 MI_02；遥控器的 MI_01 是命令端口，不能直接读标准输出。
$ports = @(Get-CimInstance Win32_SerialPort)
if ($Port -eq 'Auto') {
    $candidates = @($ports | Where-Object PNPDeviceID -Match 'VID_2888&PID_0501&MI_02')
    if ($candidates.Count -ne 1) {
        throw 'Connect one V5 Brain directly by USB. Its user console port (PID_0501, MI_02) was not uniquely detected.'
    }
    $Port = $candidates[0].DeviceID
} else {
    $selected = $ports | Where-Object DeviceID -EQ $Port
    if ($selected -and $selected.PNPDeviceID -Match 'VID_2888&PID_0503') {
        throw "$Port belongs to the V5 Controller command interface, not the Brain user console. Connect the Brain directly by USB and use -Port Auto."
    }
}
$projectRoot = Split-Path -Parent $PSScriptRoot
$captureDirectory = Join-Path $projectRoot 'build'
New-Item -ItemType Directory -Path $captureDirectory -Force | Out-Null
$capturePath = Join-Path $captureDirectory ("drive-monitor-{0}.csv" -f (Get-Date -Format 'yyyyMMdd-HHmmss'))
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200,
    [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 500
$serial.NewLine = "`n"
$writer = $null
try {
    $serial.Open()
    $writer = [System.IO.StreamWriter]::new($capturePath, $false, [System.Text.UTF8Encoding]::new($false))
    $writer.WriteLine('sequence,time_ms,dt_ms,left_mask,right_mask,acceleration_mask,left_raw_rpm,right_raw_rpm,left_rpm,right_rpm,left_accel_rpm_s,right_accel_rpm_s,left_actual_mV,right_actual_mV,LF_rpm,LM_rpm,LBD_rpm,LBU_rpm,RF_rpm,RM_rpm,RBD_rpm,RBU_rpm,dropped,command_valid,enabled,left_command_mV,right_command_mV,mode,target_rpm')
    Write-Output "Reading $Port for $DurationSeconds seconds; log: $capturePath"
    $watch = [System.Diagnostics.Stopwatch]::StartNew()
    $previousPrintTime = -1.0
    $sampleCount = 0
    while ($watch.Elapsed.TotalSeconds -lt $DurationSeconds) {
        try { $line = $serial.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        $legacy = $line.StartsWith('DSV,')
        if (!$legacy -and !$line.StartsWith('DS2,')) { continue }
        $row = $line.Substring(4)
        $fields = $row.Split(',')
        $expected = if ($legacy) { 27 } else { 29 }
        if ($fields.Count -ne $expected) { continue }
        $invalid = $false
        foreach ($field in $fields) {
            $number = 0.0
            if (![double]::TryParse($field, [System.Globalization.NumberStyles]::Float,
                    [System.Globalization.CultureInfo]::InvariantCulture, [ref]$number) -or
                    [double]::IsNaN($number) -or [double]::IsInfinity($number)) { $invalid = $true; break }
        }
        if ($invalid) { continue }
        if ($legacy) { $fields += @('-1', '0') }
        $writer.WriteLine(($fields -join ','))
        $writer.Flush()
        $sampleCount++
        if ($watch.Elapsed.TotalSeconds - $previousPrintTime -ge 1) {
            Write-Output ("L={0} R={1} rpm  masks={2}/{3} dropped={4}" -f
                $fields[8], $fields[9], $fields[3], $fields[4], $fields[22])
            $previousPrintTime = $watch.Elapsed.TotalSeconds
        }
    }
    Write-Output "Captured $sampleCount samples."
    if ($sampleCount -eq 0) { Write-Output 'No DSV telemetry received. Confirm Bartmoss v1.1.0-voltage is running and the data link is connected.' }
} finally {
    if ($writer) { $writer.Dispose() }
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
