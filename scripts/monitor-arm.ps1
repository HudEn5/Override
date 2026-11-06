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
$capturePath = Join-Path $captureDirectory ("arm-monitor-{0}.csv" -f (Get-Date -Format 'yyyyMMdd-HHmmss'))
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200,
    [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 500
$serial.NewLine = "`n"
$writer = $null
try {
    $serial.Open()
    $writer = [System.IO.StreamWriter]::new($capturePath, $false, [System.Text.UTF8Encoding]::new($false))
    $writer.WriteLine('tick,R1,R2,enabled,arm_mV,left_deg,right_deg,left_online,right_online,left_A,right_A')
    Write-Output "Reading $Port for $DurationSeconds seconds; log: $capturePath"
    $watch = [System.Diagnostics.Stopwatch]::StartNew()
    $previousButtons = ''
    $previousPrintTime = -1.0
    $sampleCount = 0
    while ($watch.Elapsed.TotalSeconds -lt $DurationSeconds) {
        try { $line = $serial.ReadLine().Trim() }
        catch [System.TimeoutException] { continue }
        if (!$line.StartsWith('BMV,')) { continue }
        $row = $line.Substring(4)
        $fields = $row.Split(',')
        if ($fields.Count -ne 11) { continue }
        $writer.WriteLine($row)
        $writer.Flush()
        $sampleCount++
        $buttons = "$($fields[1]),$($fields[2]),$($fields[3])"
        if ($buttons -ne $previousButtons -or $watch.Elapsed.TotalSeconds - $previousPrintTime -ge 1) {
            Write-Output ("R1={0} R2={1} EN={2} U_mV={3} AL={4} AR={5} ML={6} MR={7} IL={8} IR={9} tick={10}" -f
                $fields[1],$fields[2],$fields[3],$fields[4],$fields[5],$fields[6],
                $fields[7],$fields[8],$fields[9],$fields[10],$fields[0])
            $previousButtons = $buttons
            $previousPrintTime = $watch.Elapsed.TotalSeconds
        }
    }
    Write-Output "Captured $sampleCount samples."
    if ($sampleCount -eq 0) { Write-Output 'No BMV telemetry received. Confirm Bartmoss v1.1.0-voltage is running and the data link is connected.' }
} finally {
    if ($writer) { $writer.Dispose() }
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
