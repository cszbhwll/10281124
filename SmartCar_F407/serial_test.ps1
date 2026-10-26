# Serial test: sends a few debug commands to the STM32 over the CH340 port and saves the replies.
# Only read-only commands are sent (sscan / sping / sread); nothing moves.
param([string]$Port = 'COM5')
$log = Join-Path $PSScriptRoot 'serial_test_result.txt'
$out = New-Object System.Collections.Generic.List[string]
$out.Add("port=$Port time=$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')")
$out.Add("ports on this PC: " + ([System.IO.Ports.SerialPort]::GetPortNames() -join ','))
try {
    $p = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'One'
    $p.DtrEnable = $false; $p.RtsEnable = $false      # keep the one-key-download circuit from resetting the MCU
    $p.Encoding = [System.Text.Encoding]::UTF8
    $p.Open()
    Start-Sleep -Milliseconds 300
    $first = $p.ReadExisting()
    $out.Add(">>> (already in buffer)"); $out.Add($first)
    foreach ($c in @('sscan', 'sping', 'sread')) {
        $p.Write("$c`r`n")
        Start-Sleep -Milliseconds 3000
        $out.Add(">>> $c"); $out.Add($p.ReadExisting())
    }
    $p.Close()
} catch {
    $out.Add("ERROR: " + $_.Exception.Message)
}
$out | Out-File -FilePath $log -Encoding utf8
