# serial_cmd.ps1 - send the commands listed in serial_cmds.txt to the board (COM port, 115200 8N1)
# and save everything the board prints to serial_cmd_result.txt.
# Each line of serial_cmds.txt is one command; a line "wait 3000" just waits 3000 ms.
param([string]$Port = 'COM5')
# If the requested port does not exist (USB plugged into another socket -> new COM number),
# find the board automatically: first a CH340 USB-serial port, otherwise the only COM port present.
function Resolve-Port([string]$want) {
    $names = @([System.IO.Ports.SerialPort]::GetPortNames())
    if ($names -contains $want) { return $want }
    $ch = @()
    try {
        $ch = @(Get-CimInstance Win32_PnPEntity -ErrorAction Stop |
                Where-Object { $_.Name -match 'CH34\d' -and $_.Name -match '\(COM\d+\)' } |
                ForEach-Object { if ($_.Name -match '\((COM\d+)\)') { $Matches[1] } })
    } catch { }
    if ($ch.Count -ge 1) { return $ch[0] }
    if ($names.Count -eq 1) { return $names[0] }
    return $want
}
$Port = Resolve-Port $Port
$dir = $PSScriptRoot
$log = Join-Path $dir 'serial_cmd_result.txt'
$cmds = Join-Path $dir 'serial_cmds.txt'
$out = New-Object System.Collections.Generic.List[string]
$out.Add("port=$Port time=$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')")
$enc = [System.Text.Encoding]::UTF8
function Grab($p) {
    $n = $p.BytesToRead
    if ($n -le 0) { return '' }
    $b = New-Object byte[] $n
    [void]$p.Read($b, 0, $n)
    return $enc.GetString($b)
}
try {
    $p = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'One'
    $p.DtrEnable = $false; $p.RtsEnable = $false
    $p.Open()
    Start-Sleep -Milliseconds 1500
    $out.Add('>>> (board output before commands)'); $out.Add((Grab $p))
    foreach ($line in [System.IO.File]::ReadAllLines($cmds)) {
        $c = $line.Trim()
        if ($c -eq '') { continue }
        if ($c -match '^wait\s+(\d+)$') { Start-Sleep -Milliseconds ([int]$Matches[1]); $out.Add((Grab $p)); continue }
        $p.Write($enc.GetBytes("$c`r`n"), 0, $enc.GetByteCount("$c`r`n"))
        Start-Sleep -Milliseconds 2500
        $out.Add(">>> $c"); $out.Add((Grab $p))
    }
    $p.Close()
} catch {
    $out.Add('ERROR: ' + $_.Exception.Message)
}
[System.IO.File]::WriteAllLines($log, $out, $enc)
