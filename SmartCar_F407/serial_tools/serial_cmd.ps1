# serial_cmd.ps1 - send the commands listed in serial_cmds.txt to the board (COM port, 115200 8N1)
# and save everything the board prints to serial_cmd_result.txt.
# Each line of serial_cmds.txt is one command; a line "wait 3000" just waits 3000 ms.
param([string]$Port = 'COM5')
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
