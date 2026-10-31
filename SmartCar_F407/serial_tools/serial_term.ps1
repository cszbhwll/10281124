# serial_term.ps1 - 简易串口终端（UTF-8 中文正常显示）
# 用法：双击 serial_term.bat；或 powershell -File serial_term.ps1 -Port COM5 -Baud 115200
# 打字回车发送命令；上/下方向键翻历史命令；Esc 或 Ctrl+C 退出。
param([string]$Port = 'COM5', [int]$Baud = 115200)

[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
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

$asked = $Port
$Port = Resolve-Port $Port
if ($Port -ne $asked) { Write-Host "没有 $asked，自动改用板子所在的 $Port" -ForegroundColor Yellow }
try { $Host.UI.RawUI.WindowTitle = "串口终端 $Port $Baud" } catch { }

$p = New-Object System.IO.Ports.SerialPort $Port, $Baud, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$p.Encoding  = [System.Text.Encoding]::UTF8
$p.DtrEnable = $false          # 一键下载电路：DTR/RTS 都不动，免得把板子复位或拉进 bootloader
$p.RtsEnable = $false
try { $p.Open() } catch {
    Write-Host ""
    Write-Host "打不开 $Port。" -ForegroundColor Red
    Write-Host "1) 先把 CubeIDE 里的串口控制台断开（或关掉其它串口软件），同一个串口只能一个程序用；" -ForegroundColor Yellow
    Write-Host "2) 检查 USB 线插在板子的 USB_UART 口上；" -ForegroundColor Yellow
    Write-Host "3) 设备管理器里看看端口号是不是 $Port。" -ForegroundColor Yellow
    Write-Host ("这台电脑现在有的串口：" + ([System.IO.Ports.SerialPort]::GetPortNames() -join ', ')) -ForegroundColor Yellow
    exit 1
}

Write-Host "已连接 $Port  $Baud。输入命令按回车发送（help 看全部命令），上/下键翻历史，Esc 或 Ctrl+C 退出。" -ForegroundColor Green
Write-Host "提示：按板子 RESET 可以看到开机信息。" -ForegroundColor DarkGray

$dec   = [System.Text.Encoding]::UTF8.GetDecoder()
$buf   = New-Object byte[] 4096
$chars = New-Object char[] 8192
$line  = ''
$hist  = New-Object System.Collections.Generic.List[string]
$hi    = 0
[Console]::TreatControlCAsInput = $true

function Erase-Input([string]$s) {
    if ($s.Length -gt 0) {
        $w = [System.Text.Encoding]::GetEncoding(936).GetByteCount($s)   # 中文占两格
        [Console]::Write("`r" + (' ' * $w) + "`r")
    }
}
function Show-Input([string]$s) {
    if ($s.Length -gt 0) {
        $old = [Console]::ForegroundColor
        [Console]::ForegroundColor = 'Yellow'
        [Console]::Write($s)
        [Console]::ForegroundColor = $old
    }
}

try {
    while ($true) {
        # ---- 板子发来的数据 ----
        $n = 0
        try { $n = $p.BytesToRead } catch { Write-Host "`n串口断开了（USB 线拔了？）" -ForegroundColor Red; break }
        if ($n -gt 0) {
            if ($n -gt $buf.Length) { $n = $buf.Length }
            $got = $p.Read($buf, 0, $n)
            $c = $dec.GetChars($buf, 0, $got, $chars, 0)
            if ($c -gt 0) {
                $s = (New-Object string ($chars, 0, $c)) -replace "`r", ''
                Erase-Input $line
                [Console]::Write($s)
                Show-Input $line
            }
        }
        # ---- 键盘 ----
        while ([Console]::KeyAvailable) {
            $k = [Console]::ReadKey($true)
            $ctrl = ($k.Modifiers -band [ConsoleModifiers]::Control) -ne 0
            if ($k.Key -eq 'Escape' -or ($ctrl -and $k.Key -eq 'C')) { throw 'quit' }
            if ($k.Key -eq 'Enter') {
                [Console]::WriteLine()
                $p.Write($line + "`r`n")
                if ($line.Trim().Length -gt 0 -and ($hist.Count -eq 0 -or $hist[$hist.Count - 1] -ne $line)) { $hist.Add($line) }
                $hi = $hist.Count
                $line = ''
            } elseif ($k.Key -eq 'Backspace') {
                if ($line.Length -gt 0) { Erase-Input $line; $line = $line.Substring(0, $line.Length - 1); Show-Input $line }
            } elseif ($k.Key -eq 'UpArrow') {
                if ($hi -gt 0) { $hi--; Erase-Input $line; $line = $hist[$hi]; Show-Input $line }
            } elseif ($k.Key -eq 'DownArrow') {
                Erase-Input $line
                if ($hi -lt $hist.Count - 1) { $hi++; $line = $hist[$hi] } else { $hi = $hist.Count; $line = '' }
                Show-Input $line
            } elseif ([int]$k.KeyChar -ge 32) {
                $line += $k.KeyChar
                Show-Input ([string]$k.KeyChar)
            }
        }
        Start-Sleep -Milliseconds 15
    }
} catch {
    if ($_.Exception.Message -ne 'quit') { Write-Host ("`n出错：" + $_.Exception.Message) -ForegroundColor Red }
} finally {
    try { $p.Close() } catch { }
    [Console]::TreatControlCAsInput = $false
    Write-Host "`n串口已关闭。" -ForegroundColor Green
}
