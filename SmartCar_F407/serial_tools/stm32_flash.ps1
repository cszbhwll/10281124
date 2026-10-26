# stm32_flash.ps1 - flash an Intel HEX file into STM32F407 through its built-in UART bootloader
# (USART1 = the CH340 USB port on the ALIENTEK Explorer board, one-key download circuit on DTR/RTS).
# Protocol: ST AN3155. Usage: powershell -ExecutionPolicy Bypass -File stm32_flash.ps1 -Port COM5 -Hex file.hex
param([string]$Port = 'COM5', [Parameter(Mandatory = $true)][string]$Hex)

$log = Join-Path $PSScriptRoot 'flash_result.txt'
if (Test-Path $log) { Remove-Item $log }
function L([string]$m) {
    $line = (Get-Date -Format 'HH:mm:ss') + '  ' + $m
    Write-Host $line
    Add-Content -Path $log -Value $line
}

# ---------- 1. read the HEX file into a flat image starting at 0x08000000 ----------
$FLASH = [uint32]0x08000000
$img = New-Object byte[] (1024 * 1024)
for ($i = 0; $i -lt $img.Length; $i++) { $img[$i] = 0xFF }
$top = 0
$upper = [uint32]0
foreach ($raw in [System.IO.File]::ReadAllLines($Hex)) {
    $r = $raw.Trim()
    if ($r.Length -lt 11 -or $r[0] -ne ':') { continue }
    $n = [Convert]::ToInt32($r.Substring(1, 2), 16)
    $off = [Convert]::ToInt32($r.Substring(3, 4), 16)
    $type = [Convert]::ToInt32($r.Substring(7, 2), 16)
    if ($type -eq 4) { $upper = [uint32]([Convert]::ToInt32($r.Substring(9, 4), 16)) * 65536; continue }
    if ($type -ne 0) { continue }
    $addr = [uint32]($upper + $off)
    if ($addr -lt $FLASH) { continue }
    $o = [int]($addr - $FLASH)
    for ($k = 0; $k -lt $n; $k++) { $img[$o + $k] = [byte]([Convert]::ToInt32($r.Substring(9 + 2 * $k, 2), 16)) }
    if ($o + $n -gt $top) { $top = $o + $n }
}
if ($top -le 0) { L "ERROR: no data found in $Hex"; exit 1 }
$top = [int]([math]::Ceiling($top / 256.0) * 256)
L ("HEX file: $Hex  ->  {0} bytes" -f $top)

# ---------- 2. serial helpers ----------
$p = New-Object System.IO.Ports.SerialPort $Port, 115200, ([System.IO.Ports.Parity]::Even), 8, ([System.IO.Ports.StopBits]::One)
$p.ReadTimeout = 1000
$p.WriteTimeout = 3000
try { $p.Open() } catch { L ("ERROR: cannot open $Port (close any serial-port program first): " + $_.Exception.Message); exit 1 }

function Send([byte[]]$d) { $p.Write($d, 0, $d.Length) }
function SendX([byte[]]$d) {            # data + XOR checksum
    $x = 0
    foreach ($b in $d) { $x = $x -bxor $b }
    $buf = New-Object byte[] ($d.Length + 1)
    [Array]::Copy($d, $buf, $d.Length)
    $buf[$d.Length] = [byte]$x
    Send $buf
}
function Ack([int]$ms = 1000) {
    $p.ReadTimeout = $ms
    $b = $p.ReadByte()
    if ($b -eq 0x79) { return }
    if ($b -eq 0x1F) { throw 'NACK from bootloader' }
    throw ('unexpected byte 0x{0:X2}' -f $b)
}
function Cmd([int]$c) { Send ([byte[]]@([byte]$c, [byte]($c -bxor 0xFF))); Ack }
function Addr([uint32]$a) { SendX ([byte[]]@([byte](($a -shr 24) -band 0xFF), [byte](($a -shr 16) -band 0xFF), [byte](($a -shr 8) -band 0xFF), [byte]($a -band 0xFF))); Ack }
function SetLine([string]$name, [bool]$on) { if ($name -eq 'DTR') { $p.DtrEnable = $on } else { $p.RtsEnable = $on } }

# ---------- 3. enter the bootloader: try every DTR/RTS combination of the one-key circuit ----------
$combos = @(
    @{ boot = 'RTS'; bootOn = $true;  rst = 'DTR'; rstOn = $true  },
    @{ boot = 'RTS'; bootOn = $false; rst = 'DTR'; rstOn = $true  },
    @{ boot = 'RTS'; bootOn = $true;  rst = 'DTR'; rstOn = $false },
    @{ boot = 'RTS'; bootOn = $false; rst = 'DTR'; rstOn = $false },
    @{ boot = 'DTR'; bootOn = $true;  rst = 'RTS'; rstOn = $true  },
    @{ boot = 'DTR'; bootOn = $false; rst = 'RTS'; rstOn = $true  },
    @{ boot = 'DTR'; bootOn = $true;  rst = 'RTS'; rstOn = $false },
    @{ boot = 'DTR'; bootOn = $false; rst = 'RTS'; rstOn = $false }
)
$ok = $null
foreach ($c in $combos) {
    SetLine $c.boot $c.bootOn
    SetLine $c.rst $c.rstOn
    Start-Sleep -Milliseconds 150
    SetLine $c.rst (-not $c.rstOn)
    Start-Sleep -Milliseconds 250
    $p.DiscardInBuffer()
    Send ([byte[]]@(0x7F))
    try {
        $p.ReadTimeout = 400
        $b = $p.ReadByte()
        if ($b -eq 0x79 -or $b -eq 0x1F) { $ok = $c; break }
    } catch { }
}
if ($null -eq $ok) {
    L 'ERROR: the board did not enter its bootloader.'
    L 'Check: BOOT0 and BOOT1 jumpers both on GND (factory setting), USB cable in the USB_UART port, board powered.'
    $p.Close(); exit 1
}
L ("bootloader OK (boot line {0}={1}, reset line {2}={3})" -f $ok.boot, $ok.bootOn, $ok.rst, $ok.rstOn)

try {
    # chip id
    Cmd 0x02
    $p.ReadTimeout = 1000
    $nid = $p.ReadByte()
    $chipId = 0
    for ($k = 0; $k -le $nid; $k++) { $chipId = ($chipId -shl 8) -bor $p.ReadByte() }
    Ack
    L ('chip id 0x{0:X3} (0x413 = STM32F405/407)' -f $chipId)

    # ---------- 4. erase (extended erase, mass erase) ----------
    L 'erasing flash (about 10-30 s) ...'
    Cmd 0x44
    Send ([byte[]]@(0xFF, 0xFF, 0x00))
    Ack 90000
    L 'erase done'

    # ---------- 5. write in 256-byte blocks ----------
    $written = 0
    for ($o = 0; $o -lt $top; $o += 256) {
        $blank = $true
        for ($k = 0; $k -lt 256; $k++) { if ($img[$o + $k] -ne 0xFF) { $blank = $false; break } }
        if ($blank) { continue }
        $blk = New-Object byte[] 257
        $blk[0] = 255                                   # N-1
        [Array]::Copy($img, $o, $blk, 1, 256)
        Cmd 0x31
        Addr ([uint32]($FLASH + $o))
        SendX $blk
        Ack 3000
        $written += 256
        if (($written % 8192) -eq 0) { L ("written {0} / {1} bytes" -f $written, $top) }
    }
    L ("write done, {0} bytes" -f $written)

    # ---------- 6. verify ----------
    $bad = 0
    for ($o = 0; $o -lt $top; $o += 256) {
        Cmd 0x11
        Addr ([uint32]($FLASH + $o))
        Send ([byte[]]@(255, 0))
        Ack
        $p.ReadTimeout = 2000
        for ($k = 0; $k -lt 256; $k++) {
            $b = $p.ReadByte()
            if ($b -ne $img[$o + $k]) { $bad++ }
        }
    }
    if ($bad -gt 0) { L "ERROR: verify found $bad wrong bytes"; $p.Close(); exit 1 }
    L 'verify OK'
}
catch {
    L ('ERROR during programming: ' + $_.Exception.Message)
    $p.Close(); exit 1
}

# ---------- 7. leave bootloader: BOOT0 back to 0, reset, then listen to the new program ----------
SetLine $ok.boot (-not $ok.bootOn)
SetLine $ok.rst $ok.rstOn
Start-Sleep -Milliseconds 150
SetLine $ok.rst (-not $ok.rstOn)
Start-Sleep -Milliseconds 100
$p.Close()

$q = New-Object System.IO.Ports.SerialPort $Port, 115200, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$q.DtrEnable = $false; $q.RtsEnable = $false
$q.Open()
Start-Sleep -Milliseconds 2500
$n = $q.BytesToRead
$bytes = New-Object byte[] $n
if ($n -gt 0) { [void]$q.Read($bytes, 0, $n) }
$q.Close()
$text = [System.Text.Encoding]::UTF8.GetString($bytes)
L 'program started. First output from the board:'
Add-Content -Path $log -Value $text -Encoding UTF8
Write-Host $text
L 'DONE'
exit 0
