param([string]$KeilRoot = '')
$ErrorActionPreference = 'Stop'
if (-not $KeilRoot) {
    foreach ($candidate in @('D:\Keil_v5','C:\Keil_v5','C:\Keil')) {
        if (Test-Path -LiteralPath (Join-Path $candidate 'UV4\UV4.exe')) { $KeilRoot = $candidate; break }
    }
}
if (-not $KeilRoot) { throw 'Keil not found. Use -KeilRoot with the installation path.' }
$p = Start-Process -FilePath (Join-Path $KeilRoot 'UV4\UV4.exe') -ArgumentList @('-r','GroundButtons_F407.uvprojx','-j0','-o','build.log') -WorkingDirectory $PSScriptRoot -WindowStyle Hidden -PassThru -Wait
Get-Content -LiteralPath (Join-Path $PSScriptRoot 'build.log')
exit $p.ExitCode
