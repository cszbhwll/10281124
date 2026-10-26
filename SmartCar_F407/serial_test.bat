@echo off
cd /d "%~dp0"
echo Testing serial port COM5 ...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0serial_test.ps1" COM5
type serial_test_result.txt
echo.
echo Done. You can close this window.
timeout /t 15
