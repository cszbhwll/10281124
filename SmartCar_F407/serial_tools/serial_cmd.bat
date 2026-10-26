@echo off
cd /d "%~dp0"
echo Sending commands from serial_cmds.txt to COM5 ...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0serial_cmd.ps1" COM5
echo Done. This window closes in 5 s.
timeout /t 5
