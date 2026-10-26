@echo off
rem Serial terminal for the board (COM5, 115200). Chinese text shows correctly here.
rem Close the CubeIDE serial console first: one COM port can only be used by one program.
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0serial_term.ps1" -Port COM5 -Baud 115200
pause
