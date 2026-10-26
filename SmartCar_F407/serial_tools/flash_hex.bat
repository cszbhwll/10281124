@echo off
rem Flash any .hex into the board over the USB cable (COM5).
rem Usage: drag a .hex file onto this .bat, or double-click it to flash ..\SmartCar_F407.hex
cd /d "%~dp0"
set HEX=%~1
if "%HEX%"=="" set HEX=%~dp0..\SmartCar_F407.hex
echo Flashing: %HEX%
echo Close any serial-port program (XCOM, CubeIDE terminal) first.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0stm32_flash.ps1" -Port COM5 -Hex "%HEX%"
echo.
echo If it says verify OK: press the RESET button on the board to start the new program.
pause
