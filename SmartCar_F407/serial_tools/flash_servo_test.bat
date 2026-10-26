@echo off
cd /d "%~dp0"
echo Flashing servo-test firmware into the board on COM5 ...
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0stm32_flash.ps1" -Port COM5 -Hex "%~dp0servo_test_fw.hex"
echo.
echo Finished. This window closes in 20 s.
timeout /t 20
