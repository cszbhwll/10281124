@echo off
rem 本文件按 GBK 编码保存（中文 Windows 的命令行默认编码），请勿另存为 UTF-8
cd /d "%~dp0"
rem ============================================================
rem  编译并运行仿真：把 Core\Src 里最新的单片机代码编译成 sim.exe 再运行
rem  每次改完单片机代码（或 robot_config.h 里的参数），双击这个文件即可
rem  带参数 batch 运行（批量测试.bat 就是这样调用的）：编译后跑批量测试
rem ============================================================
set "MODE=%~1"

if exist "%~dp0w64devkit\bin\gcc.exe" set "PATH=%~dp0w64devkit\bin;%PATH%"
if exist "C:\w64devkit\bin\gcc.exe" set "PATH=C:\w64devkit\bin;%PATH%"
where gcc >nul 2>nul
if errorlevel 1 goto no_gcc

set SRC=..\Core\Src\imu_mpu6050.c ..\Core\Src\sys.c ..\Core\Src\field.c ..\Core\Src\motor_drv.c ..\Core\Src\chassis.c ..\Core\Src\nav.c ..\Core\Src\bus_servo.c ..\Core\Src\arm.c ..\Core\Src\vision.c ..\Core\Src\display.c ..\Core\Src\task_code.c ..\Core\Src\mission.c ..\Core\Src\console.c ..\Core\Src\obstacle.c ..\Core\Src\tof_vl53l0x.c world.c port_sim.c sim_main.c
echo 正在编译（用的是 ..\Core\Src 里的最新代码）……
if not exist build mkdir build
rem 先关掉还开着的旧仿真窗口（它会占用 sim.exe，导致新程序写不进去）
taskkill /f /im sim.exe >nul 2>nul
rem 每次编译用一个新文件名，旧文件即使被占用也不影响；能删的旧文件顺手删掉
del /q build\*.exe >nul 2>nul
set OUT=build\sim_%RANDOM%%RANDOM%.exe
rem 编译器的输出先存进 build\gcc_output.txt（每个文件只报第一处错误，免得刷屏把真正的错误顶上去）
gcc -std=c99 -O2 -Wall -Wno-unused-parameter -fmax-errors=1 -D_DEFAULT_SOURCE -DUSE_MPU6050=0 -DSIM_HAS_IMU -DSIM_HAS_TOF -I..\Core\Inc -I. -o %OUT% %SRC% -lm -lshell32 > build\gcc_output.txt 2>&1
if errorlevel 1 goto build_fail
set LOGSIZE=0
for %%A in ("build\gcc_output.txt") do set LOGSIZE=%%~zA
if "%LOGSIZE%"=="0" goto build_ok
echo.
echo 编译通过，但编译器给了下面这些警告（warning）。警告不会阻止运行，但常常就是 bug，建议看一眼：
echo ------------------------------------------------------------
call :show_file build\gcc_output.txt
echo ------------------------------------------------------------
:build_ok
echo 编译成功，开始仿真。
rem 顺便更新外面的 sim.exe（本机编译的文件，不会再弹"已保护你的电脑"）
copy /y %OUT% sim.exe >nul 2>nul
if /i "%MODE%"=="batch" goto run_batch
echo.
%OUT% --from-bat
exit /b 0

:run_batch
echo.
echo 开始批量测试：5 个任务码 x 2 个启停区 x 不同随机情况，再加轮子打滑、陀螺仪、相机看不到圆环等情况，约 10 秒……
%OUT% --batch > build\batch_output.txt 2>&1
copy /y build\batch_output.txt 批量测试结果.txt >nul 2>nul
echo ------------------------------------------------------------
call :show_file build\batch_output.txt
echo ------------------------------------------------------------
echo 结果也保存在 sim 文件夹里的“批量测试结果.txt”。
echo 每一行是一场：用时、抓取/放置次数、压线次数、各项得分、是否回到启停区、估算总分。
echo.
pause
exit /b 0

:no_gcc
echo.
echo [没找到 gcc 编译器]
echo 请先双击最外层文件夹里的“一键安装并运行仿真.bat”，它会自动安装编译器到 C:\w64devkit
echo.
pause
exit /b 1

:build_fail
echo.
echo ============================================================
echo  [编译失败] 你改过的代码里有语法错误，编译器拒绝生成新程序。
echo  所以这次不会运行仿真、也不会打开回放网页——否则你看到的会是旧程序的结果。
echo ============================================================
echo.
echo 编译器原话（英文）：
echo ------------------------------------------------------------
call :show_file build\gcc_output.txt
echo ------------------------------------------------------------
echo.
echo 怎么看：找带 error 的那一行，格式是  文件:行:列: error: 原因
echo   例：../Core/Src/field.c:40:20: error: ...  =  field.c 第 40 行第 20 个字符
echo   下面带竖线和 ^^~~~ 的两行，是编译器把出错的那行代码抄出来，^^ 指着出错的位置。
echo   用 STM32CubeIDE 或记事本打开这个文件，跳到那一行改。
echo.
echo 常见的几种 error 和改法：
echo   invalid suffix "f" on integer constant    小数常量要带小数点：100f 改成 100.0f
echo   expected ';' before ...                   上一句末尾漏了英文分号 ;
echo   expected ')' / '}' ...  或 end of input   括号没配对，少了右括号或右花括号
echo   'xxx' undeclared                          名字拼错了，或者没有定义这个变量
echo   implicit declaration of function 'xxx'    函数名拼错了，或者没 #include 对应的 .h
echo   报错里出现 \U0000ff1b、\U0000ff0c 或中文符号  用了中文标点（；，（）），改成英文半角
echo.
echo 改好后保存，再双击本文件。
echo 想先看上一次成功的结果：直接打开 三维仿真回放.html，页面右上角会写着那次是什么时候运行的。
echo.
pause
exit /b 1

rem 用 PowerShell 按 UTF-8 显示一个文本文件（编译器会抄出带中文注释的源代码行，直接 type 会乱码）
:show_file
powershell -NoProfile -ExecutionPolicy Bypass -Command "Get-Content -Encoding UTF8 '%~1'" 2>nul
if errorlevel 1 type "%~1"
exit /b 0
