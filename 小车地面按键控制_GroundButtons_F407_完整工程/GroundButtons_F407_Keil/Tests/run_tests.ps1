param([string]$ZigExe='zig')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$build=Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Path $build -Force | Out-Null
$argsList=@('cc','-std=c99','-Wall','-Wextra','-Werror',
    '-I',(Join-Path $root 'Core\Inc'),'-I',(Join-Path $root 'Tune'),
    (Join-Path $PSScriptRoot 'test_ground.c'),
    (Join-Path $root 'Core\Src\ground_drive.c'),
    (Join-Path $root 'Core\Src\motor_drv.c'),
    (Join-Path $root 'Core\Src\debug_sys.c'),'-o',(Join-Path $build 'test_ground.exe'))
& $ZigExe @argsList
if ($LASTEXITCODE -ne 0) { throw 'Ground test compilation failed.' }
& (Join-Path $build 'test_ground.exe')
if ($LASTEXITCODE -ne 0) { throw 'Ground test failed.' }
