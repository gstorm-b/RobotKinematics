@echo off
setlocal

if "%QT_MSVC_DIR%"=="" set "QT_MSVC_DIR=C:\Qt\6.11.1\msvc2022_64"

set "ROOT=%~dp0.."
set "EXE=%ROOT%\build\tools\mesh_collision_benchmark\release\mesh_collision_benchmark.exe"

if not exist "%EXE%" (
    echo [ERROR] %EXE% not found. Run scripts\build_mesh_collision_benchmark_msvc.bat first.
    exit /b 1
)

set "PATH=%QT_MSVC_DIR%\bin;%PATH%"
set "RUNTIME_ENV=%ROOT%\build\tools\mesh_collision_benchmark\robotkinematics_runtime_env.bat"
if not exist "%RUNTIME_ENV%" (
    echo [ERROR] %RUNTIME_ENV% not found. Run scripts\build_mesh_collision_benchmark_msvc.bat first.
    exit /b 1
)
call "%RUNTIME_ENV%"
"%EXE%" %* || ( echo [ERROR] mesh_collision_benchmark.exe failed & exit /b 1 )

echo [OK] MSVC mesh collision benchmark run complete.
exit /b 0
