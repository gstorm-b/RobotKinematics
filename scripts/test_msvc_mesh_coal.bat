@echo off
setlocal

if "%QT_MSVC_DIR%"=="" set "QT_MSVC_DIR=C:\Qt\6.11.1\msvc2022_64"

set "ROOT=%~dp0.."
set "BUILD=%ROOT%\build\msvc_mesh_coal"
set "EXE=%BUILD%\tests\RobotKinematicsTests.exe"

if not exist "%EXE%" (
    echo [ERROR] %EXE% not found. Run scripts\build_msvc_mesh_coal.bat first.
    exit /b 1
)

set "PATH=%QT_MSVC_DIR%\bin;%PATH%"
set "RUNTIME_ENV=%BUILD%\tests\robotkinematics_runtime_env.bat"
if not exist "%RUNTIME_ENV%" (
    echo [ERROR] %RUNTIME_ENV% not found. Run scripts\build_msvc_mesh_coal.bat first.
    exit /b 1
)
call "%RUNTIME_ENV%"
cd /d "%ROOT%" || exit /b 1
"%EXE%" || ( echo [ERROR] RobotKinematicsTests.exe failed & exit /b 1 )

echo [OK] MSVC Coal mesh tests complete.
exit /b 0
