@echo off
setlocal

if "%QT_MSVC_DIR%"=="" set "QT_MSVC_DIR=C:\Qt\6.11.1\msvc2022_64"

set "ROOT=%~dp0.."
if "%BUILD_PACKAGES_ROOT%"=="" set "BUILD_PACKAGES_ROOT=C:\build_packages"
if "%COAL_ROOT%"=="" set "COAL_ROOT=%BUILD_PACKAGES_ROOT%\coal-3.0.3"
if "%ASSIMP_ROOT%"=="" set "ASSIMP_ROOT=%BUILD_PACKAGES_ROOT%\assimp-6.0.5"
if "%BOOST_ROOT%"=="" set "BOOST_ROOT=%BUILD_PACKAGES_ROOT%\boost-1.87.0"
set "BUILD=%ROOT%\build\msvc_mesh_coal"
set "EXE=%BUILD%\tests\RobotKinematicsTests.exe"

if not exist "%EXE%" (
    echo [ERROR] %EXE% not found. Run scripts\build_msvc_mesh_coal.bat first.
    exit /b 1
)

set "PATH=%QT_MSVC_DIR%\bin;%COAL_ROOT%\bin;%ASSIMP_ROOT%\bin;%BOOST_ROOT%\lib;%PATH%"
"%EXE%" || ( echo [ERROR] RobotKinematicsTests.exe failed & exit /b 1 )

echo [OK] MSVC Coal mesh tests complete.
exit /b 0
