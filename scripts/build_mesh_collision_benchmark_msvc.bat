@echo off
setlocal

if "%QT_MSVC_DIR%"=="" set "QT_MSVC_DIR=C:\Qt\6.11.1\msvc2022_64"
if "%VCVARS%"=="" set "VCVARS=C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"

set "ROOT=%~dp0.."
if "%BUILD_PACKAGES_ROOT%"=="" set "BUILD_PACKAGES_ROOT=C:\build_packages"
if "%COAL_ROOT%"=="" set "COAL_ROOT=%BUILD_PACKAGES_ROOT%\coal-3.0.3"
if "%ASSIMP_ROOT%"=="" set "ASSIMP_ROOT=%BUILD_PACKAGES_ROOT%\assimp-6.0.5"
if "%BOOST_ROOT%"=="" set "BOOST_ROOT=%BUILD_PACKAGES_ROOT%\boost-1.87.0"
set "ROBOTKINEMATICS_LIB_DIR=%ROOT%\build\msvc_mesh_coal\lib"
set "BUILD=%ROOT%\build\tools\mesh_collision_benchmark"

call "%VCVARS%"
if not defined VCToolsInstallDir ( echo [ERROR] vcvars64 did not initialize the MSVC toolchain & exit /b 1 )
if not defined INCLUDE ( echo [ERROR] INCLUDE not set - vcvars64 setup incomplete & exit /b 1 )
set "PATH=%QT_MSVC_DIR%\bin;%COAL_ROOT%\bin;%ASSIMP_ROOT%\bin;%BOOST_ROOT%\lib;%PATH%"

call "%ROOT%\scripts\build_msvc_mesh_coal.bat" || exit /b 1

if not exist "%BUILD%" mkdir "%BUILD%"
cd /d "%BUILD%" || exit /b 1

qmake "%ROOT%\tools\mesh_collision_benchmark\mesh_collision_benchmark.pro" ^
    "CONFIG+=robotkinematics_mesh_collision" ^
    "MESH_COLLISION_BACKEND=coal" ^
    "COAL_ROOT=%COAL_ROOT%" ^
    "BOOST_ROOT=%BOOST_ROOT%" ^
    "ASSIMP_ROOT=%ASSIMP_ROOT%" ^
    "ROBOTKINEMATICS_LIB_DIR=%ROBOTKINEMATICS_LIB_DIR%" || ( echo [ERROR] qmake failed & exit /b 1 )
nmake /nologo || ( echo [ERROR] nmake failed & exit /b 1 )

echo [OK] MSVC mesh collision benchmark build complete.
exit /b 0
