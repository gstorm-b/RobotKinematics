@echo off
REM Third-party paths (Eigen, Coal/Boost/Assimp, VTK) come from qmake/local_paths.pri
REM (qmake command line > environment > local_paths.pri); see local_paths.pri.example.
setlocal

if "%QT_MSVC_DIR%"=="" set "QT_MSVC_DIR=C:\Qt\6.11.1\msvc2022_64"
if "%VCVARS%"=="" set "VCVARS=C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"

set "ROOT=%~dp0.."
set "BUILD=%ROOT%\build\msvc_mesh_coal"

call "%VCVARS%"
if not defined VCToolsInstallDir ( echo [ERROR] vcvars64 did not initialize the MSVC toolchain & exit /b 1 )
if not defined INCLUDE ( echo [ERROR] INCLUDE not set - vcvars64 setup incomplete & exit /b 1 )
set "PATH=%QT_MSVC_DIR%\bin;%PATH%"

if not exist "%BUILD%" mkdir "%BUILD%"
cd /d "%BUILD%" || exit /b 1

qmake "%ROOT%\RobotKinematics.pro" "CONFIG+=robotkinematics_mesh_collision" "MESH_COLLISION_BACKEND=coal" || ( echo [ERROR] qmake failed & exit /b 1 )
nmake /nologo || ( echo [ERROR] nmake failed & exit /b 1 )

echo [OK] MSVC Coal mesh build complete.
exit /b 0
