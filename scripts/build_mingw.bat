@echo off
REM ============================================================================
REM Clean, reproducible OUT-OF-SOURCE MinGW build for RobotKinematics.
REM
REM Handles the build traps:
REM   - Eigen is vendored at third_party/eigen (no dependency on a system Eigen,
REM     e.g. MSYS2's). The build is self-contained.
REM   - Removes any stale MSVC .qmake.stash so the MinGW compiler is detected.
REM   - Builds out-of-source so in-source artifacts never contaminate it.
REM
REM Override these via environment if your install differs:
REM   QT_MINGW_DIR  (default C:\Qt\6.8.2\mingw_64)
REM   MINGW_DIR     (default C:\Qt\Tools\mingw1310_64)
REM ============================================================================
setlocal
if "%QT_MINGW_DIR%"=="" set "QT_MINGW_DIR=C:\Qt\6.8.2\mingw_64"
if "%MINGW_DIR%"=="" set "MINGW_DIR=C:\Qt\Tools\mingw1310_64"
set "ROOT=%~dp0.."
set "BUILD=%ROOT%\_build_mingw"

set "PATH=%QT_MINGW_DIR%\bin;%MINGW_DIR%\bin;%PATH%"

REM Trap 2: a stale stash (root or build dir) from another compiler breaks detection.
del /q "%ROOT%\.qmake.stash" >nul 2>&1
if not exist "%BUILD%" mkdir "%BUILD%"
del /q "%BUILD%\.qmake.stash" "%BUILD%\.qmake.cache" "%BUILD%\Makefile" >nul 2>&1
cd /d "%BUILD%" || exit /b 1

qmake -spec win32-g++ "%ROOT%\RobotKinematics.pro" || ( echo [ERROR] qmake failed & exit /b 1 )
mingw32-make || ( echo [ERROR] mingw32-make failed & exit /b 1 )

echo [OK] MinGW build complete.
if exist "tests\RobotKinematicsTests.exe" ( tests\RobotKinematicsTests.exe & exit /b %errorlevel% )
if exist "tests\release\RobotKinematicsTests.exe" ( tests\release\RobotKinematicsTests.exe & exit /b %errorlevel% )
echo [WARN] test executable not found & exit /b 1
