# Building and Linking

RobotKinematics builds as a **static library** (`RobotKinematics.lib` / `libRobotKinematics.a`).
This page covers building it and linking it into your own application.

## Dependencies

| Dependency | How it is provided | Why |
|---|---|---|
| **Qt 6 Core** | External (you install it) | The library links `QtCore`; `PresetJsonLoader` uses Qt's JSON + file IO. |
| **Eigen** | Bundled in `third_party/eigen` (header-only) | All linear algebra and the `Pose` transform type. |
| **C++17** | Compiler flag | Standard the library targets. |
| **Qt Test** | External | Only needed to build/run the test suite, not to use the library. |

> Even if you never touch JSON presets, the static library still links `QtCore` (it is part of
> the same translation units). Your application must therefore link `Qt6Core`. If a Qt-free
> build is a hard requirement for you, raise it — `PresetJsonLoader` is the only Qt-dependent
> compilation unit and could be made optional.

Primary compiler is **MSVC**; **MinGW** is a compatibility target.

## Build the library and tests

From the repository root:

```powershell
scripts\build_msvc.bat
scripts\test_msvc.bat
```

This produces:

- the static library under the build's `lib/` directory, and
- the test executable `RobotKinematicsTests.exe` under the build's `tests/` directory.

A successful run prints one line per suite ending in `PASS` and exits with code `0`.

> Run the test executable from the **repository root**. Preset-loading tests look for
> `presets/*.json` using relative paths.

For a clean MSVC rebuild plus tests:

```powershell
scripts\rebuild_msvc.bat
```

For MinGW compatibility:

```powershell
scripts\build_mingw.bat
scripts\test_mingw.bat
```

### Shadow (out-of-source) build

The scripts above are shadow builds that use `_build_msvc/` and `_build_mingw/`.
If you need to run qmake manually, create a build directory, run `qmake` pointing
at the top-level `.pro`, then build:

```powershell
mkdir build_cli; cd build_cli
qmake ..\RobotKinematics.pro
nmake
cd ..
.\build_cli\tests\RobotKinematicsTests.exe
```

## Linking RobotKinematics into your application

You have two practical options.

### Option A — add the sources/headers to your own qmake project

Point your `.pro` at the include roots and link the built static lib:

```pro
INCLUDEPATH += \
    /path/to/RobotKinematics/include \
    /path/to/RobotKinematics/third_party/eigen

LIBS += -L/path/to/RobotKinematics/build/lib -lRobotKinematics

QT += core
CONFIG += c++17
```

### Option B — CMake (or any build system)

There is no CMake project shipped, but linking is standard once the library is built:

- **Include paths:** `include/` and `third_party/eigen/`.
- **Link:** the built `RobotKinematics` static lib **and** `Qt6::Core`.
- **Standard:** C++17.

Sketch:

```cmake
find_package(Qt6 REQUIRED COMPONENTS Core)

add_executable(myapp main.cpp)
target_compile_features(myapp PRIVATE cxx_std_17)
target_include_directories(myapp PRIVATE
    /path/to/RobotKinematics/include
    /path/to/RobotKinematics/third_party/eigen)
target_link_libraries(myapp PRIVATE
    /path/to/RobotKinematics/build/lib/RobotKinematics.lib
    Qt6::Core)
```

At runtime your app needs the Qt 6 Core shared library on its path (e.g. add the Qt `bin`
directory to `PATH` on Windows).

## Including headers

Include via the `RobotKinematics/...` prefix (the include root is `include/`):

```cpp
#include <RobotKinematics/Kinematics/SerialRobotKinematics.h>
#include <RobotKinematics/Kinematics/ForwardKinematics.h>
#include <RobotKinematics/Model/SerialRobotConfigBuilder.h>
#include <RobotKinematics/Presets/PresetJsonLoader.h>
```

There is no single umbrella header; include the specific headers you use. See
[api-reference.md](api-reference.md) for the per-module header list.
