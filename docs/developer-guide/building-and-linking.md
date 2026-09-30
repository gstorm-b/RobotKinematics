# Building and Linking

RobotKinematics builds as a **static library** (`RobotKinematics.lib` / `libRobotKinematics.a`).
This page covers building it and linking it into your own application.

## Dependencies

| Dependency | How it is provided | Why |
|---|---|---|
| **Qt 6 Core** | External (you install it) | The library links `QtCore`; `PresetJsonLoader` uses Qt's JSON + file IO. |
| **Eigen** | External, header-only. `EIGEN_INCLUDE_DIR` in `qmake/local_paths.pri` (see below) | All linear algebra and the `Pose` transform type. |
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

- the static library under the build's `lib/` directory, e.g. `build/msvc/lib`, and
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

## Build no-UI examples

Two console examples are available for library users:

```powershell
scripts\build_example_nachi_mz04_cli_msvc.bat
scripts\build_example_custom_preset_cli_msvc.bat
```

Outputs stay under each example folder:

- `examples/NachiMZ04Cli/build/msvc/release/NachiMZ04Cli.exe`
- `examples/CustomPresetCli/build/msvc/release/CustomPresetCli.exe`

See each example README for the expected output and the manual qmake flow.

## Third-party dependency paths

Third-party dependencies are not stored in the repository. Every path is declared once in
`qmake/local_paths.pri`, which is machine-local and ignored by git. Create it from the tracked template:

```powershell
copy qmake\local_paths.pri.example qmake\local_paths.pri
```

`qmake/dependency_paths.pri` resolves each variable with the precedence
**qmake command line > environment variable > `local_paths.pri`**, so Qt Creator builds and
`scripts\*.bat` builds see the same installs without exporting anything. The scripts do not
hard-code dependency paths.

| Variable | Needed by | This workstation |
|---|---|---|
| `EIGEN_INCLUDE_DIR` | every target | `C:/Program Files/PCL 1.15.1/3rdParty/Eigen3/include/eigen3` (Eigen 3.4.0) |
| `COAL_ROOT` | `CONFIG+=robotkinematics_mesh_collision` | `C:/build_packages/coal-3.0.3` |
| `BOOST_ROOT`, `BOOST_INCLUDE_DIR` | mesh collision | `C:/build_packages/boost-1.87.0` (`include/boost-1_87`) |
| `ASSIMP_ROOT` | mesh collision | `C:/build_packages/assimp-6.0.5` |
| `VTK_ROOT`, `VTK_VERSION` | Robot3DVizualize, mesh_collision_spike | `C:/build_packages/vtk/install-x64-cuda-qt`, `9.6` |

Targets that need third-party DLLs at runtime also get a generated
`robotkinematics_runtime_env.bat` in their build directory; `test_msvc_mesh_coal.bat` and
`run_mesh_collision_benchmark_msvc.bat` call it to put those DLL directories on `PATH`.

The prebuilt Coal package was compiled against PCL 1.15.1's Eigen 3.4.0, so keep
`EIGEN_INCLUDE_DIR` on the same headers. FCL and libccd are not required by Coal 3.x or by
any RobotKinematics target.

The repository does not build these packages. Their prebuilt installs under `C:\build_packages` are
consumed read-only; rebuilding them is managed outside this repository so a project script can never
overwrite an existing install.

### Shadow (out-of-source) build

The scripts above are shadow builds that use `build/msvc/` and `build/mingw/`.
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
    /path/to/eigen3

LIBS += -L/path/to/RobotKinematics/build/msvc/lib -lRobotKinematics

QT += core
CONFIG += c++17
```

### Option B — CMake (or any build system)

There is no CMake project shipped, but linking is standard once the library is built:

- **Include paths:** `include/` and your Eigen include directory (the one containing `Eigen/Core`).
- **Link:** the built `RobotKinematics` static lib **and** `Qt6::Core`.
- **Standard:** C++17.

Sketch:

```cmake
find_package(Qt6 REQUIRED COMPONENTS Core)

add_executable(myapp main.cpp)
target_compile_features(myapp PRIVATE cxx_std_17)
target_include_directories(myapp PRIVATE
    /path/to/RobotKinematics/include
    /path/to/eigen3)
target_link_libraries(myapp PRIVATE
    /path/to/RobotKinematics/build/msvc/lib/RobotKinematics.lib
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
