# Makes the DLL directories in ROBOTKINEMATICS_RUNTIME_PATHS available at run time:
#
# - Writes $$OUT_PWD/robotkinematics_runtime_env.bat, which prepends them to PATH. Scripts call it
#   before running an executable, so the runtime DLL paths come from the same resolution as the
#   build (see dependency_paths.pri).
# - Adds them as -L library search paths. Qt Creator's "Add build library search path to PATH"
#   run setting puts every -L directory on PATH, so runs started from Qt Creator find the same
#   DLLs. Extra linker search directories do not add any library to the link.
#
# Include this after appending to ROBOTKINEMATICS_RUNTIME_PATHS. Including it again later
# rewrites the file with the accumulated list.
win32 {
    ROBOTKINEMATICS_RUNTIME_PATHS = $$unique(ROBOTKINEMATICS_RUNTIME_PATHS)
    runtimePathsNative =
    for(runtimePath, ROBOTKINEMATICS_RUNTIME_PATHS) {
        runtimePathsNative += $$system_path($$runtimePath)
        !contains(LIBS, -L$$runtimePath): LIBS += -L$$runtimePath
    }
    runtimeEnvLines = "@echo off" "set PATH=$$join(runtimePathsNative, ;);%PATH%"
    !write_file($$OUT_PWD/robotkinematics_runtime_env.bat, runtimeEnvLines) {
        error("Could not write $$OUT_PWD/robotkinematics_runtime_env.bat")
    }
}
