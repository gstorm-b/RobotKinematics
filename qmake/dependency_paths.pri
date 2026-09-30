# Resolves third-party dependency paths for every RobotKinematics qmake project.
#
# Precedence: qmake command line > environment variable > qmake/local_paths.pri
#
# qmake/local_paths.pri is machine-local and ignored by git. Create it from
# qmake/local_paths.pri.example.

ROBOTKINEMATICS_DEPENDENCY_VARS = \
    EIGEN_INCLUDE_DIR \
    COAL_ROOT \
    BOOST_ROOT \
    BOOST_INCLUDE_DIR \
    ASSIMP_ROOT \
    VTK_ROOT \
    VTK_VERSION \
    VTK_INCLUDEPATH \
    VTK_LIBPATH \
    VTK_BINPATH \
    VTK_LIB_SUFFIX

for(dependencyVar, ROBOTKINEMATICS_DEPENDENCY_VARS) {
    isEmpty($$dependencyVar) {
        dependencyEnvValue = $$getenv($$dependencyVar)
        !isEmpty(dependencyEnvValue): $$dependencyVar = $$dependencyEnvValue
    }
}

ROBOTKINEMATICS_LOCAL_PATHS_FILE = $$PWD/local_paths.pri
exists($$ROBOTKINEMATICS_LOCAL_PATHS_FILE): include($$ROBOTKINEMATICS_LOCAL_PATHS_FILE)
