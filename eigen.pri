# Eigen is header-only and supplied from outside the repository.
#
# EIGEN_INCLUDE_DIR (the directory containing Eigen/Core) is resolved by
# qmake/dependency_paths.pri: qmake command line > environment > qmake/local_paths.pri.
include($$PWD/qmake/dependency_paths.pri)

isEmpty(EIGEN_INCLUDE_DIR) {
    error("EIGEN_INCLUDE_DIR is not set. Copy qmake/local_paths.pri.example to qmake/local_paths.pri and set it there.")
}
!exists("$$EIGEN_INCLUDE_DIR/Eigen/Core") {
    error("Eigen headers not found at '$$EIGEN_INCLUDE_DIR'. Check EIGEN_INCLUDE_DIR (qmake argument, environment variable, or qmake/local_paths.pri).")
}

INCLUDEPATH += "$$EIGEN_INCLUDE_DIR"
