# Eigen is header-only and supplied from outside the repository.
#
# Default: the Eigen 3.4.0 headers shipped with PCL 1.15.1. This is the same Eigen
# the prebuilt Coal package under C:\build_packages\coal-3.0.3 was compiled against,
# so library and Coal adapter see identical Eigen headers.
#
# Override with qmake "EIGEN_INCLUDE_DIR=<dir>" or the EIGEN_INCLUDE_DIR environment
# variable. <dir> must contain Eigen/Core.
isEmpty(EIGEN_INCLUDE_DIR): EIGEN_INCLUDE_DIR = $$(EIGEN_INCLUDE_DIR)
isEmpty(EIGEN_INCLUDE_DIR): EIGEN_INCLUDE_DIR = "C:/Program Files/PCL 1.15.1/3rdParty/Eigen3/include/eigen3"

!exists("$$EIGEN_INCLUDE_DIR/Eigen/Core") {
    error("Eigen headers not found at '$$EIGEN_INCLUDE_DIR'. Set EIGEN_INCLUDE_DIR to a directory containing Eigen/Core.")
}

INCLUDEPATH += "$$EIGEN_INCLUDE_DIR"
