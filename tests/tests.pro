TEMPLATE = app
TARGET = RobotKinematicsTests

QT += core testlib
QT -= gui

CONFIG += console c++17 warn_on
CONFIG -= app_bundle

INCLUDEPATH += \
    $$PWD/../include \
    $$PWD/../third_party/eigen

LIBS += -L$$OUT_PWD/../lib -lRobotKinematics

DESTDIR = $$OUT_PWD
OBJECTS_DIR = $$OUT_PWD/.obj
MOC_DIR = $$OUT_PWD/.moc
RCC_DIR = $$OUT_PWD/.rcc
UI_DIR = $$OUT_PWD/.ui

SOURCES += \
    TestMain.cpp \
    unit/DhAdapterTests.cpp \
    unit/PoseTests.cpp \
    unit/RobotModelConfigTests.cpp \
    unit/RobotModelValidatorTests.cpp \
    unit/SmokeTests.cpp \
    unit/UnitsTests.cpp \
    unit/JointLimitValidatorTests.cpp \
    unit/FrameToolTests.cpp \
    unit/ForwardKinematicsTests.cpp \
    unit/IKApiTests.cpp \
    unit/IKSolutionRankerTests.cpp \
    unit/NumericalIKSolverTests.cpp \
    unit/PostureResolverTests.cpp \
    unit/UrdfAdapterTests.cpp \
    integration/CustomPresetTests.cpp \
    integration/FrameToolFkTests.cpp \
    integration/IKIntegrationTests.cpp \
    integration/Virtual6DofTestArmTests.cpp

HEADERS += \
    unit/DhAdapterTests.h \
    unit/PoseTests.h \
    unit/RobotModelConfigTests.h \
    unit/RobotModelValidatorTests.h \
    unit/SmokeTests.h \
    unit/UnitsTests.h \
    unit/JointLimitValidatorTests.h \
    unit/FrameToolTests.h \
    unit/ForwardKinematicsTests.h \
    unit/IKApiTests.h \
    unit/IKSolutionRankerTests.h \
    unit/NumericalIKSolverTests.h \
    unit/PostureResolverTests.h \
    unit/UrdfAdapterTests.h \
    integration/CustomPresetTests.h \
    integration/FrameToolFkTests.h \
    integration/IKIntegrationTests.h \
    integration/Virtual6DofTestArmTests.h
