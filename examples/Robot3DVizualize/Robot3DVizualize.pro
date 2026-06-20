QT += widgets openglwidgets

TEMPLATE = app
TARGET = Robot3DVizualize

CONFIG += c++17 warn_on

include(vtk_config.pri)

INCLUDEPATH += \
    $$PWD/../../include \
    $$PWD/../../third_party/eigen

LIBS += -L$$PWD/../../_build_msvc/lib -lRobotKinematics
PRE_TARGETDEPS += $$PWD/../../_build_msvc/lib/RobotKinematics.lib

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    Robot3DVisualizerLogic.h \
    mainwindow.h

FORMS += \
    mainwindow.ui

RESOURCES += \
    resources.qrc

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
