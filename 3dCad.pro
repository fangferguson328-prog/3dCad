QT       += core gui opengl

QT += opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    common.cpp \
    drawingcanvas.cpp \
    HarmoRobot.cpp \
    iocontrol.cpp \
    main.cpp \
    mainwindow.cpp \
    HarmoMotionPlan.cpp \
    CalibrationCapturePolicy.cpp \
    RealSenseCalibrationCamera.cpp \
    CalibrationDatasetStore.cpp \
    CalibrationCaptureDialog.cpp \
    XPLGenerator.cpp \
    tinyxml2.cpp \
    myweldtrack.cpp \
    openglwidget.cpp \
    planeselection.cpp \
    robot.cpp \
    RobotController.cpp \
    rscamera.cpp \
    weldiosetdlg.cpp

HEADERS += \
    common.h \
    drawingcanvas.h \
    HarmoRobot.h \
    iocontrol.h \
    mainwindow.h \
    HarmoMotionPlan.h \
    CalibrationCapturePolicy.h \
    RealSenseCalibrationCamera.h \
    CalibrationDatasetStore.h \
    CalibrationCaptureDialog.h \
    XPLGenerator.h \
    tinyxml2.h \
    myweldtrack.h \
    openglwidget.h \
    planeselection.h \
    robot.h \
    RobotController.h \
    RobotTypes.h \
    rscamera.h \
    weldiosetdlg.h

FORMS += \
    iocontrol.ui \
    mainwindow.ui \
    weldiosetdlg.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    shaders/fragmentshader.fsh \
    shaders/vertexshader.vsh

RESOURCES += \
    resources.qrc

INCLUDEPATH += $$PWD/includes/glm/

include($$PWD/includes/glm/glm.pri)

DISTFILES += \
    includes/glm/glm.pri

# The bundled Windows dependencies and the VS x64 project are 64-bit. Use
# an x64 Qt/MSVC kit; keep Debug and Release import libraries separate.
win32 {
    INCLUDEPATH += $$PWD/includes/glfw/include \
                   "$$PWD/includes/Intel RealSense SDK 2.0/include" \
                   $$PWD/includes/opencv/build/include \
                   $$PWD/includes/opencv/build/include/opencv2 \
                   "$$PWD/includes/SDK V2.5/sdk-x64/include" \
                   "$$PWD/includes/KW_SDK_D_1.0/SDK_1.0/SDK_windows_1.0/lib/CPP" \
                   $$PWD/includes/robot/64
    DEPENDPATH += $$INCLUDEPATH

    LIBS += -lopengl32 -lshell32 \
            $$PWD/includes/glfw/lib-vc2015/glfw3dll.lib \
            "$$PWD/includes/Intel RealSense SDK 2.0/lib/x64/realsense2.lib" \
            $$PWD/includes/robot/64/libharmorobotservice.lib \
            "$$PWD/includes/KW_SDK_D_1.0/SDK_1.0/SDK_windows_1.0/lib/CPP/camera.lib" \
            "$$PWD/includes/KW_SDK_D_1.0/SDK_1.0/SDK_windows_1.0/lib/CPP/enumerate.lib"

    CONFIG(debug, debug|release) {
        LIBS += $$PWD/includes/opencv/build/x64/vc16/lib/opencv_world470d.lib \
                "$$PWD/includes/SDK V2.5/sdk-x64/libs/x64/EftSdkd.lib" \
                "$$PWD/includes/SDK V2.5/sdk-x64/libs/x64/log4cpp-d.lib" \
                "$$PWD/includes/SDK V2.5/sdk-x64/libs/x64/rlibcpp-bcc-d.lib" \
                "$$PWD/includes/SDK V2.5/sdk-x64/libs/x64/rlibcpp-tool-d.lib"
    } else {
        LIBS += $$PWD/includes/opencv/build/x64/vc16/lib/opencv_world470.lib \
                "$$PWD/includes/SDK V2.5/sdk-x64/libs/x64/EftSdk.lib" \
                "$$PWD/includes/SDK V2.5/sdk-x64/libs/x64/log4cpp.lib" \
                "$$PWD/includes/SDK V2.5/sdk-x64/libs/x64/rlibcpp-bcc.lib" \
                "$$PWD/includes/SDK V2.5/sdk-x64/libs/x64/rlibcpp-tool.lib"
    }
}

DEFINES += BUILD_MYDLL
