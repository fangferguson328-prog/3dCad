TEMPLATE = app
TARGET = calibration_dataset_store_test
QT += core gui
QT -= widgets
CONFIG += console c++11 warn_on
CONFIG -= app_bundle

INCLUDEPATH += ..
SOURCES += \
    test_calibration_dataset_store.cpp \
    ../CalibrationCapturePolicy.cpp \
    ../CalibrationDatasetStore.cpp
HEADERS += \
    ../CalibrationCapturePolicy.h \
    ../CalibrationDatasetStore.h \
    ../RealSenseCalibrationCamera.h

# Intentionally no robot SDK, RealSense library, network, or live device sources.
win32-msvc*: QMAKE_CXXFLAGS += /utf-8
