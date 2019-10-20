#-------------------------------------------------
#
# Project created by QtCreator 2019-06-19T10:38:09
#
#-------------------------------------------------

QT += core gui

QMAKE_CXXFLAGS += -std=c++11
CONFIG += console
CONFIG -= app_bundle
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = BinocularVIO
TEMPLATE = app

SOURCES += main.cpp\
    BinocularVIO.cpp \
    Camera.cpp \
    CameraCalib.cpp \
    Config.cpp \
    GroundTruth.cpp \
    IMU.cpp \
    IMUCalib.cpp \
    ImageProcessor.cpp \
    LocalMap.cpp \
    RANSAC.cpp \
        mainwindow.cpp


HEADERS  += mainwindow.h \
    BinocularVIO.h \
    Camera.h \
    CameraCalib.h \
    Config.h \
    GroundTruth.h \
    IMU.h \
    IMUCalib.h \
    ImageProcessor.h \
    LocalMap.h \
    RANSAC.h \
    utils.h


FORMS    += mainwindow.ui

DISTFILES +=

#LIBS += /usr/lib/x86_64-linux-gnu/libtbb.so /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so -lopencv_calib3d /usr/lib/x86_64-linux-gnu/libopencv_contrib.so -lopencv_contrib /usr/lib/x86_64-linux-gnu/libopencv_core.so -lopencv_core /usr/lib/x86_64-linux-gnu/libopencv_features2d.so -lopencv_features2d /usr/lib/x86_64-linux-gnu/libopencv_flann.so -lopencv_flann /usr/lib/x86_64-linux-gnu/libopencv_gpu.so -lopencv_gpu /usr/lib/x86_64-linux-gnu/libopencv_highgui.so -lopencv_highgui /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so -lopencv_imgproc /usr/lib/x86_64-linux-gnu/libopencv_legacy.so -lopencv_legacy /usr/lib/x86_64-linux-gnu/libopencv_ml.so -lopencv_ml /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so -lopencv_objdetect /usr/lib/x86_64-linux-gnu/libopencv_ocl.so -lopencv_ocl /usr/lib/x86_64-linux-gnu/libopencv_photo.so -lopencv_photo /usr/lib/x86_64-linux-gnu/libopencv_stitching.so -lopencv_stitching /usr/lib/x86_64-linux-gnu/libopencv_superres.so -lopencv_superres /usr/lib/x86_64-linux-gnu/libopencv_ts.so -lopencv_ts /usr/lib/x86_64-linux-gnu/libopencv_video.so -lopencv_video /usr/lib/x86_64-linux-gnu/libopencv_videostab.so -lopencv_videostab
LIBS += /usr/local/lib/libopencv_world.so.4.0.1
LIBS += /usr/lib/x86_64-linux-gnu/libtbb.so
INCLUDEPATH += /usr/local/include/opencv4


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../usr/local/lib/release/ -lopencv_world
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../usr/local/lib/debug/ -lopencv_world
#else:unix: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lopencv_world

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include
