QT       += core gui
QT       += serialport
QT       += core

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++14
CONFIG += release
# Debug 模式保留 qDebug，Release 模式禁用
CONFIG(debug, debug|release) {
    # Debug 模式，无需额外定义
    message("Debug mode: qDebug enabled")
} else {
    # Release 模式，禁用 qDebug
    DEFINES += QT_NO_DEBUG_OUTPUT
    message("Release mode: qDebug disabled")
}
# 设置编译选项
QMAKE_CXXFLAGS += -O0
QMAKE_CXXFLAGS += -g

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
    algorithm/Kinematics.cpp \
    camera/cameraworker.cpp \
    detect/detector.cpp \
    detect/utils.cpp \
    main.cpp \
    mainwindow.cpp \
    switch_type.cpp

HEADERS += \
    algorithm/Kinematics.h \
    camera/cameraworker.h \
    include/detect/cmdline.h \
    include/detect/detector.h \
    include/detect/utils.h \
    include/libfairino/robot.h \
    include/libfairino/robot_error.h \
    include/libfairino/robot_types.h\
    include/rm_service.h \
    include/switch_type.h \
    mainwindow.h




FORMS += \
    mainwindow.ui




INCLUDEPATH += $$PWD/include
INCLUDEPATH += $$PWD/algorithm
INCLUDEPATH += $$PWD/include/libfairino

LIBS+= -L$$PWD/lib -lRM_Service
# LIBS+= /home/pulan/DRobotNew/lib/libRM_Service.so

QMAKE_LFLAGS += -no-pie

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

INCLUDEPATH += /usr/local/include/opencv4

#LIBS += -L /usr/local/lib/ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs
LIBS += -L /usr/local/lib/libopencv_*.so
LIBS += -lopencv_core \
-lopencv_imgproc \
-lopencv_highgui \
-lopencv_ml \
-lopencv_video \
-lopencv_features2d \
-lopencv_calib3d \
-lopencv_objdetect \
-lopencv_flann

INCLUDEPATH += $$PWD/libs/eigen3


INCLUDEPATH +=/usr/local/include/librealsense2
LIBS += /usr/local/lib/librealsense2.so


INCLUDEPATH +=/usr/local/include/onnxruntime
LIBS += /usr/local/lib/libonnxruntime.so


INCLUDEPATH += $$PWD/auxlib/lib
DEPENDPATH += $$PWD/auxlib/lib


unix:!macx: LIBS += -L$$PWD/lib/ -lfairino

INCLUDEPATH += $$PWD/lib
DEPENDPATH += $$PWD/lib


