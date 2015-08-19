#-------------------------------------------------
#
# Project created by QtCreator 2015-08-05T14:42:58
#
#-------------------------------------------------

#QT       += core

#QT       -= gui

TARGET = MapfromImage
CONFIG   += console
CONFIG   -= app_bundle
QMAKE_CXXFLAGS += -std=c++0x

TEMPLATE = app


SOURCES += \
    ColorCPU.cpp \
    Kernel.cpp \
    SomeKernels.cpp \
    vec2.cpp \
    vec3.cpp \
    vec4.cpp \
    angleutil.cpp \
    densityheuristic.cpp \
    MapGrid.cpp \
    mainMap.cpp

HEADERS += \
    ColorCPU.h \
    Kernel.h \
    mat3x3.h \
    SomeKernels.h \
    vec2.h \
    vec3.h \
    vec4.h \
    RadiusVolumeTransferFunctions.h \
    angleutil.h \
    Heuristic.h \
    densityheuristic.h \
    MapGrid.h

INCLUDEPATH += -I/usr/local/include/opencv2
LIBS += -L/usr/local/lib `pkg-config opencv --libs`
