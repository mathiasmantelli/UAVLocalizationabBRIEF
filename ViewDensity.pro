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




INCLUDEPATH += -I/usr/local/include/opencv2
LIBS += -L/usr/local/lib `pkg-config opencv --libs`

SOURCES += \
    mainView.cpp \
    MapGrid.cpp \
    SomeKernels.cpp \
    Kernel.cpp

HEADERS += \
    MapGrid.h \
    SomeKernels.h \
    Kernel.h
