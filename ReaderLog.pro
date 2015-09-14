#-------------------------------------------------
#
# Project created by QtCreator 2015-08-05T14:42:58
#
#-------------------------------------------------

#QT       += core

#QT       -= gui

TARGET = ReaderLog
CONFIG   += console
CONFIG   -= app_bundle
QMAKE_CXXFLAGS += -std=c++0x

TEMPLATE = app

SOURCES += \
    readerlog.cpp \
    mainReader.cpp \
    UTMConverter.cpp \
    PixelTransform.cpp \
    Utils.cpp

HEADERS += \
    readerlog.h \
    UTMConverter.h \
    PixelTransform.h \
    Utils.h

INCLUDEPATH += -I/usr/local/include/opencv2 /usr/include/eigen3
LIBS += -L/usr/local/lib `pkg-config opencv --libs`
