TEMPLATE = app
CONFIG += console
CONFIG -= qt

CONFIG   -= app_bundle
QMAKE_CXXFLAGS += -std=c++0x

SOURCES += \
    GlutClass.cpp \
    PioneerRobot.cpp \
    Grid.cpp \
    main.cpp \
    Robot.cpp \
    Utils.cpp \
    Mcl.cpp \
    DroneRobot.cpp \
    MapGrid.cpp \
    Kernel.cpp \
    SomeKernels.cpp \
    densityheuristic.cpp \
    ColorCPU.cpp \
    vec2.cpp \
    vec3.cpp \
    vec4.cpp \
    colorheuristic.cpp \
    Heuristic.cpp \
    miheuristic.cpp

OTHER_FILES += \
    CONTROLE.txt

HEADERS += \
    Grid.h \
    GlutClass.h \
    PioneerRobot.h \
    Robot.h \
    Utils.h \
    Mcl.h \
    DroneRobot.h \
    MapGrid.h \
    Heuristic.h \
    Kernel.h \
    SomeKernels.h \
    ColorCPU.h \
    densityheuristic.h \
    mat3x3.h \
    vec2.h \
    vec3.h \
    vec4.h \
    RadiusVolumeTransferFunctions.h \
    colorheuristic.h \
    miheuristic.h

INCLUDEPATH+=/usr/local/Aria/include
INCLUDEPATH += -I/usr/local/include/opencv2
LIBS+=-L/usr/local/lib -L/usr/local/Aria/lib -lAria -lpthread -lglut -lGLEW -ldl -lrt `pkg-config opencv --libs` -lGL -lfreeimage
