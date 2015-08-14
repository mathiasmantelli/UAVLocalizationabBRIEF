TEMPLATE = app
CONFIG += console
CONFIG -= qt
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
    SomeKernels.cpp

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
    SomeKernels.h

INCLUDEPATH+=/usr/local/Aria/include
LIBS+=-L/usr/local/lib -L/usr/local/Aria/lib -lAria -lpthread -lglut -lGLEW -ldl -lrt -lGL -lfreeimage `pkg-config opencv --libs`
