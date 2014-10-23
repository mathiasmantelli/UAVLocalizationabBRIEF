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
    Utils.cpp

OTHER_FILES += \
    CONTROLE.txt

HEADERS += \
    Grid.h \
    GlutClass.h \
    PioneerRobot.h \
    Robot.h \
    Utils.h

INCLUDEPATH+=/usr/local/Aria/include
LIBS+=-L/usr/local/Aria/lib -lAria -lpthread -lglut -ldl -lrt -lGL -lfreeimage
