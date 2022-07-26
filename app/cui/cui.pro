QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

include(../../RTKLib.pri)
INCLUDEPATH += ../../src/
INCLUDEPATH += /usr/include
INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/local/include/gtsam
INCLUDEPATH += /usr/include/eigen3

LIBS += -lboost_system
LIBS += -ltbb
LIBS += -L /usr/local/lib -lgtsam
linux{
    RTKLIB =../../src/libRTKLib.a
    LIBS += -lpng $${RTKLIB}
}
win32 {
    CONFIG(debug) {
        RTKLIB = ../../src/debug/libRTKLib.a
    } else {
        RTKLIB =../../src/release/libRTKLib.a
    }

    LIBS+= $${RTKLIB} -lWs2_32 -lwinmm
}
PRE_TARGETDEPS = $${RTKLIB}
SOURCES += \
    main.cpp \
    rcksrv_cui.cpp

HEADERS += \
    rcksrv_cui.h \
    stdafx.h


