#-------------------------------------------------
#
# Project created by QtCreator 2016-01-30T18:29:59
#
#-------------------------------------------------

QT       += widgets core gui

greaterThan(QT_MAJOR_VERSION, 4) {
    QT += widgets
    DEFINES += QT5
}


include(../../RTKLib.pri)

TARGET = rtklaunch_qt
TEMPLATE = app

INCLUDEPATH += ../../src/  
INCLUDEPATH += /usr/include
INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/local/include/gtsam
INCLUDEPATH += /usr/include/eigen3

LIBS += -lboost_system
LIBS += -ltbb
LIBS += -L /usr/local/lib -lgtsam
linux {
LIBS += -lpng
}

SOURCES += \ 
    launchmain.cpp \
    main.cpp

HEADERS  += \ 
    launchmain.h

FORMS    += \ 
    launchmain.ui

RESOURCES += \
    rtklaunch_qt.qrc

RC_FILE = rtklaunch_qt.rc
