#-------------------------------------------------
#
# Project created by QtCreator 2016-02-09T08:58:44
#
#-------------------------------------------------

QT       -= core gui

TARGET = RTKLib
TEMPLATE = lib
#CONFIG += staticlib
CONFIG += c++11
QMAKE_CXXFLAGS += -std=c++11

include(../RTKLib.pri)

INCLUDEPATH += /usr/include
INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/local/include/gtsam
INCLUDEPATH += /usr/include/eigen3

LIBS += -lboost_system
LIBS += -ltbb
LIBS += -L /usr/local/lib -lgtsam

QMAKE_CFLAGS += -Wall -ansi -pedantic -Wno-unused-but-set-variable  -DTRACE -g
#DEFINES -= UNICODE

SOURCES += rtkcmn.cpp \
    convkml.cpp \
    convrnx.cpp \
    convgpx.cpp \
    datum.cpp \
    download.cpp \
    ephemeris.cpp \
    geoid.cpp \
    gis.cpp \
    ionex.cpp \
    lambda.cpp \
    options.cpp \
    pntpos.cpp \
    postpos.cpp \
    ppp.cpp \
    ppp_ar.cpp \
    ppp_corr.cpp \
    preceph.cpp \
    qzslex.cpp \
    rcvraw.cpp \
    rinex.cpp \
    rtcm.cpp \
    rtcm2.cpp \
    rtcm3.cpp \
    rtcm3e.cpp \
    rtkpos.cpp \
    rtksvr.cpp \
    sbas.cpp \
    solution.cpp \
    stream.cpp \
    streamsvr.cpp \
    tides.cpp \
    tle.cpp \
    rcv/binex.cpp \
    rcv/crescent.cpp \
    rcv/gw10.cpp \
    rcv/javad.cpp \
    rcv/novatel.cpp \
    rcv/nvs.cpp \
    rcv/rcvlex.cpp \
    rcv/rt17.cpp \
    rcv/septentrio.cpp \
    rcv/skytraq.cpp \
    rcv/ss2.cpp \
    rcv/ublox.cpp \
    rcv/cmr.cpp \
    rcv/tersus.cpp

HEADERS += rtklib.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}
