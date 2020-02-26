QT -= gui

CONFIG += c++14 console core
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

SOURCES += main.cpp \
    NetworkCalculus/networkcalculus.cpp \
    AFDX/afdx.cpp \
    SharedData/printer.cpp \
    SharedData/global_typedef.cpp \
    NetworkCalculus/ncdata.cpp \
    NetworkCalculus/FIFO/nc_fifo.cpp \
    NetworkCalculus/DRR/nc_drr.cpp \
    NetworkCalculus/nc_scheduling.cpp \
    NetworkCalculus/nccurves.cpp \
    NetworkCalculus/DRR/nc_drr_classic.cpp \
    NetworkCalculus/DRR/nc_drr_optimised.cpp \
    NetworkCalculus/DRR/nc_drr_quantumassignment.cpp \
    NetworkCalculus/DRR/nc_drr_quantumassignmentimproved.cpp

HEADERS += \
    NetworkCalculus/networkcalculus.h \
    AFDX/afdx.h \
    SharedData/printer.h \
    SharedData/global_typedef.h \
    NetworkCalculus/ncdata.h \
    NetworkCalculus/FIFO/nc_fifo.h \
    NetworkCalculus/DRR/nc_drr.h \
    NetworkCalculus/nc_scheduling.h \
    NetworkCalculus/nccurves.h \
    NetworkCalculus/DRR/nc_drr_classic.h \
    NetworkCalculus/DRR/nc_drr_optimised.h \
    NetworkCalculus/DRR/nc_drr_quantumassignment.h \
    NetworkCalculus/DRR/nc_drr_quantumassignmentimproved.h
