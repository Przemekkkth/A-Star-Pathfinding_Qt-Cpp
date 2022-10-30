QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

SOURCES += \
    src/main.cpp \
    src/scene.cpp \
    src/view.cpp

HEADERS += \
    src/node.h \
    src/scene.h \
    src/view.h
