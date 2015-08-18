#-------------------------------------------------
#
# Project created by QtCreator 2015-06-19T20:20:25
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Sphere3d
TEMPLATE = app

INCLUDEPATH += \
    .\    
    "/home/andrey/sdk/opencv-2.4.11/install/include"

SOURCES += main.cpp\
        mainwindow.cpp \
        ImageUtils.cpp \
        MathUtils.cpp \
        scene3d.cpp \
        trackball.cpp

HEADERS  += mainwindow.h \
    ImageUtils.h \
    MathUtils.h \
    dgegv.h \
    scene3d.h \
    trackball.h

FORMS    += mainwindow.ui

LIBS += -L"/home/andrey/sdk/opencv-2.4.11/install/lib" -lopencv_core -lopencv_features2d -lopencv_nonfree -lopencv_imgproc -lopencv_flann -lopencv_highgui -lopencv_objdetect -lopencv_calib3d
LIBS += -lgfortran -lblas -llapack -lstdc++ -lm
QMAKE_CXXFLAGS += -std=c++11

RESOURCES += \
    resource.qrc

DESTDIR = ./output
