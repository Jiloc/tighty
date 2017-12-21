#-------------------------------------------------
#
# Project created by QtCreator 2017-12-07T15:37:10
#
#-------------------------------------------------
TARGET = tighty
TEMPLATE = lib

CONFIG += c++11

DEFINES += TIGHTY_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

THIRD_PARTY_DIR="$$shell_path($$PWD/3rdParty)"
LIBREALSENSE_DIR=$$shell_path($$THIRD_PARTY_DIR/librealsense/2.8.3)
LIBREALSENSE_LIB=$$shell_path($$LIBREALSENSE_DIR/lib/x64)
LIBREALSENSE_BIN=$$shell_path($$LIBREALSENSE_DIR/bin/x64)
LIBS += -L"$$LIBREALSENSE_LIB" -lrealsense2

INCLUDEPATH += "$$shell_path($$LIBREALSENSE_DIR/include)"


SOURCES += \
    tighty.cpp \
    camera/depthcamera.cpp \
    camera/realsense/rscamera.cpp \
    camera/realsense/rscameramanager.cpp \
    camera/realsense/rsimagegeneratorworker.cpp

HEADERS += \
        tighty.h \
        tighty_global.h \
    camera/depthcamera.h \
    camera/realsense/rscamera.h \
    camera/realsense/rscameramanager.h \
    camera/realsense/rsimagegeneratorworker.h \
    camera/realsense/rscamera_p.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

CONFIG(debug, debug|release) {
    DEST_DIR = $$shell_path($${OUT_PWD}/debug)
}
else {
    DEST_DIR = $$shell_path($${OUT_PWD}/release)
}
exists($$shell_path($$DEST_DIR/bin)) {
}
else {
    QMAKE_POST_LINK += $$quote($(MKDIR) $$shell_path($$DEST_DIR/bin) $$escape_expand(\n\t))
}
exists($$shell_path($$DEST_DIR/lib)) {
}
else {
    QMAKE_POST_LINK += $$quote($(MKDIR) $$shell_path($$DEST_DIR/lib) $$escape_expand(\n\t))
}

QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$DEST_DIR/tighty.dll) $$shell_path($$DEST_DIR/bin/) $$escape_expand(\n\t))
debug: QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$DEST_DIR/tighty.pdb) $$shell_path($$DEST_DIR/bin/) $$escape_expand(\n\t))

QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$DEST_DIR/tighty.lib) $$shell_path($$DEST_DIR/lib/))

