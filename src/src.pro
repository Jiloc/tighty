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
LIBREALSENSE_DIR=$$shell_path($$THIRD_PARTY_DIR/librealsense/2.9.1a_no_OMP)

CONFIG(debug, debug|release) {
    LIBREALSENSE_BIN=$$shell_path($$LIBREALSENSE_DIR/bin/debug)
    LIBREALSENSE_LIB=$$shell_path($$LIBREALSENSE_DIR/lib/debug)
}
else {
    LIBREALSENSE_BIN=$$shell_path($$LIBREALSENSE_DIR/bin/release)
    LIBREALSENSE_LIB=$$shell_path($$LIBREALSENSE_DIR/lib/release)
}

LIBS += -L"$$LIBREALSENSE_LIB" -lrealsense2

INCLUDEPATH += "$$shell_path($$LIBREALSENSE_DIR/include)"

PCL_DIR=$$shell_path($$THIRD_PARTY_DIR/pcl/1.8.1)
PCL_LIB=$$shell_path($$PCL_DIR/lib)
PCL_BIN=$$shell_path($$PCL_DIR/bin)

CONFIG(debug, debug|release) {
    LIBS += -L"$$PCL_LIB" -lpcl_common_debug -lpcl_filters_debug -lpcl_keypoints_debug -lpcl_features_debug
}
else {
    LIBS += -L"$$PCL_LIB" -lpcl_common_release -lpcl_filters_release -lpcl_keypoints_release -lpcl_features_release
}
INCLUDEPATH += "$$shell_path($$PCL_DIR/include)"

BOOST_DIR=$$shell_path($$THIRD_PARTY_DIR/Boost/1.64)
BOOST_LIB=$$shell_path($$BOOST_DIR/lib)

CONFIG(debug, debug|release) {
    LIBS += -L"$$BOOST_LIB" -llibboost_date_time-vc140-mt-gd-1_64
}
else {
    LIBS += -L"$$BOOST_LIB" -llibboost_date_time-vc140-mt-1_64
}

INCLUDEPATH += "$$shell_path($$BOOST_DIR/include)"


EIGEN_DIR=$$shell_path($$THIRD_PARTY_DIR/Eigen/eigen3)
INCLUDEPATH += "$$shell_path($$EIGEN_DIR)"

FLANN_DIR=$$shell_path($$THIRD_PARTY_DIR/FLANN)
FLANN_LIB=$$shell_path($$FLANN_DIR/lib)
INCLUDEPATH += "$$shell_path($$FLANN_DIR/include)"

#CONFIG(debug, debug|release) {
#    LIBS += -L"$$FLANN_LIB" #-lflann-gd #-lflann_cpp-gd #-lflann_cpp_s-gd
#}
#else {
#    LIBS += -L"$$FLANN_LIB" -lflann #-lflann_cpp -lflann_cpp_s
#}

SOURCES += \
    tighty.cpp \
    camera/depthcamera.cpp \
    camera/realsense/rscamera.cpp \
    camera/realsense/rscameramanager.cpp \
    camera/realsense/rsframegeneratorworker.cpp \
    camera/realsense/rsframeprocessorworker.cpp

HEADERS += \
        tighty.h \
        tighty_global.h \
    camera/depthcamera.h \
    camera/realsense/rscamera.h \
    camera/realsense/rscameramanager.h \
    camera/realsense/rscamera_p.h \
    camera/realsense/rsframegeneratorworker.h \
    camera/realsense/rsframeprocessorworker.h

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

CONFIG(debug, debug|release) {
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$DEST_DIR/tighty.pdb) $$shell_path($$DEST_DIR/bin/) $$escape_expand(\n\t))
}

QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$DEST_DIR/tighty.lib) $$shell_path($$DEST_DIR/lib/))

