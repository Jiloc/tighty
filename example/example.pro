QT += quick
CONFIG += c++11

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
    videoplayer.cpp

RESOURCES += qml.qrc

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

# Additional import path used to resolve QML modules just for Qt Quick Designer
QML_DESIGNER_IMPORT_PATH =


# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    videoplayer.h

CONFIG(debug, debug|release) {
    DEST_DIR = $$shell_path($${OUT_PWD}/debug)
    TIGHTY_DIR = $$shell_path($$PWD/../build/src/debug)
}
else {
    DEST_DIR = $$shell_path($${OUT_PWD}/release)
    TIGHTY_DIR = $$shell_path($$PWD/../build/src/release)
}

REALSENSE_DIR = $$shell_path($$PWD/../src/3rdParty/librealsense/2.8.3)
PCL_DIR = $$shell_path($$PWD/../src/3rdParty/pcl/1.8.1)
FLANN_DIR = $$shell_path($$PWD/../src/3rdParty/FLANN)

INCLUDEPATH += "$$shell_path($$PWD/../include)"
LIBS += -L"$$shell_path($$TIGHTY_DIR/lib)" -ltighty

QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$TIGHTY_DIR/bin/tighty.dll) $$DEST_DIR $$escape_expand(\n\t));
QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$REALSENSE_DIR/bin/x64/realsense2.dll) $$DEST_DIR $$escape_expand(\n\t));

CONFIG(debug, debug|release) {
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$TIGHTY_DIR/bin/tighty.pdb) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$REALSENSE_DIR/bin/x64/realsense2.pdb) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_common_debug.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_filters_debug.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_kdtree_debug.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_octree_debug.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_sample_consensus_debug.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_features_debug.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_keypoints_debug.dll) $$DEST_DIR $$escape_expand(\n\t));
    #QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$FLANN_DIR/bin/flann-gd.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_search_debug.dll) $$DEST_DIR);

}
else {
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_common_release.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_filters_release.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_kdtree_release.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_octree_release.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_sample_consensus_release.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_features_release.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_keypoints_release.dll) $$DEST_DIR $$escape_expand(\n\t));
    QMAKE_POST_LINK += $$quote($(COPY) $$shell_path($$PCL_DIR/bin/pcl_search_release.dll) $$DEST_DIR);
}
