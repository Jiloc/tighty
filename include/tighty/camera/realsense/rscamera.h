#ifndef RSCAMERA_H
#define RSCAMERA_H

#include "tighty/camera/depthcamera.h"

#include <QUrl>
#include <QScopedPointer>

const QString DEFAULT_DEVICE("*");

class RSCameraPrivate;

class RSCamera: public DepthCamera
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(RSCamera)
    QScopedPointer<RSCameraPrivate> const d_ptr;

public:
    RSCamera(const QString &serialNumber=DEFAULT_DEVICE);
    ~RSCamera();

    // start, stop and record are slots
    void start() override;
    void stop() override;
    void record() override;

public slots:
    void playback(const QString &filename);
    void playback(const QUrl& url);

};
#endif // RSCAMERA_H
