#ifndef RSCAMERA_H
#define RSCAMERA_H

#include "tighty/camera/depthcamera.h"

#include <QImage>
#include <QThread>


class RSCameraPrivate;

class RSCamera: public DepthCamera
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(RSCamera)
    QScopedPointer<RSCameraPrivate> const d_ptr;

public:
    RSCamera(const QString& serialNumber);
    ~RSCamera();

    // start and stop are slots
    void start() override;
    void stop() override;
};

#endif // RSCAMERA_H
