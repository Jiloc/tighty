#ifndef RSCAMERA_P_H
#define RSCAMERA_P_H

#include <librealsense2/rs.hpp>

#include "rscameramanager.h"
#include "rsimagegeneratorworker.h"
#include "rscamera.h"

#include <QThread>

class RSCameraPrivate: public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(RSCameraPrivate)
    Q_DECLARE_PUBLIC(RSCamera)
    RSCamera * const q_ptr;

    rs2::pipeline m_pipe;
    rs2::config m_config;

    RSCameraManager m_cameraManager;

    RSImageGeneratorWorker m_worker;
    QThread m_workerThread;

    void start();
    void stop();
    void _stop();

    void onNewImage(QImage image);
    void onErrorOccurred(const QString &error);
    void onCameraConnected(const QString &serialNumber);
    void onCameraDisconnected(const QString &serialNumber);

public:
    RSCameraPrivate(RSCamera *camera);
    ~RSCameraPrivate();

signals:
    void started();


};

#endif // RSCAMERA_P_H
