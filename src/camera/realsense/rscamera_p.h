#ifndef RSCAMERA_P_H
#define RSCAMERA_P_H

#include <librealsense2/rs.hpp>

#include "rscameramanager.h"
#include "rsframegeneratorworker.h"
#include "rsframeprocessorworker.h"
#include "rscamera.h"

#include <QThread>

class RSCameraPrivate: public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(RSCameraPrivate)
    Q_DECLARE_PUBLIC(RSCamera)
    RSCamera * const q_ptr;

    QString m_scanningDeviceSerial;
    rs2::pipeline m_pipe;
    rs2::pipeline_profile m_profile;
    rs2::frame_queue m_queue;
    rs2::config m_config;
    bool m_isPlayback;

    RSCameraManager m_cameraManager;

    RSFrameGeneratorWorker m_generator;
    RSFrameProcessorWorker m_processor;
    QThread m_generatorThread;
    QThread m_processorThread;

    void playback(const QString &filename);
    void start();
    void stop();
    void _stop();
    void record();

    void onNewImage(QImage image);
    void onNewProcessedImage(QImage image);
    void onGeneratorErrorOccurred(const QString &error);
    void onProcessorErrorOccurred(const QString &error);
    void onCameraConnected(const QString &serialNumber);
    void onCameraDisconnected(const QString &serialNumber);

public:
    RSCameraPrivate(RSCamera *camera);
    ~RSCameraPrivate();

signals:
    void started();
    void recording(float fx, float fy);

};

#endif // RSCAMERA_P_H
