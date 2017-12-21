#include "rscamera_p.h"

#include <QDebug>

const std::string NO_CAMERA_MESSAGE = "No camera connected, please connect 1 or more";

RSCameraPrivate::RSCameraPrivate(RSCamera *camera):
    q_ptr(camera),
    m_worker(&m_pipe)
{
    connect(&m_cameraManager, &RSCameraManager::cameraConnected,
            this, &RSCameraPrivate::onCameraConnected);
    connect(&m_cameraManager, &RSCameraManager::cameraDisconnected,
            this, &RSCameraPrivate::onCameraDisconnected);
    m_cameraManager.setup();

    m_worker.moveToThread(&m_workerThread);

    connect(this, &RSCameraPrivate::started,
            &m_worker, &RSImageGeneratorWorker::doWork);

    connect(&m_worker, &RSImageGeneratorWorker::newImage,
            this, &RSCameraPrivate::onNewImage,
            Qt::ConnectionType::QueuedConnection);

    connect(&m_worker, &RSImageGeneratorWorker::stopped,
            this, &RSCameraPrivate::_stop);

    connect(&m_worker, &RSImageGeneratorWorker::errorOccurred,
            this, &RSCameraPrivate::onErrorOccurred);

    m_workerThread.start();
}


RSCameraPrivate::~RSCameraPrivate()
{
    m_workerThread.quit();
    m_workerThread.wait();
}

void RSCameraPrivate::start()
{
    Q_Q(RSCamera);
    q->setIsScanning(true);
    m_pipe.start(m_config);
    emit started();
}

void RSCameraPrivate::stop()
{
    m_worker.stop();
}

void RSCameraPrivate::_stop()
{
    qDebug() << "stop pipe";
    Q_Q(RSCamera);
    m_pipe.stop();
    q->setIsScanning(false);
}

void RSCameraPrivate::onNewImage(QImage image)
{
    Q_Q(RSCamera);
    emit q->newImage(image);
}

void RSCameraPrivate::onCameraConnected(const QString &serialNumber)
{
    qDebug() << "Connected: " << serialNumber;
    Q_Q(RSCamera);
    if (serialNumber == q->m_serialNumber)
    {
        m_config.enable_stream(RS2_STREAM_DEPTH);
        m_config.enable_device(serialNumber.toStdString());
        q->setIsConnected(true);
    }
}

void RSCameraPrivate::onCameraDisconnected(const QString &serialNumber)
{
    qDebug() << "Disconnected: " << serialNumber;
    Q_Q(RSCamera);
    if (serialNumber == q->m_serialNumber)
    {
        if (q->getIsScanning())
        {
            stop();
        }

        q->setIsConnected(false);
    }
}

void RSCameraPrivate::onErrorOccurred(const QString &error)
{
    qDebug() << error;
    m_config.disable_all_streams();
    m_pipe.stop();
    Q_Q(RSCamera);
    q->setIsScanning(false);
    emit q->errorOccurred(error);
}

RSCamera::RSCamera(const QString& serialNumber):
    DepthCamera(serialNumber),
    d_ptr(new RSCameraPrivate(this))
{
}

RSCamera::~RSCamera()
{
    stop();
}

void RSCamera::start()
{
    if (!getIsScanning())
    {
        Q_D(RSCamera);
        d->start();
    }
}

void RSCamera::stop()
{
    if (getIsScanning())
    {
        Q_D(RSCamera);
        d->stop();
    }
}


#include "moc_rscamera.cpp"
