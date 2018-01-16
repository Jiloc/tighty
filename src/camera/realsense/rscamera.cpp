#include "rscamera_p.h"
#include <QFileInfo>
#include <QDebug>

const std::string NO_CAMERA_MESSAGE = "No camera connected, please connect 1 or more";

const unsigned int FRAME_QUEUE_SIZE = 5;

RSCameraPrivate::RSCameraPrivate(RSCamera *camera):
    q_ptr(camera),
    m_generator(&m_pipe, &m_queue),
    m_processor(&m_pipe, &m_queue)
{
    connect(&m_cameraManager, &RSCameraManager::cameraConnected,
            this, &RSCameraPrivate::onCameraConnected);
    connect(&m_cameraManager, &RSCameraManager::cameraDisconnected,
            this, &RSCameraPrivate::onCameraDisconnected);
    m_cameraManager.setup();

    m_generator.moveToThread(&m_generatorThread);
    m_processor.moveToThread(&m_processorThread);

    connect(this, &RSCameraPrivate::started,
            &m_generator, &RSFrameGeneratorWorker::doWork);
    connect(this, &RSCameraPrivate::recording,
            &m_processor, &RSFrameProcessorWorker::doWork);

    connect(&m_generator, &RSFrameGeneratorWorker::newImage,
            this, &RSCameraPrivate::onNewImage,
            Qt::ConnectionType::QueuedConnection);

    connect(&m_generator, &RSFrameGeneratorWorker::stopped,
            this, &RSCameraPrivate::_stop);

    connect(&m_generator, &RSFrameGeneratorWorker::errorOccurred,
            this, &RSCameraPrivate::onErrorOccurred);
    connect(&m_processor, &RSFrameProcessorWorker::errorOccurred,
            this, &RSCameraPrivate::onErrorOccurred);

    m_generatorThread.start();
    m_processorThread.start();
}


RSCameraPrivate::~RSCameraPrivate()
{
    m_generatorThread.quit();
    m_generatorThread.wait();
    m_processorThread.quit();
    m_processorThread.wait();
}

void RSCameraPrivate::start()
{
    Q_Q(RSCamera);
    q->setIsStreaming(true);

    rs2::pipeline_profile profile = m_pipe.start(m_config);
    m_scanningDeviceSerial = profile.get_device().get_info(
                RS2_CAMERA_INFO_SERIAL_NUMBER);
    emit started();
}

void RSCameraPrivate::stop()
{
    m_generator.stop();
    m_processor.stop();
}

void RSCameraPrivate::_stop()
{
    qDebug() << "stop pipe";
    m_pipe.stop();
    qDebug() << "after pipe stop";
    Q_Q(RSCamera);
    q->setIsScanning(false);
    q->setIsStreaming(false);
}

void RSCameraPrivate::record()
{
    Q_Q(RSCamera);
    q->setIsScanning(true);
    m_generator.record();
    emit recording();
}

void RSCameraPrivate::playback(const QString &filename)
{
    m_config.enable_device_from_file(filename.toStdString());
    Q_Q(RSCamera);
    q->setIsConnected(true);
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
    if (q->m_serialNumber == DEFAULT_DEVICE)
    {
        q->setIsConnected(true);
    }
    if (serialNumber == q->m_serialNumber)
    {
        // m_config.enable_stream(RS2_STREAM_DEPTH);
        m_config.enable_device(serialNumber.toStdString());
        q->setIsConnected(true);
    }
}

void RSCameraPrivate::onCameraDisconnected(const QString &serialNumber)
{
    qDebug() << "Disconnected: " << serialNumber;
    Q_Q(RSCamera);
    if (q->m_serialNumber == DEFAULT_DEVICE && serialNumber == m_scanningDeviceSerial)
    {
        if (q->getIsStreaming())
        {
            stop();
        }
        if (m_cameraManager.getConnectedDevicesSize() <= 0)
        {
            q->setIsConnected(false);
        }
    }

    else if (serialNumber == q->m_serialNumber)
    {
        if (q->getIsStreaming())
        {
            stop();
        }

        q->setIsConnected(false);
    }
}

void RSCameraPrivate::onErrorOccurred(const QString &error)
{
    qDebug() << error;
    Q_Q(RSCamera);
    stop();
    q->setIsStreaming(false);
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
    if (!getIsStreaming())
    {
        Q_D(RSCamera);
        d->start();
    }
}

void RSCamera::stop()
{
    if (getIsStreaming())
    {
        Q_D(RSCamera);
        d->stop();
    }
}

void RSCamera::record()
{
    if (getIsStreaming() && ! getIsScanning())
    {
        Q_D(RSCamera);
        d->record();
    }
}

void RSCamera::playback(const QString &filename)
{
    QFileInfo fi(filename);
    Q_D(RSCamera);
    d->playback(fi.absoluteFilePath());
}

void RSCamera::playback(const QUrl& url)
{
    playback(url.toLocalFile());
}

#include "moc_rscamera.cpp"
