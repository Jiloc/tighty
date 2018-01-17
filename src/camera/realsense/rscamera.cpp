#include "rscamera_p.h"

#include <limits>

#include <QFileInfo>
#include <QDebug>

const std::string NO_CAMERA_MESSAGE = "No camera connected, please connect 1 or more";

const unsigned int FRAME_QUEUE_SIZE = std::numeric_limits<unsigned int>::max();

RSCameraPrivate::RSCameraPrivate(RSCamera *camera):
    q_ptr(camera),
    m_queue(FRAME_QUEUE_SIZE),
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

    connect(&m_processor, &RSFrameProcessorWorker::newImage,
            this, &RSCameraPrivate::onNewProcessedImage,
            Qt::ConnectionType::QueuedConnection);

    connect(&m_generator, &RSFrameGeneratorWorker::stopped,
            this, &RSCameraPrivate::_stop);

    connect(&m_generator, &RSFrameGeneratorWorker::errorOccurred,
            this, &RSCameraPrivate::onGeneratorErrorOccurred);
    connect(&m_processor, &RSFrameProcessorWorker::errorOccurred,
            this, &RSCameraPrivate::onProcessorErrorOccurred);

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

    m_profile = m_pipe.start(m_config);
    m_scanningDeviceSerial = m_profile.get_device().get_info(
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
    rs2::video_stream_profile streamProfile = m_profile.get_stream(RS2_STREAM_DEPTH)
            .as<rs2::video_stream_profile>();
    rs2_intrinsics intrinsics = streamProfile.get_intrinsics();
    qDebug()<<"start recording with intrinsics: Model ->"<<intrinsics.model<<
              ", fx and fy: "<<intrinsics.fx<<", "<<intrinsics.fy;
    emit recording(intrinsics.fx, intrinsics.fy);
}

void RSCameraPrivate::playback(const QString &filename)
{
    m_isPlayback = true;
    m_config.enable_device_from_file(filename.toStdString());
    Q_Q(RSCamera);
    q->setIsConnected(true);
}

void RSCameraPrivate::onNewImage(QImage image)
{
    Q_Q(RSCamera);
    emit q->newImage(image);
}

void RSCameraPrivate::onNewProcessedImage(QImage image)
{
    Q_Q(RSCamera);
    emit q->newProcessedImage(image);
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

void RSCameraPrivate::onGeneratorErrorOccurred(const QString &error)
{
    if (m_isPlayback)
    {
        rs2::playback pd = static_cast<rs2::playback>(m_profile.get_device());
        if (pd.current_status() == RS2_PLAYBACK_STATUS_STOPPED)
        {
            return;
        }
    }
    qDebug() << error;
    Q_Q(RSCamera);
    stop();
    q->setIsStreaming(false);
    q->setIsScanning(false);
    emit q->errorOccurred(error);
}

void RSCameraPrivate::onProcessorErrorOccurred(const QString &error)
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
