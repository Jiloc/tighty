#ifndef DEPTHCAMERA_H
#define DEPTHCAMERA_H

#include <QString>
#include <QImage>

class DepthCamera: public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool isConnected READ getIsConnected NOTIFY isConnectedChanged)
    Q_PROPERTY(bool isStreaming READ getIsStreaming NOTIFY isStreamingChanged)
    Q_PROPERTY(bool isScanning READ getIsScanning NOTIFY isScanningChanged)

public:
    DepthCamera(const QString& serialNumber);
    const bool getIsConnected() const;
    const bool getIsStreaming() const;
    const bool getIsScanning() const;

public slots:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void record() = 0;

signals:
    void newImage(QImage image);
    void newProcessedImage(QImage image);
    void isConnectedChanged(const bool isConnected);
    void isStreamingChanged(const bool isStreaming);
    void isScanningChanged(const bool isScanning);
    void errorOccurred(const QString &error);

protected:
    QString m_serialNumber;
    void setIsConnected(const bool isConnected);
    void setIsStreaming(const bool isStreaming);
    void setIsScanning(const bool isScanning);

private:
    bool m_isConnected;
    bool m_isStreaming;
    bool m_isScanning;

};

#endif // DEPTHCAMERA_H
