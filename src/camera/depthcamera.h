#ifndef DEPTHCAMERA_H
#define DEPTHCAMERA_H

#include "tighty_global.h"

#include <QString>
#include <QImage>

class TIGHTYSHARED_EXPORT DepthCamera: public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool isConnected READ getIsConnected NOTIFY isConnectedChanged)
    Q_PROPERTY(bool isStreaming READ getIsStreaming NOTIFY isStreamingChanged)
    Q_PROPERTY(bool isScanning READ getIsScanning NOTIFY isScanningChanged)
    // Q_PROPERTY(bool canScan READ getCanScan NOTIFY canScanChanged)

public:
    DepthCamera(const QString& serialNumber);
    const bool getIsConnected() const;
    const bool getIsStreaming() const;
    // const bool getCanScan() const;
    const bool getIsScanning() const;

public slots:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void record() = 0;
    // virtual bool reconstruct(const QString& filename) = 0;

signals:
    void newImage(QImage image);
    void isConnectedChanged(const bool isConnected);
    void isStreamingChanged(const bool isStreaming);
    void isScanningChanged(const bool isScanning);
    // void canScanChanged(const bool canScan);
    void errorOccurred(const QString &error);

protected:
    QString m_serialNumber;
    void setIsConnected(const bool isConnected);
    void setIsStreaming(const bool isStreaming);
    // void setCanScan(const bool canScan);
    void setIsScanning(const bool isScanning);

private:
    bool m_isConnected;
    bool m_isStreaming;
    // bool m_canScan;
    bool m_isScanning;

};

#endif // DEPTHCAMERA_H
