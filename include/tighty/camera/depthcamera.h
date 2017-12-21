#ifndef DEPTHCAMERA_H
#define DEPTHCAMERA_H

#include <QString>
#include <QImage>

class DepthCamera: public QObject
{
    Q_OBJECT

    Q_PROPERTY(bool isScanning READ getIsScanning NOTIFY isScanningChanged)
    Q_PROPERTY(bool isConnected READ getIsConnected NOTIFY isConnectedChanged)

public:
    DepthCamera(const QString& serialNumber);
    const bool getIsScanning() const;
    const bool getIsConnected() const;

public slots:
    virtual void start() = 0;
    virtual void stop() = 0;

signals:
    void newImage(QImage image);
    void isScanningChanged(const bool isScanning);
    void isConnectedChanged(const bool isConnected);
    void errorOccurred(const QString &error);
//    void canScanChanged(bool canScan);

private slots:
//    void setIsConnected(bool value);
//    void setCanScan(bool value);

protected:
    QString m_serialNumber;
    void setIsScanning(const bool value);
    void setIsConnected(const bool value);

private:
    bool m_isScanning;
    bool m_isConnected;

};

#endif // DEPTHCAMERA_H
