#include "depthcamera.h"


DepthCamera::DepthCamera(const QString& serialNumber):
    m_serialNumber(serialNumber),
    m_isConnected(false),
    m_isScanning(false)
{
}

const bool DepthCamera::getIsScanning() const
{
    return m_isScanning;
}

void DepthCamera::setIsScanning(bool isScanning)
{
    if (isScanning != m_isScanning)
    {
        m_isScanning = isScanning;
        emit isScanningChanged(isScanning);
    }
}

const bool DepthCamera::getIsConnected() const
{
    return m_isConnected;
}

void DepthCamera::setIsConnected(bool isConnected)
{
    if (isConnected != m_isConnected)
    {
        m_isConnected = isConnected;
        emit isConnectedChanged(isConnected);
    }
}
