#include <librealsense2/h/rs_types.h>

#include "rscameramanager.h"

#include <QMutableHashIterator>
#include <QDebug>

const std::string PLATFORM_CAMERA_NAME = "Platform Camera";

RSCameraManager::RSCameraManager()
{
}

void RSCameraManager::setup()
{
    // Register callback for tracking which devices are currently connected
    m_ctx.set_devices_changed_callback([&](rs2::event_information& info)
    {
        removeDevices(info);
        try
        {
            for (auto&& dev : info.get_new_devices())
            {
                addDevice(dev);
            }
        }
        catch (const rs2::backend_error & e)
        {
            qDebug() << "rs2error: " << rs2_exception_type_to_string(e.get_type());
            qDebug() << QString::fromStdString(e.what());

        }
        catch (const std::exception& e)
        {
            qDebug() << QString::fromStdString(e.what());
        }
    });

    // Query the list of connected RealSense devices
    for (auto&& dev : m_ctx.query_devices())
    {
        addDevice(dev);
    }
}

void RSCameraManager::addDevice(rs2::device& dev)
{
    QString serialNumber(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    QMutexLocker locker(&m_mutex);
    if (m_connectedDevices.contains(serialNumber))
    {
        return; //already in
    }
    // Ignoring platform cameras (webcams, etc..)
    if (PLATFORM_CAMERA_NAME == dev.get_info(RS2_CAMERA_INFO_NAME))
    {
        return;
    }
    m_connectedDevices.insert(serialNumber, dev);
    emit cameraConnected(serialNumber);
}

void RSCameraManager::removeDevices(const rs2::event_information& info)
{
    QMutexLocker locker(&m_mutex);
    // Go over the list of devices and check if it was disconnected
    QMutableHashIterator <QString, rs2::device> i(m_connectedDevices);
    while (i.hasNext())
    {
        i.next();
        rs2::device dev = i.value();
        if (info.was_removed(dev))
        {
            QString serialNumber = i.key();
            i.remove();
            emit cameraDisconnected(serialNumber);
        }
    }
}
