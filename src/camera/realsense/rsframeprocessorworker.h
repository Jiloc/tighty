#ifndef RSFRAMEPROCESSORWORKER_H
#define RSFRAMEPROCESSORWORKER_H

#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <QObject>
#include <QString>
#include <QMutex>
#include <QImage>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

class RSFrameProcessorWorker : public QObject
{
    Q_OBJECT
public:
    explicit RSFrameProcessorWorker(rs2::pipeline *pipe, rs2::frame_queue *queue);
    void stop();

private:
    bool m_stopped;
    bool m_paused;
    QMutex m_mutex;
    rs2::pipeline *m_pipe;
    rs2::frame_queue *m_queue;
    rs2::pointcloud m_pc;

    pcl_ptr pointsToPcl(const rs2::points &points);

public slots:
    void doWork();

signals:
    void newImage(QImage image);
    void stopped();
    void errorOccurred(const QString &error);
};

#endif // RSFRAMEPROCESSORWORKER_H
