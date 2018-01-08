#ifndef RSFRAMEGENERATORWORKER_H
#define RSFRAMEGENERATORWORKER_H

#include <librealsense2/rs.hpp>

#include <QObject>
#include <QString>
#include <QMutex>

class RSFrameGeneratorWorker : public QObject
{
    Q_OBJECT
public:
    explicit RSFrameGeneratorWorker(rs2::pipeline *pipe, rs2::frame_queue *queue);
    void stop();

private:
    bool m_stopped;
    QMutex m_mutex;
    rs2::pipeline *m_pipe;
    rs2::frame_queue *m_queue;

public slots:
    void doWork();

signals:
    void stopped();
    void errorOccurred(const QString &error);
};
#endif // RSFRAMEGENERATORWORKER_H
