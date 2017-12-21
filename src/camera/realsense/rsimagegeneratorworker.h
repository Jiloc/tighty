#ifndef RSIMAGEGENERATORWORKER_H
#define RSIMAGEGENERATORWORKER_H

#include <librealsense2/rs.hpp>

#include <QObject>
#include <QString>
#include <QImage>
#include <QMutex>


class RSImageGeneratorWorker : public QObject
{
    Q_OBJECT
public:
    explicit RSImageGeneratorWorker(rs2::pipeline *pipe);
    void stop();

private:
    bool m_stopped;
    bool m_paused;
    QMutex m_mutex;
    rs2::pipeline *m_pipe;

    QImage frameToQImage(const rs2::frame &f);

public slots:
    void doWork();

signals:
    void newImage(QImage image);
    void stopped();
    void errorOccurred(const QString &error);
};

#endif // RSIMAGEGENERATORWORKER_H
