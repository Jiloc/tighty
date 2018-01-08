#include "rsframegeneratorworker.h"
#include <QDebug>

RSFrameGeneratorWorker::RSFrameGeneratorWorker(rs2::pipeline *pipe, rs2::frame_queue *queue):
    m_pipe(pipe),
    m_queue(queue),
    m_mutex()
{
}

void RSFrameGeneratorWorker::stop()
{
    QMutexLocker locker(&m_mutex);
    m_stopped = true;
}

void RSFrameGeneratorWorker::doWork()
{
    m_stopped = false;
    try
    {
        while(true)
        {
            rs2::frameset frames = m_pipe->wait_for_frames(); // Wait for next set of frames from the camera
            m_queue->enqueue(frames.get_depth_frame());
            {
                QMutexLocker locker(&m_mutex);
                if (m_stopped)
                {
                    emit stopped();
                    break;
                }
            }// locker goes out of scope and releases the mutex
        }
    }
    catch (const rs2::error & e)
    {
        emit errorOccurred(e.what());
    }
    catch (const std::exception& e)
    {
        emit errorOccurred(e.what());
    }
}
