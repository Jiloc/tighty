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
    rs2::colorizer color_map;
    m_stopped = false;
    try
    {
        while(true)
        {
            rs2::frameset frames = m_pipe->wait_for_frames(); // Wait for next set of frames from the camera
            m_queue->enqueue(frames);

            emit newImage(frameToQImage(color_map(frames.get_depth_frame())));
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

QImage RSFrameGeneratorWorker::frameToQImage(const rs2::frame& f)
{
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        return QImage((const uchar *) f.get_data(), w, h, QImage::Format_RGB888).copy();
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return QImage((const uchar *) f.get_data(), w, h, QImage::Format_Grayscale8).copy();
    }

    throw std::runtime_error("Frame format is not supported yet!");
    /*if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        qDataType = QImage::Format_RGB888;
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    */
}
