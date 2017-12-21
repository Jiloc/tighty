#include "rsimagegeneratorworker.h"
#include <QDebug>

RSImageGeneratorWorker::RSImageGeneratorWorker(rs2::pipeline *pipe):
    m_pipe(pipe),
    m_mutex()
{
}

void RSImageGeneratorWorker::stop()
{
    QMutexLocker locker(&m_mutex);
    m_stopped = true;
}

void RSImageGeneratorWorker::doWork()
{
    rs2::colorizer color_map;
    m_stopped = false;
    try
    {
        while(true)
        {
            rs2::frameset data = m_pipe->wait_for_frames(); // Wait for next set of frames from the camera
            rs2::frame depth = color_map(data.get_depth_frame()); // Find and colorize the depth data
            emit newImage(frameToQImage(depth));

//            rs2::frameset frameset;
//            m_pipe->poll_for_frames(&frameset);
//            if (frameset.size() > 0)
//            {
//                //rs2::frameset data = m_pipe->wait_for_frames(); // Wait for next set of frames from the camera
//                // rs2::frame depth = data.get_depth_frame(); // Find and colorize the depth data
//                // rs2::depth_frame depth = data.get_depth_frame();
//                // rs2::frame frame = color_map(depth);
//                rs2::frame depth = color_map(frameset.get_depth_frame()); // Find and colorize the depth data
//                //rs2::frame color = data.get_color_frame();            // Find the color data
//                emit newImage(frameToQImage(depth));
//            }
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

QImage RSImageGeneratorWorker::frameToQImage(const rs2::frame& f)
{
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        return QImage((const uchar *) f.get_data(), w, h, QImage::Format_RGB888);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return QImage((const uchar *) f.get_data(), w, h, QImage::Format_Grayscale8);
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
