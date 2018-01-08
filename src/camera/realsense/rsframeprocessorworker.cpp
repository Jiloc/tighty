#include "rsframeprocessorworker.h"
#include <QDebug>

RSFrameProcessorWorker::RSFrameProcessorWorker(rs2::pipeline *pipe, rs2::frame_queue *queue):
    m_mutex(),
    m_pipe(pipe),
    m_queue(queue)
{
}

void RSFrameProcessorWorker::stop()
{
    QMutexLocker locker(&m_mutex);
    m_stopped = true;
}

void RSFrameProcessorWorker::doWork()
{
    rs2::colorizer color_map;
    m_stopped = false;
    try
    {
        while(true)
        {
            rs2::frame depth = m_queue->wait_for_frame();
            emit newImage(frameToQImage(color_map(depth)));
            rs2::points points = m_pc.calculate(depth);
            auto pclPoints = pointsToPcl(points);

            pcl_ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(pclPoints);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.3, 0.5);
            pass.filter(*cloudFiltered);

            qDebug() << "before: " << pclPoints->points.size() << " after: " << cloudFiltered->points.size();
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
        qDebug() << e.what();
        emit errorOccurred(e.what());
    }
    catch (const std::exception& e)
    {
        qDebug() << e.what();
        emit errorOccurred(e.what());
    }
}

pcl_ptr RSFrameProcessorWorker::pointsToPcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (pcl::PointXYZ & p : cloud->points)
    {
        //memcpy(p.data, &(ptr->x), sizeof(float) * 3);
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

QImage RSFrameProcessorWorker::frameToQImage(const rs2::frame& f)
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
