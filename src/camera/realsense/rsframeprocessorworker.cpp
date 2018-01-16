#include "rsframeprocessorworker.h"
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/point_types.h>
#include <pcl/features/narf.h>
#include <pcl/keypoints/narf_keypoint.h>
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
    m_stopped = false;
    try
    {
        while(true)
        {
            rs2::frame f = m_queue->wait_for_frame();
            rs2::frameset fs = f.as<rs2::frameset>();
            rs2::depth_frame depth = fs.get_depth_frame();
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
            Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloudFiltered->sensor_origin_[0],
                                                         cloudFiltered->sensor_origin_[1],
                    cloudFiltered->sensor_origin_[2])) * Eigen::Affine3f(cloudFiltered->sensor_orientation_);
            qDebug()<<"Calculating KeyPoints";
            qDebug()<<"*** Range image ***";
            pcl::RangeImage rangeImage;
            rangeImage.createFromPointCloud(*cloudFiltered, pcl::deg2rad (0.5f), pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                            sensorPose, pcl::RangeImage::CAMERA_FRAME, 0.0, 0.0f, 1);
            rangeImage.setUnseenToMaxRange();
            qDebug()<<"Range Image size: "<<rangeImage.size();
            pcl::RangeImageBorderExtractor extractor;
            pcl::NarfKeypoint narfKeyPointDetector;
            narfKeyPointDetector.setRangeImageBorderExtractor(&extractor);
            narfKeyPointDetector.setRangeImage(&rangeImage);
            narfKeyPointDetector.getParameters().support_size = 0.03f; //Defines the area 'covered' by an interest point (in meters)
            narfKeyPointDetector.setRadiusSearch(0.01f);
            pcl::PointCloud<int> keypointIndices;
            narfKeyPointDetector.compute(keypointIndices);
            qDebug()<<"Detected "<<keypointIndices.points.size()<<"key points";
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


