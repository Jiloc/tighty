#include "rsframeprocessorworker.h"
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/point_types.h>
#include <pcl/features/narf.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
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

void RSFrameProcessorWorker::doWork(float fx, float fy)
{
    m_stopped = false;
    float noiseLevel = 0.0f;
    float minimumRange = 0.035f;

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

            /*float angularResX = static_cast<float>(71.5f / (depth.get_width() * (M_PI / 180.0f)));
            float angularResY = static_cast<float>(55.0f / (depth.get_height() * (M_PI / 180.0f)))*/;

            qDebug() << "before: " << pclPoints->points.size() << " after: " << cloudFiltered->points.size();

            Eigen::Affine3f sensorPose = Eigen::Affine3f(
                        Eigen::Translation3f(cloudFiltered->sensor_origin_[0],
                        cloudFiltered->sensor_origin_[1],
                        cloudFiltered->sensor_origin_[2])) *
                    Eigen::Affine3f(cloudFiltered->sensor_orientation_);
//            Eigen::Affine3f sensorPose = Eigen::Affine3f(
//                        Eigen::Translation3f(pclPoints->sensor_origin_[0],
//                        pclPoints->sensor_origin_[1],
//                        pclPoints->sensor_origin_[2])) *
//                    Eigen::Affine3f(pclPoints->sensor_orientation_);
            qDebug()<<"Calculating KeyPoints";
            qDebug()<<"*** Range image ***";
            //pcl::RangeImage rangeImage;
            pcl::RangeImagePlanar rangeImage;
//            rangeImage.createFromPointCloud(*cloudFiltered, pcl::deg2rad (0.5f), pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
//                                            sensorPose, pcl::RangeImage::CAMERA_FRAME, 0.0, 0.0f, 1);
            rangeImage.createFromPointCloudWithFixedSize(*cloudFiltered, depth.get_width(), depth.get_height(),
                                                         static_cast<float>((float)depth.get_width() * 0.5f ),
                                                         static_cast<float>((float)depth.get_height() * 0.5f), fx, fy,
                                                         sensorPose, pcl::RangeImage::CAMERA_FRAME, noiseLevel,
                                                         minimumRange);
            rangeImage.setUnseenToMaxRange();
            qDebug()<<"Range Image size: "<<rangeImage.size();
            pcl::RangeImageBorderExtractor extractor;
            pcl::NarfKeypoint narfKeyPointDetector;
            narfKeyPointDetector.setRangeImageBorderExtractor(&extractor);
            narfKeyPointDetector.setRangeImage(&rangeImage);
            narfKeyPointDetector.getParameters().support_size = 0.035f; //Defines the area 'covered' by an interest point (in meters)
            narfKeyPointDetector.setRadiusSearch(0.01f);
            pcl::PointCloud<int> keypointIndices;
            narfKeyPointDetector.compute(keypointIndices);
            qDebug()<<"Detected "<<keypointIndices.points.size()<<"key points";
            // basic feature description estimation
            std::vector<int> feKeypointIndices;
            feKeypointIndices.resize(keypointIndices.size());
            size_t keypointIndicesSize = keypointIndices.size();
            for(unsigned int i=0;i<keypointIndicesSize;i++){
                feKeypointIndices[i] = keypointIndices[i];
            }
            //pcl::RangeImage::ConstPtr constRangeImagePtr(&rangeImage);
            pcl::NarfDescriptor narfDescriptor;
            //narfDescriptor.setRangeImage(constRangeImagePtr.get(),&feKeypointIndices);
            narfDescriptor.setRangeImage(&rangeImage,&feKeypointIndices);
            narfDescriptor.getParameters().support_size = 0.03f;
            narfDescriptor.getParameters().rotation_invariant = true;
            pcl::PointCloud<pcl::Narf36> narfDescriptors; //pcl::Narf36 -> struct representing Narf descriptor
            narfDescriptor.compute(narfDescriptors);
            qDebug()<<"extracted "<<narfDescriptors.size()<<" features for "<<keypointIndices.points.size()<<" keypoints";
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
    static const float nan = std::numeric_limits<float>::quiet_NaN();

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
        if (ptr->z == 0)
        {
            p.x = p.y = p.z = nan;
        }
        else
        {
            p.x = ptr->x;
            p.y = ptr->y;
            p.z = ptr->z;
        }
        ptr++;
    }
    return cloud;
}

