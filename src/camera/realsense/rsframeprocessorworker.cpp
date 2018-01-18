#include "rsframeprocessorworker.h"
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/point_types.h>
#include <pcl/features/narf.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <QDebug>
#include <QImage>
#include <QThread>


void getColorForFloat (float value,
                       uchar& r, uchar& g, uchar& b)
{
    if (pcl_isinf (value))
    {
        if (value > 0.0f)
        {
            r = 150;  g = 150;  b = 200;  // INFINITY
            return;
        }
        r = 150;  g = 200;  b = 150;  // -INFINITY
        return;
    }
    if (!pcl_isfinite (value))
    {
        r = 200;  g = 150;  b = 150;  // -INFINITY
        return;
    }

    r = g = b = 0;
    value *= 10;
    if (value <= 1.0)
    {  // black -> purple
        b = static_cast<uchar> (pcl_lrint(value*200));
        r = static_cast<uchar> (pcl_lrint(value*120));
    }
    else if (value <= 2.0)
    {  // purple -> blue
        b = static_cast<uchar> (200 + pcl_lrint((value-1.0)*55));
        r = static_cast<uchar> (120 - pcl_lrint((value-1.0)*120));
    }
    else if (value <= 3.0)
    {  // blue -> turquoise
        b = static_cast<uchar> (255 - pcl_lrint((value-2.0)*55));
        g = static_cast<uchar> (pcl_lrint((value-2.0)*200));
    }
    else if (value <= 4.0)
    {  // turquoise -> green
        b = static_cast<uchar> (200 - pcl_lrint((value-3.0)*200));
        g = static_cast<uchar> (200 + pcl_lrint((value-3.0)*55));
    }
    else if (value <= 5.0)
    {  // green -> greyish green
        g = static_cast<uchar> (255 - pcl_lrint((value-4.0)*100));
        r = static_cast<uchar> (pcl_lrint((value-4.0)*120));
    }
    else if (value <= 6.0)
    { // greyish green -> red
        r = static_cast<uchar> (100 + pcl_lrint((value-5.0)*155));
        g = static_cast<uchar> (120 - pcl_lrint((value-5.0)*120));
        b = static_cast<uchar> (120 - pcl_lrint((value-5.0)*120));
    }
    else if (value <= 7.0)
    {  // red -> yellow
        r = 255;
        g = static_cast<uchar> (pcl_lrint((value-6.0)*255));
    }
    else
    {  // yellow -> white
        r = 255;
        g = 255;
        b = static_cast<uchar> (pcl_lrint((value-7.0)*255.0/3.0));
    }
}

QImage rangeImageToQImage(
        const pcl::RangeImage &ri,
        float min_value = -std::numeric_limits<float>::infinity (),
        float max_value =  std::numeric_limits<float>::infinity (),
        bool grayscale = false)
{
    int size = ri.width * ri.height;
    int arraySize = 3 * size;
    float* floatImage = ri.getRangesArray();

    uchar *data = new uchar[arraySize];
    uchar *dataPtr = data;

    bool recalculateMinValue = pcl_isinf (min_value),
            recalculateMaxValue = pcl_isinf (max_value);
    if (recalculateMinValue) min_value = std::numeric_limits<float>::infinity ();
    if (recalculateMaxValue) max_value = -std::numeric_limits<float>::infinity ();

    if (recalculateMinValue || recalculateMaxValue)
    {
        for (int i = 0; i < size; ++i)
        {
            float value = floatImage[i];
            if (!pcl_isfinite(value)) continue;
            if (recalculateMinValue)  min_value = (std::min)(min_value, value);
            if (recalculateMaxValue)  max_value = (std::max)(max_value, value);
        }
    }

    float factor = 1.0f / (max_value - min_value), offset = -min_value;

    for (int i = 0; i < size; ++i)
    {
        uchar& r= *(dataPtr++), &g = *(dataPtr++), &b = *(dataPtr++);
        float value = floatImage[i];

        if (!pcl_isfinite(value))
        {
            getColorForFloat(value, r, g, b);
            continue;
        }

        // Normalize value to [0, 1]
        value = std::max (0.0f, std::min (1.0f, factor * (value + offset)));

        // Get a color from the value in [0, 1]
        if (grayscale)
        {
            r = g = b = static_cast<uchar> (pcl_lrint (value * 255));
        }
        else
        {
            getColorForFloat(value, r, g, b);
        }
        //cout << "Setting pixel "<<i<<" to "<<(int)r<<", "<<(int)g<<", "<<(int)b<<".\n";
    }

    QImage qimage = QImage((const uchar *) data, ri.width, ri.height, QImage::Format_RGB888).copy();
    delete[] data;
    return qimage;
}


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
    int nrThreads = QThread::idealThreadCount() - 1;
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
            qDebug() << "before: " << pclPoints->width << "x" << pclPoints->height << "after: " << cloudFiltered->width << "x" << cloudFiltered->height;
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

            pcl::RangeImagePlanar rangeImage;
            float riWidth = static_cast<float>(depth.get_width());
            float riHeight = static_cast<float>(depth.get_height());
            rangeImage.createFromPointCloudWithFixedSize(*cloudFiltered, riWidth, riHeight,
                                                         riWidth * 0.5f, riHeight * 0.5f, fx, fy,
                                                         sensorPose, pcl::RangeImage::CAMERA_FRAME, noiseLevel,
                                                         minimumRange);
            // Does this speed up the calculations?
            // rangeImage.cropImage();
//            pcl::RangeImage rangeImage;
//            rangeImage.createFromPointCloud(*cloudFiltered, pcl::deg2rad (0.5f), pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
//                                            sensorPose, pcl::RangeImage::CAMERA_FRAME, 0.0, 0.0f, 1);
            emit newImage(rangeImageToQImage(rangeImage));
            rangeImage.setUnseenToMaxRange();
            qDebug()<<"Range Image size: "<<rangeImage.size();
            pcl::RangeImageBorderExtractor extractor;
            pcl::NarfKeypoint narfKeyPointDetector;
            narfKeyPointDetector.setRangeImageBorderExtractor(&extractor);
            narfKeyPointDetector.setRangeImage(&rangeImage);
            narfKeyPointDetector.getParameters().support_size = 0.035f; //Defines the area 'covered' by an interest point (in meters)
            narfKeyPointDetector.getParameters().calculate_sparse_interest_image = false;
            narfKeyPointDetector.getParameters().max_no_of_threads = nrThreads;
//            narfKeyPointDetector.getParameters().use_recursive_scale_reduction = true;
            narfKeyPointDetector.setRadiusSearch(0.01f);
            pcl::PointCloud<int> keypointIndices;
            narfKeyPointDetector.compute(keypointIndices);
            qDebug()<<"Detected "<<keypointIndices.points.size()<<"key points";

//            // Open a Visualizer to show keypoints
//            pcl::visualization::RangeImageVisualizer viewer("NARF keypoints");
//            viewer.showRangeImage(rangeImage);
//            for (size_t i = 0; i < keypointIndices.points.size(); ++i)
//            {
//                viewer.markPoint(keypointIndices.points[i] % rangeImage.width,
//                                 keypointIndices.points[i] / rangeImage.width,
//                                 // Set the color of the pixel to red (the background
//                                 // circle is already that color). All other parameters
//                                 // are left untouched, check the API for more options.
//                                 pcl::visualization::Vector3ub(1.0f, 0.0f, 0.0f));
//            }
//            while (!viewer.wasStopped())
//            {
//                viewer.spinOnce();
//                // Sleep 100ms to go easy on the CPU.
//                pcl_sleep(0.1);
//            }

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


