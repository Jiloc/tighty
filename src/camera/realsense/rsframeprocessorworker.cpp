#include "rsframeprocessorworker.h"
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/point_types.h>
#include <pcl/features/narf.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <QDebug>
//#include <QImage>
#include "utils/customrangeimagepainter.h"
#include <QThread>
#include <sstream>

using PCLPoint = pcl::PointCloud<pcl::PointXYZ>;

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

    //QImage qimage = QImage((const uchar *) data, ri.width, ri.height, QImage::Format_RGB888).copy();
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
    float minimumRange = 0.02f;
    int nrThreads = QThread::idealThreadCount() - 1;
    static int curFrame = 0;
    const int nrFrameShow = 9;
    // save last frame narf features
    pcl::PointCloud<pcl::Narf36>::Ptr lastFeatures(new pcl::PointCloud<pcl::Narf36>());
    // save last frame point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr lastPointCloud(new pcl::PointCloud<pcl::PointXYZ>());

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
            pass.setFilterLimits(0.2, 0.5);
            pass.filter(*cloudFiltered);

            /*float angularResX = static_cast<float>(71.5f / (depth.get_width() * (M_PI / 180.0f)));
            float angularResY = static_cast<float>(55.0f / (depth.get_height() * (M_PI / 180.0f)))*/;

            qDebug() << "before: " << pclPoints->points.size() << " after: " << cloudFiltered->points.size();
            qDebug() << "before: " << pclPoints->width << "x" << pclPoints->height << "after: " << cloudFiltered->width << "x" << cloudFiltered->height;
            curFrame++;
            if(curFrame >= nrFrameShow) {
                curFrame = 0;

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
//                rangeImage.cropImage();
                //            pcl::RangeImage rangeImage;
                //            rangeImage.createFromPointCloud(*cloudFiltered, pcl::deg2rad (0.5f), pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                //                                            sensorPose, pcl::RangeImage::CAMERA_FRAME, 0.0, 0.0f, 1);
                CustomRangeImagePainter viewRangeImage = rangeImageToQImage(rangeImage);
                rangeImage.setUnseenToMaxRange();
                qDebug()<<"Range Image size: "<<rangeImage.size();
                pcl::RangeImageBorderExtractor extractor;
                pcl::NarfKeypoint narfKeyPointDetector;
                narfKeyPointDetector.setRangeImageBorderExtractor(&extractor);
                narfKeyPointDetector.setRangeImage(&rangeImage);
                narfKeyPointDetector.getParameters().support_size = 0.035f; //Defines the area 'covered' by an interest point (in meters)
                //narfKeyPointDetector.getParameters().calculate_sparse_interest_image = false;
                narfKeyPointDetector.getParameters().max_no_of_threads = nrThreads;
                //            narfKeyPointDetector.getParameters().use_recursive_scale_reduction = true;
                narfKeyPointDetector.setRadiusSearch(0.01f);
                pcl::PointCloud<int> keypointIndices;
                keypointIndices.sensor_origin_ = rangeImage.sensor_origin_;
                keypointIndices.sensor_orientation_ = rangeImage.sensor_orientation_;
                narfKeyPointDetector.compute(keypointIndices);
                qDebug()<<"Detected "<<keypointIndices.points.size()<<"key points";
                QColor narfColor = QColor::fromRgb(255,0,0);
                const int markPointSize = 8; // markPoint size in pixels
//                for( unsigned int i=0; i < keypointIndices.points.size(); i++) {
//                    int curPoint = keypointIndices.points[i];
//                    int px = curPoint % rangeImage.width;
//                    int py = curPoint / rangeImage.width;
//                    qDebug()<<"setting pixel: "<<px<<", "<<py<<" of point "<<i<<", to color "<<narfColor;
//                    for(int j=py - markPointSize;j<py+markPointSize;j++) {
//                        for(int k=px-markPointSize;k<px+markPointSize;k++)
//                            viewRangeImage.setPixelColor(k,j,narfColor);
//                    }
//                }
                viewRangeImage.markPoints(keypointIndices,rangeImage,narfColor);



                pcl::NarfKeypoint narfKeyPointDetector2;
                narfKeyPointDetector2.setRangeImageBorderExtractor(&extractor);
                narfKeyPointDetector2.setRangeImage(&rangeImage);
                narfKeyPointDetector2.getParameters().support_size = 0.1f; //Defines the area 'covered' by an interest point (in meters)
                //narfKeyPointDetector.getParameters().calculate_sparse_interest_image = false;
                narfKeyPointDetector2.getParameters().max_no_of_threads = nrThreads;
                //            narfKeyPointDetector.getParameters().use_recursive_scale_reduction = true;
                narfKeyPointDetector2.setRadiusSearch(0.1f);

                pcl::PointCloud<int> keypointIndices2;
                keypointIndices2.sensor_origin_ = rangeImage.sensor_origin_;
                keypointIndices2.sensor_orientation_ = rangeImage.sensor_orientation_;
                narfKeyPointDetector2.compute(keypointIndices2);
                qDebug()<<"Detected2 "<<keypointIndices2.points.size()<<"key points";
                QColor narfColor2 = QColor::fromRgb(0,220,0);
                viewRangeImage.markPoints(keypointIndices2,rangeImage,narfColor2);

                emit newImage(viewRangeImage);

//                pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//                pcl::PointCloud<pcl::FPFHSignature33>::Ptr fphsDescriptors(new pcl::PointCloud<pcl::FPFHSignature33>);

//                pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normalEstimation;
//                normalEstimation.setInputCloud(cloudFiltered);
//                normalEstimation.setRadiusSearch(0.1f);

//                pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
//                normalEstimation.setSearchMethod(kdtree);
//                normalEstimation.compute(*normals);

//                    // FPFH estimation object.
//                pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
//                fpfh.setInputCloud(cloudFiltered);
//                fpfh.setInputNormals(normals);
//                fpfh.setSearchMethod(kdtree);
//                // Search radius, to look for neighbors. Note: the value given here has to be
//                // larger than the radius used to estimate the normals.
//                fpfh.setRadiusSearch(0.1);

//                fpfh.compute(*fphsDescriptors);


//                pcl::visualization::RangeImageVisualizer viewer("NARF keypoints");

//                viewer.showRangeImage(rangeImage);


//                for (size_t i = 0; i < keypointIndices.points.size(); ++i)
//                {

//                    viewer.markPoint(keypointIndices.points[i] % rangeImage.width,
//                                     -(keypointIndices.points[i] / rangeImage.width) + rangeImage.height,
//                                     // Set the color of the pixel to red (the background
//                                     // circle is already that color). All other parameters
//                                     // are left untouched, check the API for more options.
//                                     pcl::visualization::Vector3ub(1.0f, 0.0f, 0.0f));
//                }

//                for (size_t i = 0; i < keypointIndices2.points.size(); ++i)
//                {
//                    viewer.markPoint(keypointIndices2.points[i] % rangeImage.width,
//                                     -(keypointIndices2.points[i] / rangeImage.width) + rangeImage.height,
//                                     // Set the color of the pixel to red (the background
//                                     // circle is already that color). All other parameters
//                                     // are left untouched, check the API for more options.
//                                     pcl::visualization::Vector3ub(0.0f, 1.0f, 0.0f),
//                                     pcl::visualization::Vector3ub(0.0f, 1.0f, 0.0f));
//                }

//                while (!viewer.wasStopped())
//                {
//                    viewer.spinOnce();
//                    // Sleep 100ms to go easy on the CPU.
//                    pcl_sleep(0.1);
//                }
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
                //pcl::PointCloud<pcl::Narf36> narfDescriptors; //pcl::Narf36 -> struct representing Narf descriptor
                pcl::PointCloud<pcl::Narf36>::Ptr narfDescriptors(new pcl::PointCloud<pcl::Narf36>());
                narfDescriptor.compute(*narfDescriptors);
                qDebug()<<"extracted "<<narfDescriptors->size()<<" features for "<<keypointIndices.points.size()<<" keypoints";
                if(lastFeatures->size() > 0) {
//                    qDebug()<<"finding corrispondence with previous narf keypoints...";
//                    std::vector<int> corr_out;
//                    std::vector<float> corr_scores_out;
//                    corr_out.resize(narfDescriptors.size());
//                    corr_scores_out.resize(narfDescriptors.size());
//                    pcl::search::KdTree<pcl::Narf36> desc_kdtree;
//                    desc_kdtree.setInputCloud(lastFeatures);
//                    const int k = 3;
//                    std::vector<int> k_indices(k);
//                    std::vector<float> k_sq_dist(k);
//                    qDebug()<<"finding k nearest neighbours with k: "<<k;
//                    for(size_t i = 0; i<narfDescriptors.size();i++) {
//                        desc_kdtree.nearestKSearch(narfDescriptors,i,k,k_indices,k_sq_dist);
//                        corr_out[i] = k_indices[0];
//                        corr_scores_out[i] = k_sq_dist[0];
//                    }
//                    for(int i: corr_out) {
//                        qDebug()<<"\t** feature descriptor # "<<i<<", found "<<corr_out[i]<<" correspondences:";
////                        for(auto j:)
//                    }
                    pcl::registration::CorrespondenceEstimation<pcl::Narf36,pcl::Narf36> estimation;
                    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
                    estimation.setInputSource(narfDescriptors);
                    estimation.setInputTarget(lastFeatures);
                    estimation.determineCorrespondences(*correspondences);

                    qDebug()<<"\t** Correspondences found (before rejection): "<<correspondences->size();
                    // Duplication rejection
                    pcl::CorrespondencesPtr correspondencesAfterDupRej(new pcl::Correspondences());
                    pcl::registration::CorrespondenceRejectorOneToOne corrRejOneToOne;
                    corrRejOneToOne.setInputCorrespondences(correspondences);
                    corrRejOneToOne.getCorrespondences(*correspondencesAfterDupRej);

                    qDebug()<<"\t** Correspondences found (After Duplication rejection): "<<correspondencesAfterDupRej->size();

                    // Correspondeces rejection RANSAC
                    // pcl source -> target final transform, initialized with identity matrix
                    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
                    pcl::CorrespondencesPtr correspondecesFiltered(new pcl::Correspondences());
                    rejector.setInputSource(cloudFiltered);
                    rejector.setInputTarget(lastPointCloud);
                    rejector.setInlierThreshold(0.30); // distance in m, TODO see SampleConsensus parameters
                    rejector.setMaximumIterations(1000000);
                    rejector.setRefineModel(false);
                    rejector.setInputCorrespondences(correspondencesAfterDupRej);
                    rejector.getCorrespondences(*correspondecesFiltered);
                    transform = rejector.getBestTransformation();

                    qDebug()<<"\t Correspondences found after RANSAC: "<<correspondecesFiltered->size();
                    Eigen::IOFormat matrixFormat(4,0,", ","\n","[","]");
                    std::stringstream stream;
                    stream<<Eigen::WithFormat<Eigen::Matrix4f>(transform,matrixFormat);
                    //must convert from std::string (str() to char* c_str()) to properly display \n
                    qDebug()<<"\t Transform Matrix:\n"<<stream.str().c_str();
                    // TransformationEstimationSVD
                    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> svd;
                    svd.estimateRigidTransformation(*cloudFiltered,*lastPointCloud,*correspondecesFiltered,transform);
                    //svd.estimateRigidTransformation();
                    std::stringstream stream2;
                    stream2<<Eigen::WithFormat<Eigen::Matrix4f>(transform,matrixFormat);
                    qDebug()<<"\t Transform Matrix after TransformationEstimationSVD:\n"<<stream2.str().c_str();
                    lastFeatures->clear();
                }
                else {
                    qDebug()<<"\t*** no previous feature descriptors found";
                }
                lastPointCloud = cloudFiltered;
                pcl::copyPointCloud(*narfDescriptors,*lastFeatures);
            }
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


