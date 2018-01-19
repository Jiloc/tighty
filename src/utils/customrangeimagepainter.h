#ifndef CUSTOMQRANGEIMAGE_H
#define CUSTOMQRANGEIMAGE_H

#include <QImage>

// Forward declaration
namespace pcl {

template<typename T>
class PointCloud;

class RangeImage;

}
//
using KeyPointIndices = pcl::PointCloud<int>;

class CustomRangeImagePainter : public QImage
{
public:
    CustomRangeImagePainter();
    CustomRangeImagePainter(const uchar* data, int width, int height, Format format);
    CustomRangeImagePainter(const QImage& other);
    const int getPixelSize() const { return m_pixelSize; }
    void setPixelSize(int newSize);
    bool isPointInImage(int x, int y) {return (x>=0 && x<width()) && (y>=0 && y <height());}
    void markPoints(const KeyPointIndices& points,pcl::RangeImage& rangeImage, const QColor& color);

private:
    int m_pixelSize;
};

#endif // CUSTOMQRANGEIMAGE_H
