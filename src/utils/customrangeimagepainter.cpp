#include "customrangeimagepainter.h"
#include <pcl/common/common.h>
#include <pcl/range_image/range_image.h>

#include <QDebug>

static const int DEFAULT_PX_SIZE = 6;

CustomRangeImagePainter::CustomRangeImagePainter() : QImage(), m_pixelSize{DEFAULT_PX_SIZE}
{

}

CustomRangeImagePainter::CustomRangeImagePainter(const uchar *data, int width, int height, Format format) :
    QImage(data,width,height,format), m_pixelSize{DEFAULT_PX_SIZE} {

}

CustomRangeImagePainter::CustomRangeImagePainter(const QImage& other) : QImage(other), m_pixelSize{DEFAULT_PX_SIZE}
{

}

void CustomRangeImagePainter::setPixelSize(int newSize) {
    if(newSize > 0)
        m_pixelSize = newSize;
}

void CustomRangeImagePainter::markPoints(const KeyPointIndices& points,pcl::RangeImage& rangeImage, const QColor &color) {
    if(points.size() <=0){
//#ifdef QT_DEBUG
        qDebug()<<"skipping markPoints because vector<int> points id <=0";
//#endif
        return;
    }

    if(rangeImage.width <= 0) {
//#ifdef QT_DEBUG
    qDebug()<<"cannot mark points because rangeImage has width <=0";
//#endif
        return;
    }
//#ifdef QT_DEBUG
    qDebug()<<"[CustomQRAngeImage] using pixelSize: "<<m_pixelSize;
//#endif
    for(int point: points) {
        int px = point % rangeImage.width;
        int py = point / rangeImage.width;

        if(!isPointInImage(px,py))
            return;
        for(int j=py - m_pixelSize;j<py+m_pixelSize;j++) {
            for(int k=px-m_pixelSize;k<px+m_pixelSize;k++){
                if(isPointInImage(k,j))
                 setPixelColor(k,j,color);
            }
        }
        setPixelColor(px,py,QColor(Qt::black));
    }
}
