#include "videoplayer.h"
#include <QDebug>

VideoPlayer::VideoPlayer() : QQuickPaintedItem() {}

void VideoPlayer::paint(QPainter *painter) {
    this->image = image.scaled(painter->device()->width(), painter->device()->height(), Qt::KeepAspectRatio);
    QRect rect(this->image.rect());
    QRect devRect(0, 0, painter->device()->width(), painter->device()->height());
    rect.moveCenter(devRect.center());
    painter->drawImage(rect.topLeft(), this->image);
}

void VideoPlayer::setImage(QImage image){
    this->image = image;
    this->update();
}

void VideoPlayer::reset() {
    setImage(QImage());
}
