//
// Created by stumbo on 19-3-5.
//

#include "QGI_Robot.h"

#include <QPainter>

void
QGI_Robot::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->setBrush(Qt::red);
    painter->drawEllipse(-10, -10, 20, 20);
}

QRectF QGI_Robot::boundingRect() const {
    return {-10, -10, 20, 20};
}

QPainterPath QGI_Robot::shape() const {
    return QGraphicsItem::shape();
}

QGI_Robot::QGI_Robot()
{

}
