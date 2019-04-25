//
// Created by stumbo on 19-3-5.
//

#include "QRobot.h"
#include "QNode.h"
#include "QEdge.h"
#include "UITOOLS.h"

#include <QPainter>

void
QRobot::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    move2(currentAt);
    painter->setBrush(Qt::red);
    painter->drawEllipse(-QNODE_CIRCLE_SIZE / 4, -QNODE_CIRCLE_SIZE / 4,
                          QNODE_CIRCLE_SIZE / 2, QNODE_CIRCLE_SIZE / 2);
}

QRectF QRobot::boundingRect() const {
    return {-10, -10, 20, 20};
}

QPainterPath QRobot::shape() const {
    return QGraphicsItem::shape();
}

QRobot::QRobot(QGraphicsItem * currentAt)
        : currentAt(currentAt)
{
    setZValue(3);
    move2(currentAt);
}

void QRobot::move2(QGraphicsItem *target) {
    if (auto nodeItem = dynamic_cast<QNode *> (target)) {
        setPos(nodeItem->pos());
        currentAt = target;
    }
    else if (auto edgeItem = dynamic_cast<QEdge *> (target)) {
        const auto & line = edgeItem->line();
        setPos((line.p1() + line.p2()) / 2);
        currentAt = target;
    } else {
        currentAt = nullptr;
        cerr << "robot move to a WRONG place!";
    }
}

QGraphicsItem *QRobot::getCurrentAt() const {
    return currentAt;
}
