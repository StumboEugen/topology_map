//
// Created by stumbo on 18-10-16.
//

#include "QGI_Node.h"

#include <QPainter>
#include <QColor>
#include <QDebug>
#include <QStyleOptionGraphicsItem>

#include "UITOOLS.h"

void
QGI_Node::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    static QRect rect{-RECT_SIZE_HALF, -RECT_SIZE_HALF,
                      RECT_SIZE_HALF * 2, RECT_SIZE_HALF * 2};
    if (drawDetail) {
//        painter->drawArc(rect, 0, 16 * 360);
        const auto & ins = relatedNodeTOPO->getInsCorrespond();
        for (const auto & exit: ins->getExits()) {
            QPointF exitGatePoint{exit.getPosX() * METER_TO_PIXLE,
                                  -exit.getPosY() * METER_TO_PIXLE};
            painter->drawLine({0,0}, exitGatePoint);
            QPointF gateDir{sin(exit.getOutDir() * DEG2RAD), -cos(exit.getOutDir() * DEG2RAD)};
            gateDir *= METER_TO_PIXLE * 0.5;
            painter->save();
            painter->setPen(Qt::red);
            painter->drawLine(exitGatePoint, gateDir + exitGatePoint);
            painter->restore();
        }
    } else {
        if (option->state & QStyle::State_Selected) {
            painter->setPen(Qt::red);
            painter->drawRect(rect.adjusted(-1, -1, 0, 0));
        }
        painter->fillRect(rect, Qt::gray);
    }
}

QRectF QGI_Node::boundingRect() const {
    static QRect rect{-RECT_SIZE_HALF - 1, -RECT_SIZE_HALF - 1,
                      RECT_SIZE_HALF * 2 + 2, RECT_SIZE_HALF * 2 + 2};
    if (drawDetail) {
        return {-100, -100, 200, 200};
    } else {
        return rect;
    }
}

void QGI_Node::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    QGraphicsItem::mousePressEvent(event);
}

QGI_Node::QGI_Node(TopoNode * node)
        :relatedNodeTOPO(node)
{
    this->setZValue(2);
    setFlag(ItemIsSelectable);
}

int QGI_Node::type() const {
    return QGI_NODE_TYPE;
}
