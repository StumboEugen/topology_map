//
// Created by stumbo on 18-10-16.
//

#include "QGI_Node.h"

#include <QPainter>
#include <QColor>

void
QGI_Node::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    QRect rect{-RECT_SIZE_HALF, -RECT_SIZE_HALF, RECT_SIZE_HALF * 2, RECT_SIZE_HALF * 2};
//    painter->drawRect(rect);
    painter->fillRect(rect, QColor(200, 200, 200));
}

QRectF QGI_Node::boundingRect() const {
    return {-RECT_SIZE_HALF - 1, -RECT_SIZE_HALF - 1,
            RECT_SIZE_HALF * 2 + 2, RECT_SIZE_HALF * 2 + 2};
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
