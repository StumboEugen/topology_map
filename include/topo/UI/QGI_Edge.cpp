//
// Created by stumbo on 18-10-16.
//

#include "QGI_Edge.h"
#include "QGI_Node.h"

#include "topo/Topo.h"

#include <QPainter>

QGI_Edge::QGI_Edge(TopoEdge *edge, QGI_Node *QNodeA, QGI_Node *QNodeB)
        :relatedEdgeTOPO(edge)
        ,QNodeA(QNodeA)
        ,QNodeB(QNodeB)
{
    if (edge->getExitA() == QNodeA->getRelatedNodeTOPO()) {
        this->QNodeA = QNodeA;
        this->QNodeB = QNodeB;
    }
    else if (edge->getExitB() == QNodeA->getRelatedNodeTOPO()) {
        this->QNodeB = QNodeA;
        this->QNodeA = QNodeB;
    }
    this->setZValue(1);
}

void
QGI_Edge::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->drawLine(QNodeA->pos(), QNodeB->pos());
}

QRectF QGI_Edge::boundingRect() const {
    //TODO http://doc.qt.io/archives/qt-4.8/qgraphicsitem.html#boundingRect
    //may need to call call prepareGeometryChange()
    auto posA = QNodeA->pos();
    auto posB = QNodeB->pos();
    auto right = qMax(posA.x(), posB.x());
    auto left = qMin(posA.x(), posB.x());
    auto down = qMax(posA.y(), posB.y());
    auto up = qMin(posA.y(), posB.y());
    return QRectF{left, up, (right - left), (up - down)};
}
