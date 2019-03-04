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
    // this means that constructor is called by the build mode
    if (edge != nullptr) {
        if (edge->getNodeA() == QNodeA->getRelatedNodeTOPO()) {
            setNodeA(QNodeA, edge->getGateA());
            setNodeB(QNodeB, edge->getGateB());
        }
        else if (edge->getNodeB() == QNodeA->getRelatedNodeTOPO()) {
            setNodeA(QNodeB, edge->getGateA());
            setNodeB(QNodeA, edge->getGateB());
        }
    }

    this->setZValue(1);
}

QRectF QGI_Edge::boundingRect() const {
    //TODO http://doc.qt.io/archives/qt-4.8/qgraphicsitem.html#boundingRect
    //may need to call call prepareGeometryChange()
//    auto posA = QNodeA->pos();
//    auto posB = QNodeB->pos();
//    auto right = qMax(posA.x(), posB.x());
//    auto left = qMin(posA.x(), posB.x());
//    auto down = qMax(posA.y(), posB.y());
//    auto up = qMin(posA.y(), posB.y());
//    return QRectF{left - 0.5, up - 0.5, (right - left) + 1, (down - up) + 1};
    return QGraphicsLineItem::boundingRect();
}

void QGI_Edge::setRelatedEdgeTOPO(TopoEdge *relatedEdgeTOPO) {
    QGI_Edge::relatedEdgeTOPO = relatedEdgeTOPO;
}

void QGI_Edge::setNodeA(QGI_Node *QNodeA, uint8_t gateA) {
    auto edgeLine = line();
    edgeLine.setP1(QNodeA->posOfExitInScene(gateA));
    setLine(edgeLine);
    QGI_Edge::QNodeA = QNodeA;
    QGI_Edge::gateA = gateA;
    this->setZValue(1);
}

void QGI_Edge::setNodeB(QGI_Node *QNodeB, uint8_t gateB) {
    auto edgeLine = line();
    edgeLine.setP1(QNodeB->posOfExitInScene(gateB));
    setLine(edgeLine);
    QGI_Edge::QNodeB = QNodeB;
    QGI_Edge::gateB = gateB;
    this->setZValue(1);
}

void
QGI_Edge::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    if (QNodeA != nullptr && QNodeB != nullptr) {
        const QPointF & pointOfExitA = QNodeA->posOfExitInScene(gateA);
        const QPointF & pointOfExitB = QNodeB->posOfExitInScene(gateB);
        setLine({pointOfExitA, pointOfExitB});
    }
    QGraphicsLineItem::paint(painter, option, widget);
}

QGI_Edge::QGI_Edge(QGI_Node *QNodeA, uint8_t gate) {
    setNodeA(QNodeA, gate);
}
