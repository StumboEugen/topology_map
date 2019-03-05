//
// Created by stumbo on 18-10-16.
//

#include "QGI_Edge.h"
#include "QGI_Node.h"

#include "topo/Topo.h"

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QMessageBox>

#include "topoui.h"

extern TopoUI * bigBrother;

QGI_Edge::QGI_Edge(TopoEdge *edge, QGI_Node *QNodeA, QGI_Node *QNodeB)
{
    // this means that constructor is called by the build mode
    if (edge != nullptr) {
        setRelatedEdgeTOPO(edge);
        if (edge->getNodeA() == QNodeA->getRelatedNodeTOPO()) {
            setNodeA(QNodeA, edge->getGateA());
            setNodeB(QNodeB, edge->getGateB());
        }
        else if (edge->getNodeB() == QNodeA->getRelatedNodeTOPO()) {
            setNodeA(QNodeB, edge->getGateA());
            setNodeB(QNodeA, edge->getGateB());
        }
    }

    setFlag(ItemIsSelectable);
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
    length = QLineF(0, 0, relatedEdgeTOPO->getOdomX(), relatedEdgeTOPO->getOdomY()).length();
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
        refreshOdom();
    }
    if (option->state & QStyle::State_Selected) {
        this->setPen(QPen(Qt::black, 7));
    } else {
        this->setPen(QPen(Qt::black, 3));
    }
    QGraphicsLineItem::paint(painter, option, widget);
}

QGI_Edge::QGI_Edge(QGI_Node *QNodeA, uint8_t gate) {
    setNodeA(QNodeA, gate);
    setFlag(ItemIsSelectable);
    setZValue(1);
}

void QGI_Edge::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) {
    QGraphicsItem::mouseDoubleClickEvent(event);
    showMsg();
}

QPainterPath QGI_Edge::shape() const {
//    const QPointF & pointOfExitA = mapFromScene(QNodeA->posOfExitInScene(gateA));
//    const QPointF & pointOfExitB = mapFromScene(QNodeB->posOfExitInScene(gateB));
    QPainterPath path;
    path.moveTo(line().p1());
    path.lineTo(line().p2());
    QPainterPathStroker stroker;
    stroker.setWidth(30);
    return stroker.createStroke(path);
}

void QGI_Edge::focusInEvent(QFocusEvent *event) {
    QGraphicsItem::focusInEvent(event);
    cout << "focus" << endl;
}

void QGI_Edge::focusOutEvent(QFocusEvent *event) {
    QGraphicsItem::focusOutEvent(event);
    cout << "out foc" << endl;
}

void QGI_Edge::setLength(double length) {
    QGI_Edge::length = length;
    showMsg();
}

void QGI_Edge::refreshOdom() {
    double odomx = line().dx() / METER_TO_PIXLE;
    double odomy = -line().dy() / METER_TO_PIXLE;
    if (abs(odomx) < 0.01) odomx = 0.0;
    if (abs(odomy) < 0.01) odomy = 0.0;
    relatedEdgeTOPO->setOdomDataDirectly(odomx, odomy);
    length = line().length() / METER_TO_PIXLE;
//    showMsg();
}

void QGI_Edge::showMsg() {
    bigBrother->setMsg("Edge Len: " + QString::number(length));
    bigBrother->appendMsg("OdomX:" + QString::number(relatedEdgeTOPO->getOdomX()));
    bigBrother->appendMsg("OdomY:" + QString::number(relatedEdgeTOPO->getOdomY()));
}
