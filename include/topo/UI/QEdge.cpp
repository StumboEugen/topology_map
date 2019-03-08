//
// Created by stumbo on 18-10-16.
//

#include "QEdge.h"
#include "QNode.h"

#include "topo/Topo.h"

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QMessageBox>

#include "topoui.h"

extern TopoUI * bigBrother;

QEdge::QEdge(TopoEdge *edge, QNode *QNodeA, QNode *QNodeB)
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

QEdge::QEdge(QNode *QNodeA, uint8_t gate) {
    setNodeA(QNodeA, gate);
    setFlag(ItemIsSelectable);
    setZValue(1);
}

QRectF QEdge::boundingRect() const {
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

void QEdge::setRelatedEdgeTOPO(TopoEdge *relatedEdgeTOPO) {
    QEdge::relatedEdgeTOPO = relatedEdgeTOPO;
    length = QLineF(0, 0, relatedEdgeTOPO->getOdomX(), relatedEdgeTOPO->getOdomY()).length();
}

void QEdge::setNodeA(QNode *QNodeA, uint8_t gateA) {
    auto edgeLine = line();
    edgeLine.setP1(QNodeA->posOfExitInScene(gateA));
    setLine(edgeLine);
    QEdge::QNodeA = QNodeA;
    QEdge::gateA = gateA;
    if (QNodeB != nullptr) {
        registerAtBothNodes();
    }
}

void QEdge::setNodeB(QNode *QNodeB, uint8_t gateB) {
    auto edgeLine = line();
    edgeLine.setP1(QNodeB->posOfExitInScene(gateB));
    setLine(edgeLine);
    QEdge::QNodeB = QNodeB;
    QEdge::gateB = gateB;
    if (QNodeA != nullptr) {
        registerAtBothNodes();
    }
}

void
QEdge::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    if (QNodeA != nullptr && QNodeB != nullptr) {
        const QPointF & pointOfExitA = QNodeA->posOfExitInScene(gateA);
        const QPointF & pointOfExitB = QNodeB->posOfExitInScene(gateB);
        setLine({pointOfExitA, pointOfExitB});
        if (QNodeA->flags() & QGraphicsItem::ItemIsMovable) {
            refreshOdom();
        }
    }
    if (option->state & QStyle::State_Selected) {
        this->setPen(QPen(Qt::black, 7));
    } else {
        this->setPen(QPen(Qt::black, 3));
    }
    QGraphicsLineItem::paint(painter, option, widget);
}

void QEdge::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) {
    QGraphicsItem::mouseDoubleClickEvent(event);
    showMsg();
}

QPainterPath QEdge::shape() const {
//    const QPointF & pointOfExitA = mapFromScene(QNodeA->posOfExitInScene(gateA));
//    const QPointF & pointOfExitB = mapFromScene(QNodeB->posOfExitInScene(gateB));
    QPainterPath path;
    path.moveTo(line().p1());
    path.lineTo(line().p2());
    QPainterPathStroker stroker;
    stroker.setWidth(30);
    return stroker.createStroke(path);
}

void QEdge::focusInEvent(QFocusEvent *event) {
    QGraphicsItem::focusInEvent(event);
    cout << "focus" << endl;
}

void QEdge::focusOutEvent(QFocusEvent *event) {
    QGraphicsItem::focusOutEvent(event);
    cout << "out foc" << endl;
}

void QEdge::setLength(double length) {
    QEdge::length = length;
    showMsg();
}

void QEdge::refreshOdom() {
    double odomx = line().dx() / METER_TO_PIXLE;
    double odomy = -line().dy() / METER_TO_PIXLE;
    if (abs(odomx) < 0.01) odomx = 0.0;
    if (abs(odomy) < 0.01) odomy = 0.0;
    relatedEdgeTOPO->setOdomDataDirectly(odomx, odomy, 0); //TODO
    length = line().length() / METER_TO_PIXLE;
//    showMsg();
}

void QEdge::showMsg() {
    bigBrother->setMsg("Edge Len: " + QString::number(length));
    bigBrother->appendMsg("OdomX:" + QString::number(relatedEdgeTOPO->getOdomX()));
    bigBrother->appendMsg("OdomY:" + QString::number(relatedEdgeTOPO->getOdomY()));
}

void QEdge::registerAtBothNodes() {
    QNodeA->setQEdge(gateA, this);
    QNodeB->setQEdge(gateB, this);
}

//QEdge::~QEdge() {
//    //TODO may need to unregister
//}

QNode *QEdge::getAnotherNode(QNode * node) {
    if (node == QNodeA) {
        return QNodeB;
    }
    else if (node == QNodeB) {
        return QNodeA;
    } else {
        return nullptr;
    }
}

TopoEdge *QEdge::getRelatedEdgeTOPO() const {
    return relatedEdgeTOPO;
}

