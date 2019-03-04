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
        QPainterPath path;
        path.addEllipse(-20,-20,40,40);
        path.setFillRule(Qt::WindingFill);
        painter->setBrush(Qt::yellow);
        painter->drawPath(path);
        const auto & ins = relatedNodeTOPO->getInsCorrespond();
        for (const auto & exit: ins->getExits()) {
            QPointF exitGatePoint{exit.getPosX() * METER_TO_PIXLE,
                                  -exit.getPosY() * METER_TO_PIXLE};
            painter->drawLine({0,0}, exitGatePoint);
            // stop drawing dir
//            QPointF gateDir{sin(exit.getOutDir() * DEG2RAD), -cos(exit.getOutDir() * DEG2RAD)};
//            gateDir *= METER_TO_PIXLE * 0.5;
//            painter->save();
//            painter->setPen(Qt::red);
//            painter->drawLine(exitGatePoint, gateDir + exitGatePoint);
//            painter->restore();
        }
    } else {
//        if (option->state & QStyle::State_Selected) {
            painter->setPen(Qt::red);
            painter->drawRect(rect.adjusted(-1, -1, 0, 0));
//        }
//        painter->fillRect(rect, Qt::gray);
    }
}

QRectF QGI_Node::boundingRect() const {
    if (drawDetail) {
        return {-100, -100, 200, 200};
    } else {
        return outline();
    }
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

QPainterPath QGI_Node::shape() const {
    QPainterPath path;
//    if (drawDetail) {
//        path.addEllipse(-20,-20,40,40);
//        path.setFillRule(Qt::WindingFill);
//    } else {
//        path.addRect(outline());
//    }
    path.addRect(outline());
    return path;
}

int QGI_Node::whichExitIsAtPos(const QPointF & pos) {
    const auto & exits = relatedNodeTOPO->getInsCorrespond()->getExits();
    for (int i = 0; i < exits.size(); i++) {
        if (QLineF(posOfExitInItem(i), pos).length() < 10) {
            return i;
        }
    }
    return -1;
}

QPointF QGI_Node::posOfExitInItem(int i) {
    if (i < 0 || i >= relatedNodeTOPO->getEdgeConnected().size()) {
        throw;
    }
    const auto & exits = relatedNodeTOPO->getInsCorrespond()->getExits();
    QPointF exitPos{exits[i].getPosX(), -exits[i].getPosY()};
    exitPos *= METER_TO_PIXLE;
    return exitPos;
}

QPointF QGI_Node::posOfExitInScene(int index) {
    return mapToScene(posOfExitInItem(index));
}
