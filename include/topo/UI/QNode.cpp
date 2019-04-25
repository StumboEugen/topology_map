//
// Created by stumbo on 18-10-16.
//

#include "QNode.h"

#include <QPainter>
#include <QColor>
#include <QDebug>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsSceneHoverEvent>

#include "UITOOLS.h"
#include "topoui.h"
#include "QEdge.h"

extern TopoUI * bigBrother;

void
QNode::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    static QRect rect{-RECT_SIZE_HALF, -RECT_SIZE_HALF,
                      RECT_SIZE_HALF * 2, RECT_SIZE_HALF * 2};
    if (drawDetail) {
        QPainterPath path;
        path.addEllipse(- QNODE_CIRCLE_SIZE / 2,- QNODE_CIRCLE_SIZE / 2,
                        QNODE_CIRCLE_SIZE, QNODE_CIRCLE_SIZE);
        path.setFillRule(Qt::WindingFill);
        painter->setBrush(Qt::yellow);
        QPen pen = painter->pen();
        if (option->state & QStyle::State_Selected) {
            pen.setWidth(2);
        } else {
            pen.setWidth(1);
        }
        painter->setPen(pen);
        painter->drawPath(path);
        const auto & ins = relatedNodeTOPO->getInsCorrespond();
        const auto & exits = ins->getExits();
        for (int i = 0; i < exits.size(); i++) {
            const auto & exit = exits[i];
            QPointF exitGatePoint{exit.getPosX(), -exit.getPosY()};
            QLineF line{{0,0}, exitGatePoint};
            line.setLength(QNODE_CIRCLE_SIZE);
            painter->drawLine(line);

            if (realTimeMode && currentExitHoverOn == i) {
                painter->save();
                painter->setBrush(Qt::gray);
                painter->drawEllipse(posOfExitInItem(i), 5, 5);
                painter->restore();
            }
//            painter->rotate(-rotation);
            painter->drawText(line.p2(), QString::number(i));
//            painter->rotate(rotation);

            // stop drawing dir
//            QPointF gateDir{sin(exit.getOutDir() * DEG2RAD), -cos(exit.getOutDir() * DEG2RAD)};
//            gateDir *= METER_TO_PIXLE * 0.5;
//            painter->save();
//            painter->setPen(Qt::red);
//            painter->drawLine(exitGatePoint, gateDir + exitGatePoint);
//            painter->restore();
        }
    } else {
        if (option->state & QStyle::State_Selected) {
            painter->setPen(Qt::red);
        } else {
            painter->setPen(Qt::darkRed);
        }
        painter->drawRect(rect.adjusted(-1, -1, 0, 0));
//        painter->fillRect(rect, Qt::gray);
    }
}

QRectF QNode::boundingRect() const {
    if (drawDetail) {
        return {-100, -100, 200, 200};
    } else {
        return outline();
    }
}

QNode::QNode(TopoNode * node)
        :relatedNodeTOPO(node),
         connectedEdges(node->getInsCorrespond()->sizeOfExits())
{
    this->setZValue(2);
    setFlag(ItemIsSelectable);
}

int QNode::type() const {
    return QGI_NODE_TYPE;
}

QPainterPath QNode::shape() const {
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

int QNode::whichExitIsAtPos(const QPointF & pos) {
    const auto & exits = relatedNodeTOPO->getInsCorrespond()->getExits();
    for (int i = 0; i < exits.size(); i++) {
        if (QLineF(posOfExitInItem(i), pos).length() < 10) {
            return i;
        }
    }
    return -1;
}

QPointF QNode::posOfExitInItem(int i) {
    if (i < 0 || i >= relatedNodeTOPO->getEdgeConnected().size()) {
        cerr << "[QNode::posOfExitInItem]" << endl;
        throw;
    }
    const auto & exits = relatedNodeTOPO->getInsCorrespond()->getExits();
    QPointF exitPos{exits[i].getPosX(), -exits[i].getPosY()};
    QLineF line{{0,0}, exitPos};
    line.setLength(QNODE_CIRCLE_SIZE);   // TODO now we ommit the len info
//    line.setAngle(line.angle() + rotation);
    return line.p2();
}

QPointF QNode::posOfExitInScene(int index) {
    return mapToScene(posOfExitInItem(index));
}

void QNode::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) {
    QGraphicsItem::mouseDoubleClickEvent(event);
    bigBrother->setMsg("POS: " +
                       QString::number(pos().x()) + " : " +
                       QString::number(pos().y()));
}

void QNode::setQEdge(int index, QEdge *edge) {
    if (connectedEdges[index] == nullptr) {
        connectedEdges[index] = edge;
    } else {
        cerr << "QNode::setQEdge you are trying to erase exist QEdge!" << endl;
    }
}

int QNode::getExitNums() {
    return relatedNodeTOPO->getInsCorrespond()->sizeOfExits();
}

QNode *QNode::getQNodeAtExit(int index) {
    auto & edge = connectedEdges[index];

    if (edge != nullptr) {
        return connectedEdges[index]->getAnotherNode(this);
    } else {
        return nullptr;
    }
}

void QNode::setRotation(double angle) {
    QGraphicsItem::setRotation(angle);
    rotation = angle;
}

void QNode::hoverMoveEvent(QGraphicsSceneHoverEvent *event) {
    QGraphicsItem::hoverMoveEvent(event);
    if (realTimeMode) {
        auto thisTime = whichExitIsAtPos(event->pos());
        if (thisTime != currentExitHoverOn) {
            currentExitHoverOn = thisTime;
            update();
        }
    }
}

void QNode::hoverLeaveEvent(QGraphicsSceneHoverEvent *event) {
    QGraphicsItem::hoverLeaveEvent(event);
    if (realTimeMode) {
        currentExitHoverOn = -1;
        update();
    }
}

void QNode::mousePressEvent(QGraphicsSceneMouseEvent *event) {
    QGraphicsItem::mousePressEvent(event);
    if (realTimeMode) {
        if (event->button() == Qt::MouseButton::RightButton) {
            const auto exit = whichExitIsAtPos(event->pos());
            if (exit != -1) {
                bigBrother->realTimeMode_sendMoveCmd(exit);
            }
        }
    }
}
