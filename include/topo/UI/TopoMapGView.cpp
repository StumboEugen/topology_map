//
// Created by stumbo on 18-10-15.
//

#include <QWheelEvent>
#include <QMouseEvent>
#include <QGraphicsItem>
#include <QDebug>

#include "TopoMapGView.h"
#include "QNode.h"
#include "QEdge.h"
#include "topoui.h"
#include "UITOOLS.h"



extern UIMode CURRENT_MODE;

void TopoMapGView::wheelEvent(QWheelEvent *event) {
    if (event->modifiers() & Qt::ControlModifier) {
        auto curTrans = transform();
        auto curScale = curTrans.m11();
        if (event->delta() > 0 && curScale <= SCALE_MAX) {
            curTrans.scale(SCALE_TIME, SCALE_TIME);
        } 
        else if (event->delta() < 0 && curScale >= 1 / SCALE_MAX) {
            curTrans.scale(1 / SCALE_TIME, 1 / SCALE_TIME);
        }
        setMatrix(curTrans.toAffine());
    } else {
        QGraphicsView::wheelEvent(event);
    }
}

void TopoMapGView::mousePressEvent(QMouseEvent *event) {
    QGraphicsView::mousePressEvent(event);
    const auto & clickPosInView = event->pos();
    const auto & clickPosInScene = mapToScene(clickPosInView);
    auto item = scene()->itemAt(clickPosInScene);

    if (item != nullptr) {

        if (auto nodeItem = dynamic_cast<QNode*>(item)) {
            switch (event->button()) {
                case Qt::MouseButton::LeftButton:
                    Q_EMIT QGI_Node_clicked(nodeItem);
                    break;
                case Qt::MouseButton ::RightButton:
                    Q_EMIT rightClickOn_QGI_Node(nodeItem);
                    break;
                default:
                    break;
            }

            if (CURRENT_MODE == BUILD_MODE && drawingEdgeMode) {
                const auto & clickPosInItem = item->mapFromScene(clickPosInScene);
                int exitNumber = nodeItem->whichExitIsAtPos(clickPosInItem);
                if (exitNumber != -1) {
                    if (nodeItem->getRelatedNodeTOPO()->getEdge(
                            static_cast<gateId>(exitNumber)) == nullptr) {

                        theDrawingEdge = new QEdge(nodeItem,
                                static_cast<uint8_t>(exitNumber));
                        isDrawingEdge = true;
                        const auto & p1 = theDrawingEdge->line().p1();
                        theDrawingEdge->setLine({p1, p1});
                        scene()->addItem(theDrawingEdge);
                        nodeAofDrawingEdge = nodeItem->getRelatedNodeTOPO();
                        exitAofDrawingEdge = static_cast<uint8_t>(exitNumber);
                    }
                }
            }
        }
    }
}

void TopoMapGView::mouseMoveEvent(QMouseEvent *event) {
    QGraphicsView::mouseMoveEvent(event);
    if (isDrawingEdge && drawingEdgeMode) {
        const auto & clickPos = mapToScene(event->pos());
        const auto & firstPos = theDrawingEdge->line().p1();
        theDrawingEdge->setLine({firstPos, clickPos});
    }
}

void TopoMapGView::mouseReleaseEvent(QMouseEvent *event) {
    QGraphicsView::mouseReleaseEvent(event);
    if (isDrawingEdge && drawingEdgeMode) {
        isDrawingEdge = false;
        const auto & clickPosInScene = mapToScene(event->pos());
        auto clickedItem = scene()->itemAt(clickPosInScene);
        if (clickedItem != nullptr) {
            if (const auto & nodeItem = dynamic_cast<QNode*>(clickedItem)) {
                const auto & clickPosInItem = nodeItem->mapFromScene(clickPosInScene);
                int exitNumber = nodeItem->whichExitIsAtPos(clickPosInItem);
                if (exitNumber > -1) {
                    if (nodeItem->getRelatedNodeTOPO()->getEdge(
                            static_cast<gateId>(exitNumber)) == nullptr) {
                        theDrawingEdge->setNodeB(nodeItem, static_cast<uint8_t>(exitNumber));
                        auto * edge = new TopoEdge(nodeAofDrawingEdge, exitAofDrawingEdge,
                                                   nodeItem->getRelatedNodeTOPO(),
                                                   static_cast<uint8_t>(exitNumber));
//                        edge->registerAtNodes();
                        theDrawingEdge->setRelatedEdgeTOPO(edge);
                        Q_EMIT newEdgeConnected(edge);
//                        const auto & firstPos = theDrawingEdge->line().p1();
//                        theDrawingEdge->setLine(
//                                {firstPos, nodeItem->posOfExitInScene(exitNumber)});
                        return;
                    }
                }
            }
        }
        scene()->removeItem(theDrawingEdge);
        delete theDrawingEdge;
    }
}


void TopoMapGView::switch2DrawEdgeMode(bool start) {

    setNodesMoveable(!start);
    drawingEdgeMode = start;

    if (start) {
        setCursor(Qt::CrossCursor);
    } else {
        setCursor(Qt::ArrowCursor);
    }
}

void TopoMapGView::setNodesMoveable(bool move) {
    for (const auto & item :scene()->items()) {
        if (auto nodeItem = dynamic_cast<QNode*>(item)) {
            nodeItem->setFlag(QGraphicsItem::ItemIsMovable, move);
        }
    }
}

