//
// Created by stumbo on 18-10-15.
//

#include <QWheelEvent>
#include <QMouseEvent>
#include <QGraphicsItem>
#include <QDebug>

#include "TopoMapGView.h"
#include "QGI_Node.h"
#include "topoui.h"

#define SCALE_TIME 1.2
#define SCALE_MAX 4.0

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
    auto viewPos = event->pos();
//    qDebug() << "viewPos" << viewPos;
    auto scenePos = mapToScene(viewPos);
//    qDebug() << "scenePos" << scenePos;
    auto item = scene()->itemAt(scenePos);
    if (item != nullptr) {
        auto itemPos = item->mapFromScene(scenePos);
//        qDebug() << "itemPos" << itemPos << "\n";
        if (auto nodeItem = dynamic_cast<QGI_Node*>(item)) {
            Q_EMIT QGI_Node_clicked(nodeItem->getRelatedNodeTOPO());
        }
    }
}

void TopoMapGView::drawingEdge(bool start) {

    setNodesMoveable(!start);
    edgeDrawingMode = start;

    if (start) {
        setCursor(Qt::CrossCursor);
    } else {
        setCursor(Qt::ArrowCursor);
    }
}

void TopoMapGView::setNodesMoveable(bool move) {
    for (const auto & item :scene()->items()) {
        if (auto nodeItem = dynamic_cast<QGI_Node*>(item)) {
            nodeItem->setFlag(QGraphicsItem::ItemIsMovable, move);
        }
    }
}

