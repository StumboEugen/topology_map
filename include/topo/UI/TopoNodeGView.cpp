//
// Created by stumbo on 18-10-17.
//

#include "TopoNodeGView.h"
#include "UITOOLS.h"

#include <QDebug>
#include <QMouseEvent>
#include <QLineF>

NodeInstance * TopoNodeGView::getTheDrawnInstance(){
    if (drawingIns == nullptr) {
        return nullptr;
    }
    drawingIns->completeAdding();
    auto temp = drawingIns;
    drawingIns = nullptr;
    return temp;
}

void TopoNodeGView::startDrawingIns() {
    delete drawingIns;
    scene()->clear();

    drawingIns = new NodeInstance();
    scene()->addItem(new QGraphicsRectItem(-90, -90, 180, 180));
}

void TopoNodeGView::mousePressEvent(QMouseEvent *event) {
    QGraphicsView::mousePressEvent(event);
    if (drawingIns != nullptr) {
        const auto & clickPoint = mapToScene(event->pos());
        theDrawingLine = new QGraphicsLineItem(getLineDrawingFromOrigin(clickPoint));
        drawingLine = true;
        scene()->addItem(theDrawingLine);
    }
}

void TopoNodeGView::mouseMoveEvent(QMouseEvent *event) {
    QGraphicsView::mouseMoveEvent(event);
    if (!drawingLine)
        return;
    const auto & clickPoint = mapToScene(event->pos());
    theDrawingLine->setLine(getLineDrawingFromOrigin(clickPoint));
}

void TopoNodeGView::mouseReleaseEvent(QMouseEvent *event) {
    QGraphicsView::mouseReleaseEvent(event);

    //TODO
    if (drawingLine) {
        drawingLine = false;
        drawingIns->addExit(drawLineStartPos.x() / METER_TO_PIXLE,
                            -drawLineStartPos.y() / METER_TO_PIXLE,
                            -theDrawingLine->line().angle() + 90);
    }
}

TopoNodeGView::~TopoNodeGView() {
    delete drawingIns;
}

QLineF TopoNodeGView::getLineDrawingFromOrigin(const QPointF point) {
    if (point.x() == 0 && point.y() == 0) {
        return {0,0,0,0};
    }
    QLineF dir{{0,0}, {1 * METER_TO_PIXLE, 0}};// len is 30 now
    double angle = atan2(point.y(), point.x());
    double index = angle / piHalf;
    if (index > 1.5 || index < -1.5) {
        dir.setAngle(180);
    }
    else if (index > 0.5) {
        dir.setAngle(-90);
    }
    else if (index < -0.5) {
        dir.setAngle(90);
    }
    return dir;
}

