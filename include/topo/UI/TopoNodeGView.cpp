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
        drawLineStartPos = mapToScene(event->pos());
        qDebug() << "start line" << drawLineStartPos;
        if (drawLineStartPos.x() > 60 || drawLineStartPos.x() < -60
            || drawLineStartPos.y() > 60 || drawLineStartPos.y() < -60) {
            return;
        }
        drawingLine = true;
        theDrawingLine = new QGraphicsLineItem(drawLineStartPos.x(), drawLineStartPos.y()
                , drawLineStartPos.x(), drawLineStartPos.y() + 1 * METER_TO_PIXLE);
        scene()->addItem(theDrawingLine);
    }
}

void TopoNodeGView::mouseMoveEvent(QMouseEvent *event) {
    QGraphicsView::mouseMoveEvent(event);
    if (!drawingLine)
        return;
    auto pos = mapToScene(event->pos());
    QLineF dir{drawLineStartPos, pos};
    auto unitDir = dir.unitVector();
    unitDir.setLength(1 * METER_TO_PIXLE);
    theDrawingLine->setLine(unitDir);
}

void TopoNodeGView::mouseReleaseEvent(QMouseEvent *event) {
    QGraphicsView::mouseReleaseEvent(event);
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
