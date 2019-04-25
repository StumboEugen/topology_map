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

    drawingIns = new NodeInstance(false);
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

    //TODO ARRANGE THIS
    // this is caused by the size problem
    if (drawingLine) {
        drawingLine = false;
        for (const auto & exit : drawingIns->getExits()) {
            QPointF point{exit.getPosX(), -exit.getPosY()};
            QLineF line{{0,0}, point};
            line.setLength(QNODE_CIRCLE_SIZE);
            if (QLineF{line.p2(), theDrawingLine->line().p2()}.length() < 10) {
                scene()->removeItem(theDrawingLine);
                return;
            }
        }
        drawingIns->addExit( theDrawingLine->line().x2() / QNODE_CIRCLE_SIZE,
                            -theDrawingLine->line().y2() / QNODE_CIRCLE_SIZE,
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
    QLineF dir{{0,0}, {QNODE_CIRCLE_SIZE, 0}};
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

